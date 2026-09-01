/*
	Copyright 2025

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "encoder_interface.h"

#if ENCODER_TYPE == ENCODER_TYPE_QUADRATURE || ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "QUAD_ENCODER";

// Quadrature encoder state
typedef struct {
    float angle_rad;
    float angle_deg;
    float velocity_rad_s;
    float velocity_deg_s;
    volatile int32_t pulse_count;
    int32_t last_pulse_count;
    uint32_t timestamp_us;
    uint32_t last_update_us;
    bool valid;
    uint32_t error_count;
    float radians_per_pulse;
    volatile uint8_t last_state;
} quad_encoder_state_t;

static quad_encoder_state_t quad_state = {0};

#define VELOCITY_FILTER_ALPHA 0.1f

// Quadrature decoder lookup table for 2x decoding
// [current_state][last_state] = count_change
static const int8_t quad_table[4][4] = {
    // Last: 00  01  10  11     Current
    {  0, -1,  1,  0 }, // 00
    {  1,  0,  0, -1 }, // 01
    { -1,  0,  0,  1 }, // 10
    {  0,  1, -1,  0 }  // 11
};

// GPIO interrupt handler for quadrature decoding
static void IRAM_ATTR quad_gpio_isr_handler(void* arg) {
    uint8_t a_state = gpio_get_level(ENCODER_A_PIN);
    uint8_t b_state = gpio_get_level(ENCODER_B_PIN);
    uint8_t current_state = (a_state << 1) | b_state;
    
    // Look up count change in quadrature table
    int8_t count_change = quad_table[current_state][quad_state.last_state];
    quad_state.pulse_count += count_change;
    quad_state.last_state = current_state;
}

// Initialize quadrature encoder
static bool quad_encoder_init(void) {
    ESP_LOGI(TAG, "Initializing quadrature encoder on GPIO %d/%d (PPR: %d)", 
             ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_PPR);
    
    // Calculate radians per pulse (4x encoding: 4 edges per line, but using 2x with GPIO interrupts)
    quad_state.radians_per_pulse = (2.0f * M_PI) / (ENCODER_PPR * 2.0f);
    
    esp_err_t ret = gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO A direction setup failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO B direction setup failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO A pull setup failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO B pull setup failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = gpio_set_intr_type(ENCODER_A_PIN, GPIO_INTR_ANYEDGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO A interrupt type setup failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = gpio_set_intr_type(ENCODER_B_PIN, GPIO_INTR_ANYEDGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO B interrupt type setup failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = gpio_intr_disable(ENCODER_A_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO A interrupt disable failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = gpio_intr_disable(ENCODER_B_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO B interrupt disable failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Add ISR handlers for both pins
    ret = gpio_isr_handler_add(ENCODER_A_PIN, quad_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR handler add for A pin failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = gpio_isr_handler_add(ENCODER_B_PIN, quad_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR handler add for B pin failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize state
    quad_state.pulse_count = 0;
    quad_state.last_pulse_count = 0;
    quad_state.valid = true;
    quad_state.error_count = 0;
    quad_state.last_update_us = esp_timer_get_time();
    
    // Read initial state
    uint8_t a_state = gpio_get_level(ENCODER_A_PIN);
    uint8_t b_state = gpio_get_level(ENCODER_B_PIN);
    quad_state.last_state = (a_state << 1) | b_state;
    
    ESP_LOGI(TAG, "Quadrature encoder initialized successfully");
    return true;
}

// Update quadrature encoder data
static bool quad_encoder_update(void) {
    uint32_t current_time_us = esp_timer_get_time();
    
    // Read current pulse count (volatile, so this is thread-safe)
    int32_t current_count = quad_state.pulse_count;
    
    // Calculate angle
    quad_state.angle_rad = current_count * quad_state.radians_per_pulse;
    quad_state.angle_deg = encoder_rad_to_deg(quad_state.angle_rad);
    
    // Calculate velocity
    uint32_t dt_us = current_time_us - quad_state.last_update_us;
    if (dt_us > 0 && quad_state.last_update_us > 0) {
        float dt_s = dt_us / 1000000.0f;
        int32_t count_diff = current_count - quad_state.last_pulse_count;
        float angle_diff = count_diff * quad_state.radians_per_pulse;
        
        float inst_velocity = angle_diff / dt_s;
        
        // Low-pass filter for velocity
        quad_state.velocity_rad_s = VELOCITY_FILTER_ALPHA * inst_velocity + 
                                   (1.0f - VELOCITY_FILTER_ALPHA) * quad_state.velocity_rad_s;
        quad_state.velocity_deg_s = encoder_rad_to_deg(quad_state.velocity_rad_s);
    }
    
    quad_state.last_pulse_count = current_count;
    quad_state.last_update_us = current_time_us;
    quad_state.timestamp_us = current_time_us;
    
    // Update global encoder data
    encoder_data.angle_rad = quad_state.angle_rad;
    encoder_data.angle_deg = quad_state.angle_deg;
    encoder_data.velocity_rad_s = quad_state.velocity_rad_s;
    encoder_data.velocity_deg_s = quad_state.velocity_deg_s;
    encoder_data.timestamp_us = current_time_us;
    encoder_data.valid = quad_state.valid;
    encoder_data.error_count = quad_state.error_count;
    
    return true;
}

// Get angle in radians
static float quad_encoder_get_angle_rad(void) {
    return quad_state.angle_rad;
}

// Get angle in degrees
static float quad_encoder_get_angle_deg(void) {
    return quad_state.angle_deg;
}

// Get velocity in rad/s
static float quad_encoder_get_velocity_rad_s(void) {
    return quad_state.velocity_rad_s;
}

// Get velocity in deg/s
static float quad_encoder_get_velocity_deg_s(void) {
    return quad_state.velocity_deg_s;
}

// Check if data is valid
static bool quad_encoder_is_valid(void) {
    return quad_state.valid;
}

// Get error count
static uint32_t quad_encoder_get_error_count(void) {
    return quad_state.error_count;
}

// Reset error count
static void quad_encoder_reset_errors(void) {
    quad_state.error_count = 0;
}

// Get encoder type name
static const char* quad_encoder_get_type_name(void) {
    return "Quadrature";
}

// Quadrature encoder interface
const encoder_interface_t quad_encoder_interface = {
    .init = quad_encoder_init,
    .update = quad_encoder_update,
    .get_angle_rad = quad_encoder_get_angle_rad,
    .get_angle_deg = quad_encoder_get_angle_deg,
    .get_velocity_rad_s = quad_encoder_get_velocity_rad_s,
    .get_velocity_deg_s = quad_encoder_get_velocity_deg_s,
    .is_valid = quad_encoder_is_valid,
    .get_error_count = quad_encoder_get_error_count,
    .reset_errors = quad_encoder_reset_errors,
    .get_type_name = quad_encoder_get_type_name
};

#endif /* ENCODER_TYPE == ENCODER_TYPE_QUADRATURE */
