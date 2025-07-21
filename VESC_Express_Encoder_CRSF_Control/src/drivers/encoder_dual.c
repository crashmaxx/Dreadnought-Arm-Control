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

#if ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"

static const char *TAG = "DUAL_ENCODER";

// Dual encoder state combining PWM magnetic sensor and quadrature encoder  
typedef struct {
    // PWM sensor data
    float pwm_angle_rad;
    float pwm_angle_deg;
    uint32_t pwm_pulse_width_us;
    uint32_t pwm_last_pulse_us;
    uint32_t pwm_timestamp_us;
    bool pwm_valid;
    uint32_t pwm_error_count;
    
    // Quadrature encoder data (GPIO-based for simplicity)
    float quad_angle_rad;
    float quad_angle_deg;
    int32_t quad_pulse_count;
    volatile int32_t quad_count;
    float quad_radians_per_pulse;
    bool quad_valid;
    uint32_t quad_error_count;
    uint8_t last_a_state;
    uint8_t last_b_state;
    
    // Combined output
    float combined_angle_rad;
    float combined_angle_deg;
    float velocity_rad_s;
    float velocity_deg_s;
    uint32_t last_update_us;
    float last_angle_rad;
    bool valid;
    uint32_t total_error_count;
    
    // Offset for combining encoders
    float pwm_offset_rad;
    bool offset_initialized;
} dual_encoder_state_t;

static dual_encoder_state_t dual_state = {0};

#define PWM_TIMEOUT_US 100000
#define PWM_MIN_PULSE_US 500
#define PWM_MAX_PULSE_US 2500
#define VELOCITY_FILTER_ALPHA 0.1f

// PWM GPIO interrupt handler
static void IRAM_ATTR pwm_gpio_isr_handler(void* arg) {
    uint32_t current_time_us = esp_timer_get_time();
    int gpio_level = gpio_get_level(ENCODER_PWM_PIN);
    
    if (gpio_level == 1) {
        // Rising edge
        dual_state.pwm_last_pulse_us = current_time_us;
    } else {
        // Falling edge
        if (dual_state.pwm_last_pulse_us > 0) {
            uint32_t pulse_width = current_time_us - dual_state.pwm_last_pulse_us;
            
            if (pulse_width >= PWM_MIN_PULSE_US && pulse_width <= PWM_MAX_PULSE_US) {
                dual_state.pwm_pulse_width_us = pulse_width;
                dual_state.pwm_timestamp_us = current_time_us;
                dual_state.pwm_valid = true;
            } else {
                dual_state.pwm_error_count++;
            }
        }
    }
}

// Quadrature encoder GPIO interrupt handlers
static void IRAM_ATTR quad_a_gpio_isr_handler(void* arg) {
    uint8_t a_state = gpio_get_level(ENCODER_A_PIN);
    uint8_t b_state = gpio_get_level(ENCODER_B_PIN);
    
    if (a_state != dual_state.last_a_state) {
        if (a_state) {
            // Rising edge on A
            if (b_state) {
                dual_state.quad_count--;
            } else {
                dual_state.quad_count++;
            }
        } else {
            // Falling edge on A
            if (b_state) {
                dual_state.quad_count++;
            } else {
                dual_state.quad_count--;
            }
        }
        dual_state.last_a_state = a_state;
    }
}

static void IRAM_ATTR quad_b_gpio_isr_handler(void* arg) {
    uint8_t a_state = gpio_get_level(ENCODER_A_PIN);
    uint8_t b_state = gpio_get_level(ENCODER_B_PIN);
    
    if (b_state != dual_state.last_b_state) {
        if (b_state) {
            // Rising edge on B
            if (a_state) {
                dual_state.quad_count++;
            } else {
                dual_state.quad_count--;
            }
        } else {
            // Falling edge on B  
            if (a_state) {
                dual_state.quad_count--;
            } else {
                dual_state.quad_count++;
            }
        }
        dual_state.last_b_state = b_state;
    }
}

// Initialize dual hybrid encoder
static bool dual_encoder_init(void) {
    ESP_LOGI(TAG, "Initializing dual hybrid encoder (PWM: %d, Quad: %d/%d)", 
             ENCODER_PWM_PIN, ENCODER_A_PIN, ENCODER_B_PIN);
    
    // Initialize PWM encoder
    gpio_config_t pwm_io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_PWM_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    esp_err_t ret = gpio_config(&pwm_io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM GPIO config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Add PWM ISR handler
    ret = gpio_isr_handler_add(ENCODER_PWM_PIN, pwm_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM GPIO ISR handler add failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize quadrature encoder with GPIO interrupts
    dual_state.quad_radians_per_pulse = (2.0f * M_PI) / (ENCODER_PPR * 4.0f);
    
    // Configure quadrature A pin
    gpio_config_t quad_a_conf = {
        .pin_bit_mask = (1ULL << ENCODER_A_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    ret = gpio_config(&quad_a_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Quad A GPIO config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configure quadrature B pin
    gpio_config_t quad_b_conf = {
        .pin_bit_mask = (1ULL << ENCODER_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    ret = gpio_config(&quad_b_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Quad B GPIO config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Add quadrature ISR handlers
    ret = gpio_isr_handler_add(ENCODER_A_PIN, quad_a_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Quad A GPIO ISR handler add failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = gpio_isr_handler_add(ENCODER_B_PIN, quad_b_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Quad B GPIO ISR handler add failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize GPIO states
    dual_state.last_a_state = gpio_get_level(ENCODER_A_PIN);
    dual_state.last_b_state = gpio_get_level(ENCODER_B_PIN);
    dual_state.quad_count = 0;
    
    // Initialize state
    dual_state.pwm_valid = false;
    dual_state.quad_valid = true;
    dual_state.valid = false;
    dual_state.offset_initialized = false;
    dual_state.pwm_error_count = 0;
    dual_state.quad_error_count = 0;
    dual_state.last_update_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "Dual hybrid encoder initialized successfully");
    return true;
}

// Update dual encoder data
static bool dual_encoder_update(void) {
    uint32_t current_time_us = esp_timer_get_time();
    bool updated = false;
    
    // Update PWM sensor
    if (dual_state.pwm_valid && (current_time_us - dual_state.pwm_timestamp_us) > PWM_TIMEOUT_US) {
        dual_state.pwm_valid = false;
        ESP_LOGW(TAG, "PWM sensor timeout");
    }
    
    if (dual_state.pwm_valid && dual_state.pwm_pulse_width_us > 0) {
        // Convert PWM pulse to angle
        float pulse_range = ENCODER_PWM_MAX_US - ENCODER_PWM_MIN_US;
        float normalized = (float)(dual_state.pwm_pulse_width_us - ENCODER_PWM_MIN_US) / pulse_range;
        
        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;
        
        dual_state.pwm_angle_rad = normalized * 2.0f * M_PI;
        dual_state.pwm_angle_deg = encoder_rad_to_deg(dual_state.pwm_angle_rad);
        updated = true;
    }
    
    // Update quadrature encoder
    int32_t current_count = dual_state.quad_count;
    dual_state.quad_angle_rad = current_count * dual_state.quad_radians_per_pulse;
    dual_state.quad_angle_deg = encoder_rad_to_deg(dual_state.quad_angle_rad);
    updated = true;
    
    if (updated) {
        // Initialize offset between PWM and quadrature (like in Claw_Control)
        if (!dual_state.offset_initialized && dual_state.pwm_valid) {
            dual_state.pwm_offset_rad = dual_state.pwm_angle_rad;
            dual_state.offset_initialized = true;
            ESP_LOGI(TAG, "PWM offset initialized: %.3f rad (%.1f deg)", 
                     dual_state.pwm_offset_rad, encoder_rad_to_deg(dual_state.pwm_offset_rad));
        }
        
        // Combine encoders: use quadrature for relative movement, PWM for absolute reference
        if (dual_state.pwm_valid && dual_state.offset_initialized) {
            // Use PWM as absolute reference + quadrature for precision
            dual_state.combined_angle_rad = dual_state.quad_angle_rad + dual_state.pwm_offset_rad;
        } else {
            // Fall back to quadrature only
            dual_state.combined_angle_rad = dual_state.quad_angle_rad;
        }
        
        dual_state.combined_angle_deg = encoder_rad_to_deg(dual_state.combined_angle_rad);
        
        // Calculate velocity
        uint32_t dt_us = current_time_us - dual_state.last_update_us;
        if (dt_us > 0 && dual_state.last_update_us > 0) {
            float dt_s = dt_us / 1000000.0f;
            float angle_diff = dual_state.combined_angle_rad - dual_state.last_angle_rad;
            
            // Handle wrap-around
            if (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
            if (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
            
            float inst_velocity = angle_diff / dt_s;
            
            // Filter velocity
            dual_state.velocity_rad_s = VELOCITY_FILTER_ALPHA * inst_velocity + 
                                       (1.0f - VELOCITY_FILTER_ALPHA) * dual_state.velocity_rad_s;
            dual_state.velocity_deg_s = encoder_rad_to_deg(dual_state.velocity_rad_s);
        }
        
        dual_state.last_angle_rad = dual_state.combined_angle_rad;
        dual_state.last_update_us = current_time_us;
        dual_state.valid = true;
        dual_state.total_error_count = dual_state.pwm_error_count + dual_state.quad_error_count;
        
        // Update global encoder data
        encoder_data.angle_rad = dual_state.combined_angle_rad;
        encoder_data.angle_deg = dual_state.combined_angle_deg;
        encoder_data.velocity_rad_s = dual_state.velocity_rad_s;
        encoder_data.velocity_deg_s = dual_state.velocity_deg_s;
        encoder_data.timestamp_us = current_time_us;
        encoder_data.valid = dual_state.valid;
        encoder_data.error_count = dual_state.total_error_count;
    }
    
    return updated;
}

// Dual encoder interface functions
static float dual_encoder_get_angle_rad(void) { return dual_state.combined_angle_rad; }
static float dual_encoder_get_angle_deg(void) { return dual_state.combined_angle_deg; }
static float dual_encoder_get_velocity_rad_s(void) { return dual_state.velocity_rad_s; }
static float dual_encoder_get_velocity_deg_s(void) { return dual_state.velocity_deg_s; }
static bool dual_encoder_is_valid(void) { return dual_state.valid; }
static uint32_t dual_encoder_get_error_count(void) { return dual_state.total_error_count; }

static void dual_encoder_reset_errors(void) {
    dual_state.pwm_error_count = 0;
    dual_state.quad_error_count = 0;
    dual_state.total_error_count = 0;
}

static const char* dual_encoder_get_type_name(void) {
    return "Dual Hybrid (PWM+Quad)";
}

// Dual encoder interface
const encoder_interface_t dual_encoder_interface = {
    .init = dual_encoder_init,
    .update = dual_encoder_update,
    .get_angle_rad = dual_encoder_get_angle_rad,
    .get_angle_deg = dual_encoder_get_angle_deg,
    .get_velocity_rad_s = dual_encoder_get_velocity_rad_s,
    .get_velocity_deg_s = dual_encoder_get_velocity_deg_s,
    .is_valid = dual_encoder_is_valid,
    .get_error_count = dual_encoder_get_error_count,
    .reset_errors = dual_encoder_reset_errors,
    .get_type_name = dual_encoder_get_type_name
};

#endif /* ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID */
