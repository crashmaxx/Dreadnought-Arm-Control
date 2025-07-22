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

#if ENCODER_TYPE == ENCODER_TYPE_PWM_MAGNETIC || ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PWM_ENCODER";

// PWM encoder state
typedef struct {
    float angle_rad;
    float angle_deg;
    float velocity_rad_s;
    float velocity_deg_s;
    uint32_t last_pulse_us;
    uint32_t pulse_width_us;
    uint32_t timestamp_us;
    uint32_t last_update_us;
    bool valid;
    uint32_t error_count;
    uint32_t timeout_count;
    float last_angle_rad;
} pwm_encoder_state_t;

static pwm_encoder_state_t pwm_state = {0};

// PWM capture configuration
#define PWM_TIMEOUT_US 200000  // 200ms timeout (increased for stability)
#define PWM_MIN_PULSE_US 1     // Minimum 1μs pulse to filter noise (even though encoder can go to 0)
#define PWM_MAX_PULSE_US ENCODER_PWM_MAX_US   // Use board config maximum
#define VELOCITY_FILTER_ALPHA 0.1f  // Low-pass filter for velocity

// GPIO interrupt handler for PWM capture
static void IRAM_ATTR pwm_gpio_isr_handler(void* arg) {
    uint32_t current_time_us = esp_timer_get_time();
    int gpio_level = gpio_get_level(ENCODER_PWM_PIN);
    
    if (gpio_level == 1) {
        // Rising edge - start of pulse
        pwm_state.last_pulse_us = current_time_us;
    } else {
        // Falling edge - end of pulse
        if (pwm_state.last_pulse_us > 0) {
            uint32_t pulse_width = current_time_us - pwm_state.last_pulse_us;
            
            // Validate pulse width - clamp high values to max instead of rejecting
            // This allows readings >PWM_MAX_US to be treated as 359° instead of invalid
            if (pulse_width >= PWM_MIN_PULSE_US) {
                // Clamp to maximum range instead of rejecting
                if (pulse_width > PWM_MAX_PULSE_US) {
                    pulse_width = PWM_MAX_PULSE_US;
                    ESP_LOGD(TAG, "Clamped pulse to max: %lu μs", pulse_width);
                }
                pwm_state.pulse_width_us = pulse_width;
                pwm_state.timestamp_us = current_time_us;
                pwm_state.valid = true;
                ESP_LOGD(TAG, "Valid pulse: %lu μs", pulse_width);
            } else {
                // Only reject if below minimum (too short - likely noise)
                pwm_state.error_count++;
                ESP_LOGD(TAG, "Rejected short pulse: %lu μs", pulse_width);
            }
        }
    }
}

// Initialize PWM encoder
static bool pwm_encoder_init(void) {
    ESP_LOGI(TAG, "Initializing PWM magnetic encoder on GPIO %d", ENCODER_PWM_PIN);
    
    // Configure GPIO for input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_PWM_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Add ISR handler
    ret = gpio_isr_handler_add(ENCODER_PWM_PIN, pwm_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ISR handler add failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize state
    pwm_state.valid = false;
    pwm_state.error_count = 0;
    pwm_state.timeout_count = 0;
    pwm_state.last_update_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "PWM encoder initialized successfully");
    return true;
}

// Update PWM encoder data
static bool pwm_encoder_update(void) {
    uint32_t current_time_us = esp_timer_get_time();
    
    // Timeout function DISABLED - user prefers readings over timeout errors
    // Once we have a valid reading, we keep it until a new one arrives
    /*
    if (pwm_state.valid && (current_time_us - pwm_state.timestamp_us) > PWM_TIMEOUT_US) {
        pwm_state.valid = false;
        pwm_state.timeout_count++;
        ESP_LOGW(TAG, "PWM encoder timeout (count: %lu, last pulse: %lu μs ago)", 
                pwm_state.timeout_count, (current_time_us - pwm_state.timestamp_us));
    }
    */
    
    // Special case: Check if GPIO is constantly low (0V = 0 degrees)
    // Re-enabled with very conservative settings to detect true 0V only
    int gpio_level = gpio_get_level(ENCODER_PWM_PIN);
    static uint32_t low_start_time = 0;
    static bool was_low = false;
    static uint32_t last_valid_pulse_time = 0;
    
    // Track when we last had a valid PWM pulse
    if (pwm_state.valid && pwm_state.pulse_width_us > 0) {
        last_valid_pulse_time = current_time_us;
    }
    
    if (gpio_level == 0) {
        if (!was_low) {
            low_start_time = current_time_us;
            was_low = true;
        } else if (current_time_us - low_start_time > 500000) { // Low for >500ms = very likely constant 0V
            // Check if we haven't had any valid pulses for a very long time
            if (last_valid_pulse_time == 0 || (current_time_us - last_valid_pulse_time) > 600000) { // No pulse for >600ms
                // Only now treat as constant 0V (true 0 degrees)
                pwm_state.angle_rad = 0.0f;
                pwm_state.angle_deg = 0.0f;
                pwm_state.timestamp_us = current_time_us;
                pwm_state.valid = true;
                
                // Update global encoder data for 0 degrees
                encoder_data.angle_rad = 0.0f;
                encoder_data.angle_deg = 0.0f;
                encoder_data.velocity_rad_s = 0.0f;  // Assume stationary at 0
                encoder_data.velocity_deg_s = 0.0f;
                encoder_data.timestamp_us = current_time_us;
                encoder_data.valid = true;
                encoder_data.error_count = pwm_state.error_count;
                
                ESP_LOGI(TAG, "Detected constant 0V - encoder at 0 degrees");
                return true;
            }
        }
    } else {
        was_low = false;
    }
    
    if (pwm_state.valid && pwm_state.pulse_width_us > 0) {
        // Convert pulse width to angle
        // Map from ENCODER_PWM_MIN_US-ENCODER_PWM_MAX_US to 0-2π
        float pulse_range = ENCODER_PWM_MAX_US - ENCODER_PWM_MIN_US;
        float normalized = (float)(pwm_state.pulse_width_us - ENCODER_PWM_MIN_US) / pulse_range;
        
        // Only clamp lower bound (upper bound handled in ISR)
        if (normalized < 0.0f) normalized = 0.0f;
        // Note: normalized can now be exactly 1.0f for max angle (359°)
        
        pwm_state.angle_rad = normalized * 2.0f * M_PI;
        pwm_state.angle_deg = encoder_rad_to_deg(pwm_state.angle_rad);
        
        // Calculate velocity (with filtering)
        uint32_t dt_us = current_time_us - pwm_state.last_update_us;
        if (dt_us > 0 && pwm_state.last_update_us > 0) {
            float dt_s = dt_us / 1000000.0f;
            float angle_diff = pwm_state.angle_rad - pwm_state.last_angle_rad;
            
            // Handle angle wrap-around
            if (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
            if (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
            
            float inst_velocity = angle_diff / dt_s;
            
            // Low-pass filter for velocity
            pwm_state.velocity_rad_s = VELOCITY_FILTER_ALPHA * inst_velocity + 
                                      (1.0f - VELOCITY_FILTER_ALPHA) * pwm_state.velocity_rad_s;
            pwm_state.velocity_deg_s = encoder_rad_to_deg(pwm_state.velocity_rad_s);
        }
        
        pwm_state.last_angle_rad = pwm_state.angle_rad;
        pwm_state.last_update_us = current_time_us;
        
        // Update global encoder data
        encoder_data.angle_rad = pwm_state.angle_rad;
        encoder_data.angle_deg = pwm_state.angle_deg;
        encoder_data.velocity_rad_s = pwm_state.velocity_rad_s;
        encoder_data.velocity_deg_s = pwm_state.velocity_deg_s;
        encoder_data.timestamp_us = current_time_us;
        encoder_data.valid = true;
        encoder_data.error_count = pwm_state.error_count;
        
        return true;
    }
    
    return false;
}

// Get angle in radians
static float pwm_encoder_get_angle_rad(void) {
    return pwm_state.angle_rad;
}

// Get angle in degrees
static float pwm_encoder_get_angle_deg(void) {
    return pwm_state.angle_deg;
}

// Get velocity in rad/s
static float pwm_encoder_get_velocity_rad_s(void) {
    return pwm_state.velocity_rad_s;
}

// Get velocity in deg/s
static float pwm_encoder_get_velocity_deg_s(void) {
    return pwm_state.velocity_deg_s;
}

// Check if data is valid
static bool pwm_encoder_is_valid(void) {
    return pwm_state.valid;
}

// Get error count
static uint32_t pwm_encoder_get_error_count(void) {
    return pwm_state.error_count;
}

// Reset error count
static void pwm_encoder_reset_errors(void) {
    pwm_state.error_count = 0;
    pwm_state.timeout_count = 0;
}

// Get encoder type name
static const char* pwm_encoder_get_type_name(void) {
    return "PWM Magnetic";
}

// PWM encoder interface
const encoder_interface_t pwm_encoder_interface = {
    .init = pwm_encoder_init,
    .update = pwm_encoder_update,
    .get_angle_rad = pwm_encoder_get_angle_rad,
    .get_angle_deg = pwm_encoder_get_angle_deg,
    .get_velocity_rad_s = pwm_encoder_get_velocity_rad_s,
    .get_velocity_deg_s = pwm_encoder_get_velocity_deg_s,
    .is_valid = pwm_encoder_is_valid,
    .get_error_count = pwm_encoder_get_error_count,
    .reset_errors = pwm_encoder_reset_errors,
    .get_type_name = pwm_encoder_get_type_name
};

#endif /* ENCODER_TYPE == ENCODER_TYPE_PWM_MAGNETIC */
