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

#include "esp_log.h"
#include "esp_timer.h"
#include "board_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DUAL_ENCODER";

// Dual encoder state for combining PWM and quadrature readings
typedef struct {
    // Combined output
    float combined_angle_rad;
    float combined_angle_deg;
    float velocity_rad_s;
    float velocity_deg_s;
    uint32_t last_update_us;
    float last_angle_rad;
    bool valid;
    uint32_t total_error_count;
    
    // Initialization tracking
    bool offset_initialized;
    float initial_quad_offset_rad;
    bool pwm_zero_at_startup;
    float initial_quad_position_rad;
    bool movement_detected;
    uint32_t startup_time_us;
} dual_encoder_state_t;

static dual_encoder_state_t dual_state = {0};

#define VELOCITY_FILTER_ALPHA 0.1f

// Forward declare the external encoder interfaces
extern const encoder_interface_t pwm_encoder_interface;
extern const encoder_interface_t quad_encoder_interface;

// Initialize dual hybrid encoder by initializing both sub-encoders
static bool dual_encoder_init(void) {
    ESP_LOGI(TAG, "Initializing dual hybrid encoder (PWM + Quadrature)");
    
    // Initialize PWM encoder
    bool pwm_init_ok = pwm_encoder_interface.init();
    if (!pwm_init_ok) {
        ESP_LOGE(TAG, "PWM encoder initialization failed");
        return false;
    }
    
    // Initialize quadrature encoder  
    bool quad_init_ok = quad_encoder_interface.init();
    if (!quad_init_ok) {
        ESP_LOGE(TAG, "Quadrature encoder initialization failed");
        return false;
    }
    
    // Initialize dual encoder state
    dual_state.valid = false;
    dual_state.offset_initialized = false;
    dual_state.pwm_zero_at_startup = false;
    dual_state.movement_detected = false;
    dual_state.last_update_us = esp_timer_get_time();
    dual_state.startup_time_us = dual_state.last_update_us;
    dual_state.total_error_count = 0;
    
    // Check if PWM reads 0 at startup
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for PWM to stabilize
    pwm_encoder_interface.update();
    if (pwm_encoder_interface.is_valid()) {
        float pwm_angle_deg = pwm_encoder_interface.get_angle_deg();
        if (fabsf(pwm_angle_deg) < 1.0f) {  // Less than 1 degree = effectively zero
            dual_state.pwm_zero_at_startup = true;
            dual_state.initial_quad_position_rad = quad_encoder_interface.get_angle_rad();
            ESP_LOGI(TAG, "PWM reads 0° at startup - waiting for movement to initialize offset (will use MIN_ANGLE %.1f° as reference)", MIN_ANGLE);
        }
    }
    
    ESP_LOGI(TAG, "Dual hybrid encoder initialized successfully (PWM zero at startup: %s)", 
             dual_state.pwm_zero_at_startup ? "YES" : "NO");
    return true;
}

// Update dual encoder by combining PWM and quadrature readings
static bool dual_encoder_update(void) {
    uint32_t current_time_us = esp_timer_get_time();
    
    // Update both encoders
    bool pwm_updated = pwm_encoder_interface.update();
    bool quad_updated = quad_encoder_interface.update();
    
    if (!pwm_updated && !quad_updated) {
        return false; // Nothing to update
    }
    
    // Get current readings from both encoders
    bool pwm_valid = pwm_encoder_interface.is_valid();
    bool quad_valid = quad_encoder_interface.is_valid();
    
    // Check for movement if PWM was zero at startup
    if (dual_state.pwm_zero_at_startup && !dual_state.movement_detected && quad_valid) {
        float current_quad_position = quad_encoder_interface.get_angle_rad();
        float movement_threshold = 0.1f;  // 0.1 radians = ~5.7 degrees
        float movement_delta = current_quad_position - dual_state.initial_quad_position_rad;
        
        if (fabsf(movement_delta) > movement_threshold) {
            dual_state.movement_detected = true;
            ESP_LOGI(TAG, "Movement detected (%.2f rad = %.1f deg) - can now initialize with PWM", 
                     movement_delta, encoder_rad_to_deg(movement_delta));
        }
    }
    
    // Timeout for PWM zero detection - after 5 seconds, give up waiting for PWM
    if (dual_state.pwm_zero_at_startup && !dual_state.offset_initialized && 
        (current_time_us - dual_state.startup_time_us) > 5000000) {  // 5 seconds
        ESP_LOGW(TAG, "PWM zero timeout - initializing with quadrature at MIN_ANGLE (%.1f°)", MIN_ANGLE);
        dual_state.offset_initialized = true;
        // Set offset so that current quadrature position equals MIN_ANGLE
        float min_angle_rad = encoder_deg_to_rad(MIN_ANGLE);
        float current_quad_rad = quad_encoder_interface.get_angle_rad();
        dual_state.initial_quad_offset_rad = min_angle_rad - current_quad_rad;
    }
    
    // Initialize absolute position from PWM when conditions are met
    if (!dual_state.offset_initialized && pwm_valid && quad_valid) {
        bool can_initialize = false;
        
        if (dual_state.pwm_zero_at_startup) {
            // If PWM was zero at startup, only initialize after movement is detected
            // AND PWM is no longer reading near zero (handles both positive and negative readings)
            float pwm_angle_deg = pwm_encoder_interface.get_angle_deg();
            if (dual_state.movement_detected && fabsf(pwm_angle_deg) > 2.0f) {
                can_initialize = true;
                ESP_LOGI(TAG, "Movement detected and PWM now reads %.1f° - initializing offset", pwm_angle_deg);
            }
        } else {
            // If PWM didn't read zero at startup, initialize immediately
            can_initialize = true;
        }
        
        if (can_initialize) {
            float pwm_angle_rad = pwm_encoder_interface.get_angle_rad();
            float quad_angle_rad = quad_encoder_interface.get_angle_rad();
            
            // Calculate offset to align quadrature with PWM absolute position
            dual_state.initial_quad_offset_rad = pwm_angle_rad - quad_angle_rad;
            dual_state.offset_initialized = true;
            
            ESP_LOGI(TAG, "PWM initialization: %.3f rad (%.1f deg), quad offset: %.3f rad", 
                     pwm_angle_rad, encoder_rad_to_deg(pwm_angle_rad), 
                     dual_state.initial_quad_offset_rad);
        }
    }
    
    // Determine combined angle based on initialization state
    if (dual_state.offset_initialized && quad_valid) {
        // After initialization: use quadrature + offset for position tracking
        float quad_angle_rad = quad_encoder_interface.get_angle_rad();
        dual_state.combined_angle_rad = quad_angle_rad + dual_state.initial_quad_offset_rad;
        dual_state.velocity_rad_s = quad_encoder_interface.get_velocity_rad_s();
    } else if (pwm_valid && !dual_state.pwm_zero_at_startup) {
        // Before initialization and PWM wasn't zero at startup: use PWM
        dual_state.combined_angle_rad = pwm_encoder_interface.get_angle_rad();
        dual_state.velocity_rad_s = pwm_encoder_interface.get_velocity_rad_s();
    } else if (quad_valid) {
        // PWM was zero at startup and not yet initialized: use relative quadrature from MIN_ANGLE
        float quad_angle_rad = quad_encoder_interface.get_angle_rad();
        float movement_from_start = quad_angle_rad - dual_state.initial_quad_position_rad;
        float min_angle_rad = encoder_deg_to_rad(MIN_ANGLE);
        dual_state.combined_angle_rad = min_angle_rad + movement_from_start;
        dual_state.velocity_rad_s = quad_encoder_interface.get_velocity_rad_s();
        ESP_LOGD(TAG, "Using relative quadrature from MIN_ANGLE: %.2f° (waiting for PWM initialization)", 
                encoder_rad_to_deg(dual_state.combined_angle_rad));
    } else {
        // No valid readings
        dual_state.valid = false;
        return false;
    }
    
    // Update derived values
    dual_state.combined_angle_deg = encoder_rad_to_deg(dual_state.combined_angle_rad);
    dual_state.velocity_deg_s = encoder_rad_to_deg(dual_state.velocity_rad_s);
    dual_state.last_update_us = current_time_us;
    dual_state.valid = true;
    
    // Combine error counts
    dual_state.total_error_count = pwm_encoder_interface.get_error_count() + 
                                   quad_encoder_interface.get_error_count();
    
    // Update global encoder data
    encoder_data.angle_rad = dual_state.combined_angle_rad;
    encoder_data.angle_deg = dual_state.combined_angle_deg;
    encoder_data.velocity_rad_s = dual_state.velocity_rad_s;
    encoder_data.velocity_deg_s = dual_state.velocity_deg_s;
    encoder_data.timestamp_us = current_time_us;
    encoder_data.valid = dual_state.valid;
    encoder_data.error_count = dual_state.total_error_count;
    
    return true;
}

// Dual encoder interface functions
static float dual_encoder_get_angle_rad(void) { 
    return dual_state.combined_angle_rad; 
}

static float dual_encoder_get_angle_deg(void) { 
    return dual_state.combined_angle_deg; 
}

static float dual_encoder_get_velocity_rad_s(void) { 
    return dual_state.velocity_rad_s; 
}

static float dual_encoder_get_velocity_deg_s(void) { 
    return dual_state.velocity_deg_s; 
}

static bool dual_encoder_is_valid(void) { 
    return dual_state.valid; 
}

static uint32_t dual_encoder_get_error_count(void) { 
    return dual_state.total_error_count; 
}

static void dual_encoder_reset_errors(void) {
    pwm_encoder_interface.reset_errors();
    quad_encoder_interface.reset_errors();
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
