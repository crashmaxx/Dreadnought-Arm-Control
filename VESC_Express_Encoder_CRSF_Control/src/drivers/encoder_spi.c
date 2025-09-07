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
#include "enc_as504x.h"
#include "board_config.h"
#include "debug_config.h"  // For DEBUG_VERBOSE macros
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#if ENCODER_TYPE == ENCODER_TYPE_SPI_MAGNETIC

static const char *TAG = "SPI_ENCODER";

// SPI encoder configuration
static AS504x_config_t spi_encoder_config;
static bool spi_encoder_initialized = false;
static float last_angle_deg = 0.0f;
static float velocity_deg_s = 0.0f;
static uint32_t last_update_time_us = 0;
static uint32_t error_count = 0;

// Multi-turn tracking variables
static float raw_angle_deg = 0.0f;      // Raw 0-360° reading from encoder
static float multi_turn_angle_deg = 0.0f;  // Accumulated multi-turn angle
static int32_t turn_count = 0;           // Number of complete rotations
static bool first_reading = true;       // Flag for first valid reading

// Function implementations
static bool spi_encoder_update_impl(void);  // Forward declaration

// Simple SPI connectivity test
static void spi_connectivity_test(void) {
    ESP_LOGI(TAG, "Running SPI connectivity test...");
    
    // Test basic SPI communication with AS504x
    spi_bb_begin(&(spi_encoder_config.sw_spi));
    
    // Send a dummy command and read response
    uint16_t test_response = 0;
    spi_bb_transfer_16(&(spi_encoder_config.sw_spi), &test_response, 0, 1);
    
    spi_bb_end(&(spi_encoder_config.sw_spi));
    
    ESP_LOGI(TAG, "SPI test response: 0x%04X", test_response);
    
    // Check if we got any response (not all 0s or all 1s)
    if (test_response == 0x0000 || test_response == 0xFFFF) {
        ESP_LOGW(TAG, "SPI response suggests connection issue");
        ESP_LOGW(TAG, "Check: 1) Wiring, 2) Power supply to AS504x, 3) Ground connections");
    } else {
        ESP_LOGI(TAG, "SPI appears to be responding");
    }
}

static bool spi_encoder_init_impl(void) {
    ESP_LOGI(TAG, "Initializing SPI magnetic encoder (AS504x)");
    
    // Configure SPI pins
    spi_encoder_config.sw_spi.miso_pin = ENCODER_SPI_MISO_PIN;
    spi_encoder_config.sw_spi.mosi_pin = ENCODER_SPI_MOSI_PIN;
    spi_encoder_config.sw_spi.sck_pin = ENCODER_SPI_CLK_PIN;
    spi_encoder_config.sw_spi.nss_pin = ENCODER_SPI_CS_PIN;  // CS pin is called nss_pin
    
    ESP_LOGI(TAG, "SPI pin configuration: CS=%d, CLK=%d, MISO=%d, MOSI=%d", 
             spi_encoder_config.sw_spi.nss_pin, spi_encoder_config.sw_spi.sck_pin,
             spi_encoder_config.sw_spi.miso_pin, spi_encoder_config.sw_spi.mosi_pin);
    
    // Initialize AS504x encoder
    bool init_result = enc_as504x_init(&spi_encoder_config);
    
    if (init_result) {
        ESP_LOGI(TAG, "AS504x SPI encoder initialized successfully");
        ESP_LOGI(TAG, "SPI pins configured - CS: %d, MISO: %d, MOSI: %d, CLK: %d", 
                 ENCODER_SPI_CS_PIN, ENCODER_SPI_MISO_PIN, 
                 ENCODER_SPI_MOSI_PIN, ENCODER_SPI_CLK_PIN);
        
        spi_encoder_initialized = true;
        last_update_time_us = esp_timer_get_time();
        error_count = 0;
        
        // Run SPI connectivity test first
        spi_connectivity_test();
        
        // Give the AS504x time to stabilize after SPI init
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Do initial read to establish baseline with multiple attempts
        ESP_LOGI(TAG, "Performing initial encoder read attempts...");
        bool initial_read = false;
        for (int attempt = 1; attempt <= 5; attempt++) {
            ESP_LOGI(TAG, "Read attempt %d/5...", attempt);
            initial_read = spi_encoder_update_impl();
            if (initial_read) {
                ESP_LOGI(TAG, "Success on attempt %d! Initial angle: %.2f degrees", attempt, last_angle_deg);
                break;
            } else {
                ESP_LOGW(TAG, "Attempt %d failed", attempt);
                vTaskDelay(pdMS_TO_TICKS(100));  // Wait between attempts
            }
        }
        
        if (!initial_read) {
            ESP_LOGE(TAG, "All initial read attempts failed - encoder may not be connected");
        }
        
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to initialize AS504x SPI encoder");
        ESP_LOGE(TAG, "Check SPI wiring and encoder power supply");
        return false;
    }
}

static bool spi_encoder_update_impl(void) {
    if (!spi_encoder_initialized) {
        ESP_LOGW(TAG, "SPI encoder not initialized");
        return false;
    }
    
    // Run the AS504x routine to update internal state
    enc_as504x_routine(&spi_encoder_config);
    
    // Get angle data directly - this is what actually matters
    float new_angle_deg = enc_as504x_read_angle(&spi_encoder_config);
    
    // Debug: Log the raw angle reading periodically and multi-turn info
    static uint32_t debug_counter = 0;
    debug_counter++;
    if (debug_counter % 20 == 1) {  // Log every 20th reading
        DEBUG_VERBOSE_LOG(TAG, "AS504x: raw=%.1f° turns=%ld multi=%.1f° SPI=0x%04X", 
                         raw_angle_deg, turn_count, multi_turn_angle_deg, spi_encoder_config.state.spi_val);
    }
    
    // Check if we got valid angle data
    // Note: 0.0° might be valid, but if we keep getting exactly 0.0°, it's probably an error
    static float last_non_zero_angle = -1.0f;
    static uint32_t zero_angle_count = 0;
    
    if (new_angle_deg == 0.0f) {
        zero_angle_count++;
        if (zero_angle_count > 10 && last_non_zero_angle >= 0.0f) {
            // Too many consecutive zero readings - probably an error
            DEBUG_VERBOSE_LOGW(TAG, "Too many zero angle readings (%lu), last good: %.2f°", zero_angle_count, last_non_zero_angle);
        }
    } else if (!isnan(new_angle_deg) && !isinf(new_angle_deg) && new_angle_deg > 0.0f && new_angle_deg <= 360.0f) {
        last_non_zero_angle = new_angle_deg;
        zero_angle_count = 0;
    }
    
    if (!isnan(new_angle_deg) && !isinf(new_angle_deg) && new_angle_deg >= 0.0f && new_angle_deg <= 360.0f) {
        // We have valid angle data - the encoder is working!
        
        // Store the raw single-turn angle (0-360°)
        raw_angle_deg = encoder_normalize_angle_deg(new_angle_deg);
        
        // Multi-turn tracking logic
        if (first_reading) {
            // Initialize multi-turn tracking on first valid reading
            multi_turn_angle_deg = raw_angle_deg;
            turn_count = 0;
            first_reading = false;
            DEBUG_VERBOSE_LOG(TAG, "Multi-turn tracking initialized at %.2f°", raw_angle_deg);
        } else {
            // Detect wraparound crossings
            float angle_diff = raw_angle_deg - (multi_turn_angle_deg - (turn_count * 360.0f));
            
            // Check for forward crossing (359° -> 0°)
            if (angle_diff < -270.0f) {
                turn_count++;
                DEBUG_VERBOSE_LOG(TAG, "Forward turn detected: %d turns, raw=%.2f°", turn_count, raw_angle_deg);
            }
            // Check for reverse crossing (0° -> 359°)
            else if (angle_diff > 270.0f) {
                turn_count--;
                DEBUG_VERBOSE_LOG(TAG, "Reverse turn detected: %d turns, raw=%.2f°", turn_count, raw_angle_deg);
            }
            
            // Calculate multi-turn angle
            multi_turn_angle_deg = raw_angle_deg + (turn_count * 360.0f);
        }
        
        // Use multi-turn angle for calculations
        new_angle_deg = multi_turn_angle_deg;
        
    } else {
        // Invalid angle data
        error_count++;
        if (error_count % 50 == 1) {
            DEBUG_VERBOSE_LOGW(TAG, "Invalid angle reading: %.2f (error count: %lu)", new_angle_deg, error_count);
        }
        return false;
    }
    
    // If we get here, we have valid angle data and a "connected" sensor
    
    // Calculate velocity using multi-turn angles
    uint32_t current_time_us = esp_timer_get_time();
    uint32_t dt_us = current_time_us - last_update_time_us;
    
    if (dt_us > 0 && last_update_time_us > 0) {
        float dt_s = dt_us / 1000000.0f;
        float angle_diff = new_angle_deg - last_angle_deg;
        
        // No need to handle wraparound with multi-turn angles
        // Simple velocity calculation with low-pass filtering
        float inst_velocity = angle_diff / dt_s;
        
        // Apply simple low-pass filter to velocity (alpha = 0.2)
        velocity_deg_s = 0.2f * inst_velocity + 0.8f * velocity_deg_s;
        
        // Limit velocity to reasonable range to filter noise
        if (fabsf(velocity_deg_s) > 36000.0f) {  // 100 rev/s limit
            velocity_deg_s = 0.0f;  // Likely noise, set to zero
        }
    }
    
    // Update stored values
    last_angle_deg = new_angle_deg;
    last_update_time_us = current_time_us;
    
    // Update global encoder data structure
    encoder_data.angle_deg = new_angle_deg;
    encoder_data.angle_rad = encoder_deg_to_rad(new_angle_deg);
    encoder_data.velocity_deg_s = velocity_deg_s;
    encoder_data.velocity_rad_s = encoder_deg_to_rad(velocity_deg_s);
    encoder_data.timestamp_us = current_time_us;
    encoder_data.valid = true;
    encoder_data.error_count = error_count;
    
    return true;
}

static float spi_encoder_get_angle_rad_impl(void) {
    return encoder_deg_to_rad(last_angle_deg);
}

static float spi_encoder_get_angle_deg_impl(void) {
    return last_angle_deg;
}

static float spi_encoder_get_velocity_rad_s_impl(void) {
    return encoder_deg_to_rad(velocity_deg_s);
}

static float spi_encoder_get_velocity_deg_s_impl(void) {
    return velocity_deg_s;
}

static bool spi_encoder_is_valid_impl(void) {
    if (!spi_encoder_initialized) {
        return false;
    }
    
    // Check connection status and recent update
    bool connected = AS504x_IS_CONNECTED(&spi_encoder_config);
    uint32_t current_time_us = esp_timer_get_time();
    bool recent_update = (current_time_us - last_update_time_us) < 100000; // 100ms timeout
    
    // Check for sensor diagnostic issues
    bool comp_ok = !AS504x_IS_COMP_HIGH(&spi_encoder_config) && !AS504x_IS_COMP_LOW(&spi_encoder_config);
    
    // Debug validation failures periodically
    static uint32_t debug_count = 0;
    debug_count++;
    if (debug_count % 100 == 1) {
        ESP_LOGW(TAG, "Validation check: connected=%d, recent_update=%d, comp_ok=%d, angle=%.2f°", 
                 connected, recent_update, comp_ok, last_angle_deg);
        ESP_LOGW(TAG, "Time since last update: %lu us, error_count=%lu", 
                 current_time_us - last_update_time_us, error_count);
    }
    
    return connected && recent_update && comp_ok;
}

static uint32_t spi_encoder_get_error_count_impl(void) {
    return error_count + spi_encoder_config.state.spi_communication_error_count;
}

static void spi_encoder_reset_errors_impl(void) {
    error_count = 0;
    spi_encoder_config.state.spi_communication_error_count = 0;
}

static const char* spi_encoder_get_type_name_impl(void) {
    return "SPI_MAGNETIC_AS504x";
}

// Set zero position for multi-turn tracking
static bool spi_encoder_set_zero_position_impl(float target_angle_deg) {
    if (!spi_encoder_initialized) {
        ESP_LOGW(TAG, "Cannot set zero position - encoder not initialized");
        return false;
    }
    
    // Update to get current raw angle
    if (!spi_encoder_update_impl()) {
        ESP_LOGW(TAG, "Cannot set zero position - encoder update failed");
        return false;
    }
    
    // Calculate new turn count to achieve target angle
    // target_angle = raw_angle + (turn_count * 360)
    // turn_count = (target_angle - raw_angle) / 360
    float turn_offset = (target_angle_deg - raw_angle_deg) / 360.0f;
    turn_count = (int32_t)roundf(turn_offset);
    
    // Recalculate multi-turn angle with new turn count
    multi_turn_angle_deg = raw_angle_deg + (turn_count * 360.0f);
    last_angle_deg = multi_turn_angle_deg;
    
    ESP_LOGI(TAG, "Zero position set: target=%.2f°, raw=%.2f°, turns=%ld, multi-turn=%.2f°", 
             target_angle_deg, raw_angle_deg, turn_count, multi_turn_angle_deg);
    
    return true;
}

// SPI encoder interface definition
const encoder_interface_t spi_encoder_interface = {
    .init = spi_encoder_init_impl,
    .update = spi_encoder_update_impl,
    .get_angle_rad = spi_encoder_get_angle_rad_impl,
    .get_angle_deg = spi_encoder_get_angle_deg_impl,
    .get_velocity_rad_s = spi_encoder_get_velocity_rad_s_impl,
    .get_velocity_deg_s = spi_encoder_get_velocity_deg_s_impl,
    .is_valid = spi_encoder_is_valid_impl,
    .get_error_count = spi_encoder_get_error_count_impl,
    .reset_errors = spi_encoder_reset_errors_impl,
    .set_zero_position = spi_encoder_set_zero_position_impl,
    .get_type_name = spi_encoder_get_type_name_impl
};

#endif // ENCODER_TYPE == ENCODER_TYPE_SPI_MAGNETIC
