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
#include "esp_log.h"
#include "esp_timer.h"
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

// Function implementations
static bool spi_encoder_update_impl(void);  // Forward declaration

static bool spi_encoder_init_impl(void) {
    ESP_LOGI(TAG, "Initializing SPI magnetic encoder (AS504x)");
    
    // Configure SPI pins
    spi_encoder_config.sw_spi.miso_pin = ENCODER_SPI_MISO_PIN;
    spi_encoder_config.sw_spi.mosi_pin = ENCODER_SPI_MOSI_PIN;
    spi_encoder_config.sw_spi.sck_pin = ENCODER_SPI_CLK_PIN;
    spi_encoder_config.sw_spi.nss_pin = ENCODER_SPI_CS_PIN;  // CS pin is called nss_pin
    
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
        
        // Do initial read to establish baseline
        spi_encoder_update_impl();
        
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to initialize AS504x SPI encoder");
        ESP_LOGE(TAG, "Check SPI wiring and encoder power supply");
        return false;
    }
}

static bool spi_encoder_update_impl(void) {
    if (!spi_encoder_initialized) {
        return false;
    }
    
    // Run the AS504x routine to update internal state
    enc_as504x_routine(&spi_encoder_config);
    
    // Check if encoder is connected
    if (!AS504x_IS_CONNECTED(&spi_encoder_config)) {
        error_count++;
        return false;
    }
    
    // Read angle from encoder
    float new_angle_deg = enc_as504x_read_angle(&spi_encoder_config);
    
    // Check for valid angle reading
    if (isnan(new_angle_deg) || isinf(new_angle_deg)) {
        error_count++;
        return false;
    }
    
    // Normalize angle to 0-360 degrees
    new_angle_deg = encoder_normalize_angle_deg(new_angle_deg);
    
    // Calculate velocity
    uint32_t current_time_us = esp_timer_get_time();
    uint32_t dt_us = current_time_us - last_update_time_us;
    
    if (dt_us > 0 && last_update_time_us > 0) {
        float dt_s = dt_us / 1000000.0f;
        float angle_diff = new_angle_deg - last_angle_deg;
        
        // Handle angle wraparound
        if (angle_diff > 180.0f) {
            angle_diff -= 360.0f;
        } else if (angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }
        
        // Simple velocity calculation (could be improved with filtering)
        velocity_deg_s = angle_diff / dt_s;
        
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
    .get_type_name = spi_encoder_get_type_name_impl
};

#endif // ENCODER_TYPE == ENCODER_TYPE_SPI_MAGNETIC
