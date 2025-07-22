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
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ENCODER";

// Global encoder data
encoder_data_t encoder_data = {0};

// Global encoder interface pointer
const encoder_interface_t* encoder_interface = NULL;

// Initialize encoder based on board configuration
bool encoder_init(void) {
    // Select appropriate encoder implementation based on board config
    #if ENCODER_TYPE == ENCODER_TYPE_PWM_MAGNETIC
        encoder_interface = &pwm_encoder_interface;
        ESP_LOGI(TAG, "Initializing PWM magnetic encoder");
    #elif ENCODER_TYPE == ENCODER_TYPE_QUADRATURE
        encoder_interface = &quad_encoder_interface;
        ESP_LOGI(TAG, "Initializing quadrature encoder");
    #elif ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID
        encoder_interface = &dual_encoder_interface;
        ESP_LOGI(TAG, "Initializing dual hybrid encoder (PWM + Quadrature)");
    #elif ENCODER_TYPE == ENCODER_TYPE_SPI_MAGNETIC
        encoder_interface = &spi_encoder_interface;
        ESP_LOGI(TAG, "Initializing SPI magnetic encoder");
    #elif ENCODER_TYPE == ENCODER_TYPE_I2C_MAGNETIC
        encoder_interface = &i2c_encoder_interface;
        ESP_LOGI(TAG, "Initializing I2C magnetic encoder");
    #elif ENCODER_TYPE == ENCODER_TYPE_HALL_SENSOR
        encoder_interface = &hall_encoder_interface;
        ESP_LOGI(TAG, "Initializing Hall sensor encoder");
    #elif ENCODER_TYPE == ENCODER_TYPE_NONE
        ESP_LOGW(TAG, "No encoder configured");
        return false;
    #else
        ESP_LOGE(TAG, "Unknown encoder type: %d", ENCODER_TYPE);
        return false;
    #endif

    if (encoder_interface && encoder_interface->init) {
        bool result = encoder_interface->init();
        if (result) {
            ESP_LOGI(TAG, "Encoder initialized successfully: %s", encoder_get_type_name());
        } else {
            ESP_LOGE(TAG, "Encoder initialization failed");
        }
        return result;
    }
    
    ESP_LOGE(TAG, "Encoder interface not available");
    return false;
}

// Update encoder data
bool encoder_update(void) {
    if (encoder_interface && encoder_interface->update) {
        return encoder_interface->update();
    }
    return false;
}

// Get angle in radians
float encoder_get_angle_rad(void) {
    if (encoder_interface && encoder_interface->get_angle_rad) {
        return encoder_interface->get_angle_rad();
    }
    return 0.0f;
}

// Get angle in degrees
float encoder_get_angle_deg(void) {
    if (encoder_interface && encoder_interface->get_angle_deg) {
        return encoder_interface->get_angle_deg();
    }
    return 0.0f;
}

// Get velocity in rad/s
float encoder_get_velocity_rad_s(void) {
    if (encoder_interface && encoder_interface->get_velocity_rad_s) {
        return encoder_interface->get_velocity_rad_s();
    }
    return 0.0f;
}

// Get velocity in deg/s
float encoder_get_velocity_deg_s(void) {
    if (encoder_interface && encoder_interface->get_velocity_deg_s) {
        return encoder_interface->get_velocity_deg_s();
    }
    return 0.0f;
}

// Check if encoder data is valid
bool encoder_is_valid(void) {
    if (encoder_interface && encoder_interface->is_valid) {
        return encoder_interface->is_valid();
    }
    return false;
}

// Get error count
uint32_t encoder_get_error_count(void) {
    if (encoder_interface && encoder_interface->get_error_count) {
        return encoder_interface->get_error_count();
    }
    return 0;
}

// Reset error count
void encoder_reset_errors(void) {
    if (encoder_interface && encoder_interface->reset_errors) {
        encoder_interface->reset_errors();
    }
}

// Get encoder type name
const char* encoder_get_type_name(void) {
    if (encoder_interface && encoder_interface->get_type_name) {
        return encoder_interface->get_type_name();
    }
    return "Unknown";
}

// Get encoder data structure
encoder_data_t* encoder_get_data(void) {
    return &encoder_data;
}

// Utility function: Normalize angle to 0-2π range
float encoder_normalize_angle_rad(float angle) {
    while (angle < 0.0f) angle += 2.0f * M_PI;
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    return angle;
}

// Utility function: Normalize angle to 0-360° range
float encoder_normalize_angle_deg(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Utility function: Convert radians to degrees
float encoder_rad_to_deg(float rad) {
    return rad * 180.0f / M_PI;
}

// Utility function: Convert degrees to radians
float encoder_deg_to_rad(float deg) {
    return deg * M_PI / 180.0f;
}
