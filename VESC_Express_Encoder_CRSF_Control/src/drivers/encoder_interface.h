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

#ifndef ENCODER_INTERFACE_H_
#define ENCODER_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "board_config.h"

// Constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Encoder data structure
typedef struct {
    float angle_rad;        // Current angle in radians
    float angle_deg;        // Current angle in degrees
    float velocity_rad_s;   // Angular velocity in rad/s
    float velocity_deg_s;   // Angular velocity in deg/s
    uint32_t timestamp_us;  // Last update timestamp in microseconds
    bool valid;             // Data validity flag
    uint32_t error_count;   // Error counter
} encoder_data_t;

// Encoder interface functions
typedef struct {
    bool (*init)(void);
    bool (*update)(void);
    float (*get_angle_rad)(void);
    float (*get_angle_deg)(void);
    float (*get_velocity_rad_s)(void);
    float (*get_velocity_deg_s)(void);
    bool (*is_valid)(void);
    uint32_t (*get_error_count)(void);
    void (*reset_errors)(void);
    const char* (*get_type_name)(void);
} encoder_interface_t;

// Global encoder interface
extern const encoder_interface_t* encoder_interface;
extern encoder_data_t encoder_data;

// Main encoder functions (these call the appropriate encoder implementation)
bool encoder_init(void);
bool encoder_update(void);
float encoder_get_angle_rad(void);
float encoder_get_angle_deg(void);
float encoder_get_velocity_rad_s(void);
float encoder_get_velocity_deg_s(void);
bool encoder_is_valid(void);
uint32_t encoder_get_error_count(void);
void encoder_reset_errors(void);
const char* encoder_get_type_name(void);
encoder_data_t* encoder_get_data(void);

// Utility functions
float encoder_normalize_angle_rad(float angle);
float encoder_normalize_angle_deg(float angle);
float encoder_rad_to_deg(float rad);
float encoder_deg_to_rad(float deg);

// Encoder type implementations
#if ENCODER_TYPE == ENCODER_TYPE_PWM_MAGNETIC || ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID
extern const encoder_interface_t pwm_encoder_interface;
#endif

#if ENCODER_TYPE == ENCODER_TYPE_QUADRATURE || ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID
extern const encoder_interface_t quad_encoder_interface;
#endif

#if ENCODER_TYPE == ENCODER_TYPE_DUAL_HYBRID
extern const encoder_interface_t dual_encoder_interface;
#endif

#if ENCODER_TYPE == ENCODER_TYPE_SPI_MAGNETIC
extern const encoder_interface_t spi_encoder_interface;
#endif

#if ENCODER_TYPE == ENCODER_TYPE_I2C_MAGNETIC
extern const encoder_interface_t i2c_encoder_interface;
#endif

#if ENCODER_TYPE == ENCODER_TYPE_HALL_SENSOR
extern const encoder_interface_t hall_encoder_interface;
#endif

#endif /* ENCODER_INTERFACE_H_ */
