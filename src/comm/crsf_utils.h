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

#ifndef CRSF_UTILS_H_
#define CRSF_UTILS_H_

#include "crsf_receiver.h"

// Common CRSF utility functions and macros

// Standard channel mapping (1-based)
#define CRSF_CHANNEL_ROLL       1
#define CRSF_CHANNEL_PITCH      2
#define CRSF_CHANNEL_THROTTLE   3
#define CRSF_CHANNEL_YAW        4
#define CRSF_CHANNEL_AUX1       5  // Often used for arm/disarm
#define CRSF_CHANNEL_AUX2       6
#define CRSF_CHANNEL_AUX3       7
#define CRSF_CHANNEL_AUX4       8

// Channel value constants
#define CRSF_CHANNEL_MIN_SCALED     1000
#define CRSF_CHANNEL_MID_SCALED     1500
#define CRSF_CHANNEL_MAX_SCALED     2000

// Switch thresholds for 3-position switches
#define CRSF_SWITCH_LOW_THRESHOLD   1300
#define CRSF_SWITCH_HIGH_THRESHOLD  1700

// Utility functions
static inline bool crsf_is_channel_low(uint8_t channel) {
    return crsf_get_channel_scaled(channel) < CRSF_SWITCH_LOW_THRESHOLD;
}

static inline bool crsf_is_channel_mid(uint8_t channel) {
    uint16_t value = crsf_get_channel_scaled(channel);
    return (value >= CRSF_SWITCH_LOW_THRESHOLD) && (value <= CRSF_SWITCH_HIGH_THRESHOLD);
}

static inline bool crsf_is_channel_high(uint8_t channel) {
    return crsf_get_channel_scaled(channel) > CRSF_SWITCH_HIGH_THRESHOLD;
}

// Convert channel value to percentage (-100% to +100%)
static inline float crsf_channel_to_percent(uint8_t channel) {
    uint16_t value = crsf_get_channel_scaled(channel);
    return (value - CRSF_CHANNEL_MID_SCALED) / 5.0f;
}

// Convert channel value to normalized range (-1.0 to +1.0)
static inline float crsf_channel_to_normalized(uint8_t channel) {
    uint16_t value = crsf_get_channel_scaled(channel);
    return (value - CRSF_CHANNEL_MID_SCALED) / 500.0f;
}

// Check if armed based on channel 5 (AUX1 - commonly used for arm/disarm)
static inline bool crsf_is_armed(void) {
    return crsf_is_connected() && !crsf_is_failsafe() && crsf_is_channel_high(CRSF_CHANNEL_AUX1);
}

#endif /* CRSF_UTILS_H_ */
