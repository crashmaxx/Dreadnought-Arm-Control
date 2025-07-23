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

#ifndef CRSF_CONFIG_H_
#define CRSF_CONFIG_H_

#include "driver/uart.h"

// =============================================================================
// CRSF Configuration Settings
// =============================================================================

// Hardware Configuration
#define CRSF_UART_NUM               UART_NUM_1      // UART port number
// Note: CRSF_TX_PIN and CRSF_RX_PIN are now defined in hardware-specific files (hw_*.h)
#define CRSF_BAUDRATE               420000          // Standard CRSF baud rate

// Protocol Configuration
#define CRSF_CONNECTION_TIMEOUT_MS  500             // Connection timeout in milliseconds
#define CRSF_TASK_PRIORITY          5               // FreeRTOS task priority
#define CRSF_TASK_STACK_SIZE        8192            // Task stack size in bytes (increased for debug logging)
#define CRSF_UART_BUFFER_SIZE       256             // UART buffer size
#define CRSF_UART_QUEUE_SIZE        10              // UART event queue size

// Control Configuration
#define CRSF_CONTROL_UPDATE_RATE_HZ 100             // Control loop update rate
#define CRSF_DEBUG_PRINT_RATE_MS    500             // Debug print interval
#define CRSF_ARM_CHANNEL            CRSF_CHANNEL_AUX1  // Channel used for arming
#define CRSF_ARM_THRESHOLD          1700            // PWM threshold for armed state

// Channel Assignment (customize based on your radio setup)
#define CRSF_THROTTLE_CHANNEL       CRSF_CHANNEL_THROTTLE
#define CRSF_STEERING_CHANNEL       CRSF_CHANNEL_ROLL
#define CRSF_ARM_CHANNEL_NUM        5               // Physical channel number for arming

// Failsafe Configuration
#define CRSF_FAILSAFE_CURRENT       0.0f            // Current to send on failsafe
#define CRSF_FAILSAFE_ENABLE_BRAKE  true            // Enable brake on failsafe
#define CRSF_FAILSAFE_BRAKE_CURRENT 5.0f            // Brake current on failsafe
#define CRSF_MAX_CURRENT_AMPS       10.0f           // Maximum current limit for safety

// CAN Configuration
// Note: VESC controller ID is now defined in board_config.h as CAN_VESC_ID
#define CRSF_CAN_ENABLE_STATUS      true            // Enable CAN status monitoring

// Feature Toggles
#define CRSF_ENABLE_LOGGING         true            // Enable ESP_LOG output
#define CRSF_ENABLE_TELEMETRY       false           // Enable telemetry feedback
#define CRSF_ENABLE_STATISTICS      true            // Enable connection statistics

// Advanced Configuration
#define CRSF_ENABLE_CHANNEL_SMOOTHING   false       // Enable channel value smoothing
#define CRSF_SMOOTHING_FACTOR           0.1f        // Smoothing factor (0.0-1.0)
#define CRSF_ENABLE_EXPO                false       // Enable exponential curve
#define CRSF_EXPO_FACTOR                0.3f        // Exponential factor

// Validation Macros
#if CRSF_CONTROL_UPDATE_RATE_HZ > 1000
    #error "CRSF_CONTROL_UPDATE_RATE_HZ too high"
#endif

#if CRSF_CONNECTION_TIMEOUT_MS < 100
    #warning "CRSF_CONNECTION_TIMEOUT_MS is very low"
#endif

#endif /* CRSF_CONFIG_H_ */
