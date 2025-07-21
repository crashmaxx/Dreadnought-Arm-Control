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

#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

// ================= BOARD CONFIGURATION SELECTOR =================
// Uncomment ONE of the following lines to select your board configuration

//#define BOARD_LEFT_CLAW
//#define BOARD_RIGHT_CLAW
//#define BOARD_LEFT_ELBOW
//#define BOARD_RIGHT_ELBOW
//#define BOARD_LEFT_UPPER
//#define BOARD_RIGHT_UPPER
//#define BOARD_LEFT_SHOULDER
//#define BOARD_RIGHT_SHOULDER
#define BOARD_VESC_EXPRESS_DEFAULT
//#define BOARD_VESC_EXPRESS_QUAD
//#define BOARD_VESC_EXPRESS_DUAL
//#define BOARD_CUSTOM

// ================= ENCODER TYPE CONFIGURATION =================
// Uncomment ONE encoder type per board

// Encoder type options
#define ENCODER_TYPE_NONE           0
#define ENCODER_TYPE_PWM_MAGNETIC   1
#define ENCODER_TYPE_QUADRATURE     2
#define ENCODER_TYPE_DUAL_HYBRID    3  // PWM + Quadrature
#define ENCODER_TYPE_SPI_MAGNETIC   4
#define ENCODER_TYPE_I2C_MAGNETIC   5
#define ENCODER_TYPE_HALL_SENSOR    6

// ================= BOARD CONFIGURATIONS =================

#ifdef BOARD_LEFT_CLAW
  #define BOARD_NAME "Left_Claw"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 4
  #define DEFAULT_KP 1.2f
  #define DEFAULT_KI 0.05f
  #define DEFAULT_KD 0.15f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio
  
  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_RIGHT_CLAW
  #define BOARD_NAME "Right_Claw"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 4
  #define DEFAULT_KP 1.2f
  #define DEFAULT_KI 0.05f
  #define DEFAULT_KD 0.15f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio
  
  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_LEFT_ELBOW
  #define BOARD_NAME "Left_Elbow"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 3
  #define DEFAULT_KP 1.0f
  #define DEFAULT_KI 0.02f
  #define DEFAULT_KD 0.10f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_RIGHT_ELBOW
  #define BOARD_NAME "Right_Elbow"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 3
  #define DEFAULT_KP 1.0f
  #define DEFAULT_KI 0.03f
  #define DEFAULT_KD 0.12f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio
  
  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_LEFT_UPPER
  #define BOARD_NAME "Left_Upper"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 2
  #define DEFAULT_KP 0.8f
  #define DEFAULT_KI 0.02f
  #define DEFAULT_KD 0.10f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio
  
  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_RIGHT_UPPER
  #define BOARD_NAME "Right_Upper"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 2
  #define DEFAULT_KP 0.8f
  #define DEFAULT_KI 0.02f
  #define DEFAULT_KD 0.10f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio
  
  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_LEFT_SHOULDER
  #define BOARD_NAME "Left_Shoulder"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters for shoulder joint
  #define CONTROL_CHANNEL 1
  #define DEFAULT_KP 0.6f
  #define DEFAULT_KI 0.01f
  #define DEFAULT_KD 0.08f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 180.0f
  #define GEAR_RATIO 89.6f  // Motor to joint encoder reduction ratio
  
  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_RIGHT_SHOULDER
  #define BOARD_NAME "Right_Shoulder"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Control parameters for shoulder joint
  #define CONTROL_CHANNEL 1
  #define DEFAULT_KP 0.6f
  #define DEFAULT_KI 0.01f
  #define DEFAULT_KD 0.08f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 180.0f
  #define GEAR_RATIO 89.6f  // Motor to joint encoder reduction ratio

  // ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_VESC_EXPRESS_DEFAULT
  #define BOARD_NAME "VESC_Express_Default"
  #define ENCODER_TYPE ENCODER_TYPE_PWM_MAGNETIC
  #define ENCODER_PWM_PIN 4
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Default control parameters
  #define CONTROL_CHANNEL 3
  #define DEFAULT_KP 1.0f
  #define DEFAULT_KI 0.0f
  #define DEFAULT_KD 0.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio
  
  // Default ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#endif

#ifdef BOARD_VESC_EXPRESS_QUAD
  #define BOARD_NAME "VESC_Express_Quadrature"
  #define ENCODER_TYPE ENCODER_TYPE_QUADRATURE
  #define ENCODER_A_PIN 4
  #define ENCODER_B_PIN 5
  #define ENCODER_PPR 1024
  
  // Default control parameters
  #define CONTROL_CHANNEL 3
  #define DEFAULT_KP 1.0f
  #define DEFAULT_KI 0.0f
  #define DEFAULT_KD 0.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio
  
  // Default ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#endif

#ifdef BOARD_VESC_EXPRESS_DUAL
  #define BOARD_NAME "VESC_Express_Dual"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Default control parameters
  #define CONTROL_CHANNEL 3
  #define DEFAULT_KP 1.0f
  #define DEFAULT_KI 0.0f
  #define DEFAULT_KD 0.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio
  
  // Default ESP-NOW peer MAC address
  #define PEER_MAC_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#endif

#ifdef BOARD_CUSTOM
  // Custom configuration - modify these values as needed
  #define BOARD_NAME "Custom_Board"
  #define ENCODER_TYPE ENCODER_TYPE_PWM_MAGNETIC
  #define ENCODER_PWM_PIN 4
  #define ENCODER_A_PIN 5
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 1024
  #define ENCODER_PWM_MIN_US 500
  #define ENCODER_PWM_MAX_US 2500
  
  // Custom control parameters
  #define CONTROL_CHANNEL 3
  #define DEFAULT_KP 1.0f
  #define DEFAULT_KI 0.0f
  #define DEFAULT_KD 0.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio
  
  // ESP-NOW peer MAC address (update for your receiver)
  #define PEER_MAC_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#endif

// ================= ESP-NOW TELEMETRY CONFIGURATION =================
#ifndef PEER_MAC_ADDR
  #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}  // Broadcast address
#endif

#define ESP_NOW_WIFI_CHANNEL 1
#define ESP_NOW_ENCRYPT false
#define ESP_NOW_TELEMETRY_ENABLE true

// ================= VALIDATION =================
#ifndef BOARD_NAME
  #error "No board configuration selected! Please uncomment one board type in board_config.h"
#endif

#ifndef ENCODER_TYPE
  #error "No encoder type defined for selected board!"
#endif

// ================= DEFAULT VALUES =================
// Default encoder configuration if not defined
#ifndef ENCODER_PPR
  #define ENCODER_PPR 1024
#endif

// PWM encoder defaults
#ifndef ENCODER_PWM_MIN_US
  #define ENCODER_PWM_MIN_US 500
#endif

#ifndef ENCODER_PWM_MAX_US
  #define ENCODER_PWM_MAX_US 2500
#endif

// Default control parameters
#ifndef DEFAULT_KP
  #define DEFAULT_KP 1.0f
#endif

#ifndef DEFAULT_KI
  #define DEFAULT_KI 0.0f
#endif

#ifndef DEFAULT_KD
  #define DEFAULT_KD 0.0f
#endif

#ifndef MIN_ANGLE
  #define MIN_ANGLE 0.0f
#endif

#ifndef MAX_ANGLE
  #define MAX_ANGLE 359.0f
#endif

#ifndef CONTROL_CHANNEL
  #define CONTROL_CHANNEL 3
#endif

#ifndef GEAR_RATIO
  #define GEAR_RATIO 1.0f
#endif

#endif /* BOARD_CONFIG_H_ */
