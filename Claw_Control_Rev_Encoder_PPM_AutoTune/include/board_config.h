#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <Arduino.h>

// ================= BOARD CONFIGURATION SELECTOR =================
// Uncomment ONE of the following lines to select your board configuration
// #define BOARD_LEFT_CLAW
// #define BOARD_RIGHT_CLAW
// #define BOARD_LEFT_ELBOW
// #define BOARD_RIGHT_ELBOW
// #define BOARD_LEFT_UPPER
// #define BOARD_RIGHT_UPPER
#define BOARD_CUSTOM  // Use this for custom configurations

// ================= BOARD CONFIGURATIONS =================

#ifdef BOARD_LEFT_CLAW
  #define BOARD_NAME "Left_Claw"
  #define CONTROL_CHANNEL 4
  #define DEFAULT_KP 1.2
  #define DEFAULT_KI 0.05
  #define DEFAULT_KD 0.15
  #define MIN_ANGLE 0
  #define MAX_ANGLE 359
  // ESP-NOW peer MAC address
  #define PEER_MAC {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_RIGHT_CLAW
  #define BOARD_NAME "Right_Claw"
  #define CONTROL_CHANNEL 3
  #define DEFAULT_KP 1.2
  #define DEFAULT_KI 0.05
  #define DEFAULT_KD 0.15
  #define MIN_ANGLE 0
  #define MAX_ANGLE 359
  // ESP-NOW peer MAC address
  #define PEER_MAC {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_LEFT_ELBOW
  #define BOARD_NAME "Left_Elbow"
  #define CONTROL_CHANNEL 5
  #define DEFAULT_KP 1.0
  #define DEFAULT_KI 0.03
  #define DEFAULT_KD 0.12
  #define MIN_ANGLE 0
  #define MAX_ANGLE 180
  // ESP-NOW peer MAC address
  #define PEER_MAC {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_RIGHT_ELBOW
  #define BOARD_NAME "Right_Elbow"
  #define CONTROL_CHANNEL 6
  #define DEFAULT_KP 1.0
  #define DEFAULT_KI 0.03
  #define DEFAULT_KD 0.12
  #define MIN_ANGLE 0
  #define MAX_ANGLE 180
  // ESP-NOW peer MAC address
  #define PEER_MAC {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_LEFT_UPPER
  #define BOARD_NAME "Left_Upper"
  #define CONTROL_CHANNEL 7
  #define DEFAULT_KP 0.8
  #define DEFAULT_KI 0.02
  #define DEFAULT_KD 0.10
  #define MIN_ANGLE -90
  #define MAX_ANGLE 90
  // ESP-NOW peer MAC address
  #define PEER_MAC {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_RIGHT_UPPER
  #define BOARD_NAME "Right_Upper"
  #define CONTROL_CHANNEL 8
  #define DEFAULT_KP 0.8
  #define DEFAULT_KI 0.02
  #define DEFAULT_KD 0.10
  #define MIN_ANGLE -90
  #define MAX_ANGLE 90
  // ESP-NOW peer MAC address
  #define PEER_MAC {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

#ifdef BOARD_CUSTOM
  // Custom configuration - modify these values as needed
  #define BOARD_NAME "Left_Claw"
  #define CONTROL_CHANNEL 4
  #define DEFAULT_KP 0.5
  #define DEFAULT_KI 0.01
  #define DEFAULT_KD 0.05
  #define MIN_ANGLE 0
  #define MAX_ANGLE 359
  // ESP-NOW peer MAC address (update for your receiver)
  #define PEER_MAC {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
#endif

// Validate configuration
#ifndef BOARD_NAME
  #error "No board configuration selected! Please uncomment one of the BOARD_* defines in board_config.h"
#endif

#endif // BOARD_CONFIG_H
