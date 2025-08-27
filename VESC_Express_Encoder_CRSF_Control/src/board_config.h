#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

// =============================================================================
// BOARD SELECTION - Uncomment ONE board type for your specific robot joint
// =============================================================================
//#define BOARD_LEFT_CLAW
//#define BOARD_RIGHT_CLAW
//#define BOARD_LEFT_ELBOW
//#define BOARD_RIGHT_ELBOW
//#define BOARD_LEFT_UPPER
//#define BOARD_RIGHT_UPPER
#define BOARD_LEFT_SHOULDER
//#define BOARD_RIGHT_SHOULDER
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
#define ENCODER_TYPE_VESC_INTERNAL  7  // Use VESC's internal position feedback

// ESPNOW Peer MAC address for telemetry
#define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}  // Replace with your peer's MAC address

// ================= BOARD CONFIGURATIONS =================

#ifdef BOARD_LEFT_CLAW
  #define BOARD_NAME "Left_Claw"
  #define ENCODER_TYPE ENCODER_TYPE_QUADRATURE
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PWM_PIN 5
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 4

  // CAN bus pins
  #define CAN_TX_GPIO_NUM 13       // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 12       // CAN RX pin for ESP32-S3

  // Control parameters
  #define CONTROL_CHANNEL 4
  #define REST_ANGLE 18.0f
  #define MIN_ANGLE 15.0f
  #define MAX_ANGLE 300.0f
  #define MAX_VEL 400.0f  // Maximum velocity for position commands
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 0

#endif

#ifdef BOARD_RIGHT_CLAW
  #define BOARD_NAME "Right_Claw"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PWM_PIN 5
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 4
  
  // CAN bus pins
  #define CAN_TX_GPIO_NUM 13       // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 12       // CAN RX pin for ESP32-S3

  // Control parameters
  #define CONTROL_CHANNEL 4
  #define REST_ANGLE 18.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define MAX_VEL 8400.0f  // Maximum velocity for position commands
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 1

#endif

#ifdef BOARD_LEFT_ELBOW
  #define BOARD_NAME "Left_Elbow"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PWM_PIN 5
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 3
  
  // CAN bus pins
  #define CAN_TX_GPIO_NUM 13      // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 12      // CAN RX pin for ESP32-S3

  // Control parameters
  #define CONTROL_CHANNEL 3
  #define REST_ANGLE 65.0f
  #define MIN_ANGLE 55.0f
  #define MAX_ANGLE 260.0f
  #define MAX_VEL 6300.0f  // Maximum velocity for position commands
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 1

#endif

#ifdef BOARD_RIGHT_ELBOW
  #define BOARD_NAME "Right_Elbow"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PWM_PIN 5
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 3
  
  // CAN bus pins
  #define CAN_TX_GPIO_NUM 13      // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 12      // CAN RX pin for ESP32-S3

  // Control parameters
  #define CONTROL_CHANNEL 3
  #define REST_ANGLE 65.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define MAX_VEL 6300.0f  // Maximum velocity for position commands
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 0

#endif

#ifdef BOARD_LEFT_UPPER
  #define BOARD_NAME "Left_Upper"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PWM_PIN 5
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 2

  // CAN bus pins
  #define CAN_TX_GPIO_NUM 13       // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 12       // CAN RX pin for ESP32-S3

  // Control parameters
  #define CONTROL_CHANNEL 2
  #define REST_ANGLE 65.0f
  #define MIN_ANGLE 60.0f
  #define MAX_ANGLE 293.0f
  #define MAX_VEL 6300.0f    // Maximum velocity for position commands
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 0

#endif

#ifdef BOARD_RIGHT_UPPER
  #define BOARD_NAME "Right_Upper"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PWM_PIN 5
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 2

    // CAN bus pins
  #define CAN_TX_GPIO_NUM 13       // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 12       // CAN RX pin for ESP32-S3

  // Control parameters
  #define CONTROL_CHANNEL 2
  #define REST_ANGLE 65.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define MAX_VEL 6300.0f    // Maximum velocity for position commands
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 1

#endif

#ifdef BOARD_LEFT_SHOULDER
  #define BOARD_NAME "Left_Shoulder"
  #define ENCODER_TYPE ENCODER_TYPE_SPI_MAGNETIC

  // SPI pins for magnetic encoder
  #define ENCODER_SPI_CS_PIN 10    // Chip Select
  #define ENCODER_SPI_MOSI_PIN 11  // Master Out Slave In
  #define ENCODER_SPI_CLK_PIN 12   // Clock
  #define ENCODER_SPI_MISO_PIN 13  // Master In Slave Out
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 1
  
  // CAN bus pins
  #define CAN_TX_GPIO_NUM 8       // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 9       // CAN RX pin for ESP32-S3
  
  // Control parameters
  #define CONTROL_CHANNEL 1
  #define REST_ANGLE 30.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 180.0f
  #define MAX_VEL 6300.0f    // Maximum velocity for position commands (slower for shoulder)
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 89.6f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 0

#endif

#ifdef BOARD_RIGHT_SHOULDER
  #define BOARD_NAME "Right_Shoulder"
  #define ENCODER_TYPE ENCODER_TYPE_SPI_MAGNETIC

  // SPI pins for magnetic encoder
  #define ENCODER_SPI_CS_PIN 10    // Chip Select
  #define ENCODER_SPI_MOSI_PIN 11  // Master Out Slave In
  #define ENCODER_SPI_CLK_PIN 12   // Clock
  #define ENCODER_SPI_MISO_PIN 13  // Master In Slave Out
  
  // CAN Configuration
  #define CAN_VESC_ID 0
  #define CAN_ESP32_ID 1

  // CAN bus pins
  #define CAN_TX_GPIO_NUM 8       // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 9       // CAN RX pin for ESP32-S3
  
  // Control parameters
  #define CONTROL_CHANNEL 1
  #define REST_ANGLE 30.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 180.0f
  #define MAX_VEL 6300.0f    // Maximum velocity for position commands (slower for shoulder)
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 89.6f  // Motor to joint encoder reduction ratio

  // ESP-NOW telemetry configuration
  #define ESP_NOW_TELEMETRY_ENABLE 1

#endif

#ifdef BOARD_CUSTOM
  // Custom configuration - modify these values as needed
  #define BOARD_NAME "Custom_Board"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 10      // ESP32-S3 compatible GPIO
  #define ENCODER_A_PIN 11        // ESP32-S3 compatible GPIO
  #define ENCODER_B_PIN 12        // ESP32-S3 compatible GPIO
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // CAN Configuration
  #define CAN_VESC_ID 115  // Custom VESC controller ID - modify as needed
  #define CAN_ESP32_ID 215  // ESP32 controller ID (VESC_ID + 100)
  
  // CAN bus pins
  #define CAN_TX_GPIO_NUM 4       // CAN TX pin for ESP32-S3
  #define CAN_RX_GPIO_NUM 5       // CAN RX pin for ESP32-S3

  // Custom control parameters
  #define CONTROL_CHANNEL 3
  #define REST_ANGLE 30.0f
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define MAX_VEL 1000.0f   // Maximum velocity for position commands
  #define MAX_ACCEL 10000.0f  // Maximum acceleration for position commands
  #define MAX_DECEL 10000.0f  // Maximum deceleration for position commands
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio
  
#endif

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
  #define ENCODER_PPR 4096
#endif

// PWM encoder defaults
#ifndef ENCODER_PWM_MIN_US
  #define ENCODER_PWM_MIN_US 0
#endif

#ifndef ENCODER_PWM_MAX_US
  #define ENCODER_PWM_MAX_US 1020
#endif

// Default control parameters
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

#ifndef MAX_VEL
  #define MAX_VEL 1000.0f
#endif

#ifndef MAX_ACCEL
  #define MAX_ACCEL 10000.0f
#endif

#ifndef MAX_DECEL
  #define MAX_DECEL 10000.0f
#endif

// CAN defaults
#ifndef CAN_VESC_ID
  #define CAN_VESC_ID 0
#endif

#ifndef CAN_ESP32_ID
  #define CAN_ESP32_ID 1
#endif


#endif /* BOARD_CONFIG_H_ */
