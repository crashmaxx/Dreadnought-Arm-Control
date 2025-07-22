#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

// ================= BOARD CONFIGURATION SELECTOR =================
// Uncomment ONE of the following lines to select your board configuration

#define BOARD_LEFT_CLAW
//#define BOARD_RIGHT_CLAW
//#define BOARD_LEFT_ELBOW
//#define BOARD_RIGHT_ELBOW
//#define BOARD_LEFT_UPPER
//#define BOARD_RIGHT_UPPER
//#define BOARD_LEFT_SHOULDER
//#define BOARD_RIGHT_SHOULDER
//#define BOARD_VESC_EXPRESS_DEFAULT
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
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 4
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_RIGHT_CLAW
  #define BOARD_NAME "Right_Claw"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 4
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_LEFT_ELBOW
  #define BOARD_NAME "Left_Elbow"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 3
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_RIGHT_ELBOW
  #define BOARD_NAME "Right_Elbow"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 3
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_LEFT_UPPER
  #define BOARD_NAME "Left_Upper"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 2
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_RIGHT_UPPER
  #define BOARD_NAME "Right_Upper"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Control parameters from Claw_Control
  #define CONTROL_CHANNEL 2
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 20.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_LEFT_SHOULDER
  #define BOARD_NAME "Left_Shoulder"
  #define ENCODER_TYPE ENCODER_TYPE_SPI_MAGNETIC
  
  // SPI pins for magnetic encoder
  #define ENCODER_SPI_CS_PIN 7
  #define ENCODER_SPI_MISO_PIN 8
  #define ENCODER_SPI_MOSI_PIN 9
  #define ENCODER_SPI_CLK_PIN 10
  
  // Control parameters for shoulder joint
  #define CONTROL_CHANNEL 1
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 180.0f
  #define GEAR_RATIO 89.6f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_RIGHT_SHOULDER
  #define BOARD_NAME "Right_Shoulder"
  #define ENCODER_TYPE ENCODER_TYPE_SPI_MAGNETIC
  
  // SPI pins for magnetic encoder
  #define ENCODER_SPI_CS_PIN 7
  #define ENCODER_SPI_MISO_PIN 8
  #define ENCODER_SPI_MOSI_PIN 9
  #define ENCODER_SPI_CLK_PIN 10
  
  // Control parameters for shoulder joint
  #define CONTROL_CHANNEL 1
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 180.0f
  #define GEAR_RATIO 89.6f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_VESC_EXPRESS_DEFAULT
  #define BOARD_NAME "VESC_Express_Default"
  #define ENCODER_TYPE ENCODER_TYPE_PWM_MAGNETIC
  #define ENCODER_PWM_PIN 5
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Default control parameters
  #define CONTROL_CHANNEL 3
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio
  
#endif

#ifdef BOARD_VESC_EXPRESS_QUAD
  #define BOARD_NAME "VESC_Express_Quadrature"
  #define ENCODER_TYPE ENCODER_TYPE_QUADRATURE
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  
  // Default control parameters
  #define CONTROL_CHANNEL 3
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_VESC_EXPRESS_DUAL
  #define BOARD_NAME "VESC_Express_Dual"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Default control parameters
  #define CONTROL_CHANNEL 3
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
  #define GEAR_RATIO 1.0f  // Motor to joint encoder reduction ratio

#endif

#ifdef BOARD_CUSTOM
  // Custom configuration - modify these values as needed
  #define BOARD_NAME "Custom_Board"
  #define ENCODER_TYPE ENCODER_TYPE_DUAL_HYBRID  // Switch to dual hybrid encoder
  #define ENCODER_PWM_PIN 5
  #define ENCODER_A_PIN 7
  #define ENCODER_B_PIN 6
  #define ENCODER_PPR 4096
  #define ENCODER_PWM_MIN_US 0     // 0v = 0°
  #define ENCODER_PWM_MAX_US 1020  // 359° at max
  
  // Custom control parameters
  #define CONTROL_CHANNEL 3
  #define MIN_ANGLE 0.0f
  #define MAX_ANGLE 359.0f
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

#endif /* BOARD_CONFIG_H_ */
