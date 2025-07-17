#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <AlfredoCRSF.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "board_config.h"

// ------------------- Pin Definitions -------------------
#define PIN_RX 1
#define PIN_TX 2
#define VESC_PPM_PIN 8
#define ENCODER_I_PIN 4
#define ENCODER_PWM_PIN 5
#define ENCODER_A_PIN 7
#define ENCODER_B_PIN 6
#define ENCODER_PPR 2048

// ------------------- EEPROM Configuration -------------------
#define EEPROM_SIZE 512
#define CONFIG_VERSION 2
#define CONFIG_START 0

struct Config {
  int version;
  double Kp;
  double Ki; 
  double Kd;
  char board_name[32];
  int channel_number;
  int min_angle;
  int max_angle;
  byte checksum;
};

// ------------------- Global Variables -------------------
extern Config config;

// PID tuning parameters (use board config defaults)
#define min_deg MIN_ANGLE
#define max_deg MAX_ANGLE
extern double Kp, Ki, Kd;

// Board configuration
extern String board_name;
extern int channel_number;

// Encoder angles and positions
extern float angleAB;        // Quadrature encoder angle (radians)
extern float anglePWM;       // PWM sensor angle (radians)
extern double pos_deg;       // Combined position (degrees)
extern double pos_deg_PWM;   // PWM sensor position (degrees)

extern double target_deg;    // Target position (degrees)
extern double pid_output;    // PID output

// AutoTune variables
extern bool autoTuneEnabled;
extern bool tuningComplete;
extern unsigned long tuneStartTime;
extern PID_ATune aTune;

// Debug output control
extern bool debugEnabled;

// ------------------- Hardware Objects -------------------
extern Servo vescPPM;
extern MagneticSensorPWM encoderPWM;
extern Encoder encoder;
extern HardwareSerial crsfSerial;
extern AlfredoCRSF crsf;
extern const uint8_t peer_addr[6];
extern PID myPID;
