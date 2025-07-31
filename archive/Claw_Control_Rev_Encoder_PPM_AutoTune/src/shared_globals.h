#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <AlfredoCRSF.h>
#include <ESP32Servo.h>
#include <QuickPID.h>
#include <sTune.h>
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

// ------------------- Global Variables -------------------

// PID tuning parameters (use board config defaults)
#define min_deg MIN_ANGLE
#define max_deg MAX_ANGLE
extern float Kp, Ki, Kd;

// Board configuration
extern String board_name;

// Encoder angles and positions
extern float angleAB;        // Quadrature encoder angle (radians)
extern float anglePWM;       // PWM sensor angle (radians)
extern float pos_deg;        // Combined position (degrees) - changed to float for QuickPID
extern float pos_deg_PWM;    // PWM sensor position (degrees) - changed to float for QuickPID

extern float target_deg;     // Target position (degrees) - changed to float for QuickPID
extern float pid_output;     // PID output - changed to float for QuickPID

// AutoTune variables
extern bool autoTuneEnabled;
extern bool tuningComplete;
extern bool waitingForArm;
extern unsigned long tuneStartTime;

// Debug output control
extern bool debugEnabled;

// ------------------- Hardware Objects -------------------
extern Servo vescPPM;
extern MagneticSensorPWM encoderPWM;
extern Encoder encoder;
extern HardwareSerial crsfSerial;
extern AlfredoCRSF crsf;
extern const uint8_t peer_addr[6];
extern QuickPID myPID;
extern sTune tuner;
