#include "shared_globals.h"
#include "esp_now_telemetry.h"
#include "pid_autotune.h"
#include "encoder_handler.h"
#include "serial_commands.h"
#include "manual_tuning.h"

// ------------------- Global Variable Definitions -------------------

// PID tuning parameters (use board config defaults)
float Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;

// Board configuration
String board_name = BOARD_NAME;

// Encoder angles and positions
float angleAB = 0;        // Quadrature encoder angle (radians)
float anglePWM = 0;       // PWM sensor angle (radians)
float pos_deg = 0;        // Combined position (degrees)
float pos_deg_PWM = 0;    // PWM sensor position (degrees)

float target_deg = 0;     // Target position (degrees)
float pid_output = 0;     // PID output

// AutoTune variables
bool autoTuneEnabled = false;
bool tuningComplete = false;
bool waitingForArm = false;
unsigned long tuneStartTime = 0;

// Debug output control
bool debugEnabled = false;

// ------------------- Hardware Objects -------------------
Servo vescPPM; // VESC PPM output

// PWM sensor (1us = 0 deg, 1060us = 359 deg)
MagneticSensorPWM encoderPWM = MagneticSensorPWM(ENCODER_PWM_PIN, 1, 1060);

// Quadrature encoder
Encoder encoder = Encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_PPR, 4);

// CRSF receiver
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

// ESP-NOW peer MAC address (from board config)
const uint8_t peer_addr[6] = PEER_MAC;

// ------------------- PID Controller -------------------
QuickPID myPID(&pos_deg, &pid_output, &target_deg);

// sTune autotuner for QuickPID
sTune tuner = sTune(&pos_deg, &pid_output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  
  // Initialize encoders
  initializeEncoders();

  // Initialize CRSF receiver
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
  crsf.begin(crsfSerial);
  Serial.println("CRSF initialized. Waiting for receiver...");

  // Initialize VESC PPM output
  vescPPM.attach(VESC_PPM_PIN);
  vescPPM.setPeriodHertz(50);

  // Initialize PID with board config defaults
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(1);  // 1 = AUTOMATIC mode in QuickPID
  myPID.SetOutputLimits(-500, 500);

  // Initialize ESP-NOW telemetry
  setTelemetryPeer(peer_addr);
  
  Serial.println("Setup complete. Serial Commands:");
  Serial.println("  'autotune' - Start/Stop sTune AutoTune");
  Serial.println("  'manual' - Start manual step-response tuning");
  Serial.println("  'debug' - Toggle debug output on/off");
  Serial.println("  'status' - Show current configuration");
  Serial.printf("Board: %s, Channel: %d\n", board_name.c_str(), CONTROL_CHANNEL);
}

// ------------------- Main Loop -------------------
void loop() {
  // Handle serial commands
  handleSerialCommands();

  // Update encoders
  updateEncoders();

  // Update CRSF
  crsf.update();

  // Map channel (1000-2000us) to servo range (only when not autotuning or manual tuning)
  if (!autoTuneEnabled && !manualTuningEnabled) {
    int target_ppm = crsf.getChannel(CONTROL_CHANNEL);
    target_deg = (float)map(target_ppm, 1000, 2000, min_deg, max_deg);
    if (target_deg < min_deg) target_deg = min_deg;
    if (target_deg > max_deg) target_deg = max_deg;
  }

  // Handle AutoTune or normal PID operation
  handleAutoTune();

  // Always run PID to chase the target (whether set by RC or AutoTune)
  myPID.Compute();

  // Read arming channel
  int armswitch = crsf.getChannel(5);
  
  // Calculate PPM output for VESC
  int ppm_out;
  if (armswitch >= 1700) { // Channel 5 armed
    ppm_out = 1500 + (int)pid_output;
    if (ppm_out < 1000) ppm_out = 1000;
    if (ppm_out > 2000) ppm_out = 2000;
  } else {
    ppm_out = 1500; // Disarmed
  }
  vescPPM.writeMicroseconds(ppm_out);

  // Print debug info if enabled
  printDebugInfo(ppm_out);

  // Send telemetry every tenth of a second
  static unsigned long lastSend = 0;
  unsigned long now = millis();
  if (now - lastSend > 100) {
    sendTelemetry(board_name.c_str(), pos_deg, target_deg, ppm_out);
    lastSend = now;
  }
  // No blocking delay here
}