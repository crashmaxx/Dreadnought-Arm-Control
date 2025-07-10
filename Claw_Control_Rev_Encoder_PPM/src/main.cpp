#include <Arduino.h>
#include <SimpleFOC.h>
#include <AlfredoCRSF.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <esp_now_telemetry.h>

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
// PID tuning parameters
#define min_deg 40
#define max_deg 400
double Kp = 2.0, Ki = 0.0, Kd = 0.1;

// Indentify the board and channel number
String board_name = "Left_Claw";
int channel_number = 4;

// Encoder angles and positions
float angleAB = 0;        // Quadrature encoder angle (radians)
float anglePWM = 0;       // PWM sensor angle (radians)
double pos_deg = 0;       // Combined position (degrees)
double pos_deg_PWM = 0;   // PWM sensor position (degrees)

double target_deg = 0;    // Target position (degrees)
double pid_output = 0;    // PID output

// ------------------- Hardware Objects -------------------
Servo vescPPM; // VESC PPM output

// PWM sensor (1us = 0 deg, 1060us = 359 deg)
MagneticSensorPWM encoderPWM = MagneticSensorPWM(ENCODER_PWM_PIN, 1, 1060);
void doPWM() { encoderPWM.handlePWM(); }

// Quadrature encoder
Encoder encoder = Encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_PPR, 4);
void doEncoderA() { encoder.handleA(); }
void doEncoderB() { encoder.handleB(); }

// CRSF receiver
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

// ESP-NOW peer MAC address (update for your receiver)
const uint8_t peer_addr[6] = {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4};

// ------------------- PID Controller -------------------
PID myPID(&pos_deg, &pid_output, &target_deg, Kp, Ki, Kd, DIRECT);

// ------------------- Utility Functions -------------------
// Print all CRSF channel values
void printChannels() {
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++) {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);

  // Initialize PWM sensor
  encoderPWM.init();
  encoderPWM.enableInterrupt(doPWM);

  // Initialize quadrature encoder
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doEncoderA, doEncoderB);

  // Initialize CRSF receiver
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
  crsf.begin(crsfSerial);
  Serial.println("CRSF initialized. Waiting for receiver...");

  // Initialize VESC PPM output
  vescPPM.attach(VESC_PPM_PIN);
  vescPPM.setPeriodHertz(50);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-500, 500);

  // Initialize ESP-NOW telemetry
  setTelemetryPeer(peer_addr);

  // Read initial PWM sensor angle
  encoderPWM.update();
  anglePWM = encoderPWM.getAngle();
  pos_deg_PWM = anglePWM * 180.0 / PI;
  Serial.print("PWM Sensor initialized. Angle: ");
  Serial.println(pos_deg_PWM);
}

// ------------------- Main Loop -------------------
void loop() {
  // Update encoders
  encoder.update();
  angleAB = encoder.getAngle();
  angleAB = angleAB + anglePWM; // Combine quadrature and PWM if needed
  pos_deg = angleAB * 180.0 / PI;

  // Update CRSF
  crsf.update();

  // Map channel (1000-2000us) to claw range
  int target_ppm = crsf.getChannel(channel_number);
  target_deg = map(target_ppm, 1000, 2000, min_deg, max_deg);
  if (target_deg < min_deg) target_deg = min_deg;
  if (target_deg > max_deg) target_deg = max_deg;

  // Compute PID
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

  // Print debug info every second
  // This is non-blocking and uses millis() to avoid delays
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    Serial.print("Encoder angle: ");
    Serial.print(pos_deg);
    Serial.print(" deg, PWM angle: ");
    Serial.print(pos_deg_PWM);
    Serial.print(" deg, Target angle: ");
    Serial.print(target_deg);
    Serial.print(" deg, PPM output: ");
    Serial.println(ppm_out);
    printChannels();
    lastPrint = now;
  }

  // Send telemetry every tenth of a second
  static unsigned long lastSend = 0;
  if (now - lastSend > 100) {
    sendTelemetry(board_name.c_str(), pos_deg, target_deg, ppm_out);
    lastSend = now;
  }
  // No blocking delay here
}