#include <Arduino.h>
#include <SimpleFOC.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

#define PIN_RX 6
#define PIN_TX 7

// Define the PWM pin for the encoder
#define ENCODER_PWM_PIN 4

// Define the PPM pin for the VESC
#define VESC_PPM_PIN 19

Servo vescPPM; // Create a Servo object for the VESC PPM pin

// Create the PWM sensor object (1us = 0 deg, 1060us = 359 deg)
MagneticSensorPWM encoder = MagneticSensorPWM(ENCODER_PWM_PIN, 1, 1060);
void doPWM() {encoder.handlePWM();}

// Set up a new Serial object
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

//Use crsf.getChannel(x) to get us channel values (1-16).
void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}

// PID variables
double target_deg = 0;
double pos_deg = 0;
double pid_output = 0;

// PID tuning parameters
double Kp = 2.0, Ki = 0.0, Kd = 0.1;
PID myPID(&pos_deg, &pid_output, &target_deg, Kp, Ki, Kd, DIRECT);

#define min_deg 0
#define max_deg 359

void setup() {
  Serial.begin(115200);
  encoder.init();
  encoder.enableInterrupt(doPWM); // Enable the interrupt handler for the PWM signal
  Serial.println("Encoder initialized. Waiting for PWM signal...");
  _delay(1000); // Allow some time for the encoder to initialize

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
  crsf.begin(crsfSerial); // Pass the serial port by reference
  Serial.println("CRSF initialized. Waiting for receiver...");
  _delay(1000); // Allow some time for the encoder to initialize
  
  vescPPM.attach(VESC_PPM_PIN); // Attach the VESC PPM pin to the Servo object
  vescPPM.setPeriodHertz(50); // Set PWM frequency to 50Hz (20ms period)

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-500, 500); // Output range for PPM offset
}

void loop() {
  static unsigned long lastPrint = 0;
  encoder.update();
  float angle = encoder.getAngle(); // in radians
  pos_deg = angle * 180.0 / PI;
  crsf.update();

  // Map CH3 (1000-2000us) to claw range
  int ch3 = crsf.getChannel(3);
  target_deg = map(ch3, 1000, 2000, min_deg, max_deg);
  if (target_deg < min_deg) target_deg = min_deg;
  if (target_deg > max_deg) target_deg = max_deg;

  myPID.Compute();

  int ch5 = crsf.getChannel(5);

  int ppm_out;
  if (ch5 >= 1700) { // Channel 5 armed
    // Map PID output to PPM (1000-2000us, center at 1500)
    ppm_out = 1500 + (int)pid_output;
    if (ppm_out < 1000) ppm_out = 1000;
    if (ppm_out > 2000) ppm_out = 2000;
  } else {
    // Channel 5 disarmed, set to neutral position
    ppm_out = 1500;
  }
  vescPPM.writeMicroseconds(ppm_out);

  unsigned long now = millis();
  if (now - lastPrint >= 500) { // Print every half second
    Serial.print("Encoder angle: ");
    Serial.print(pos_deg);
    Serial.print(" deg, Target: ");
    Serial.print(target_deg);
    Serial.print(" deg, PPM: ");
    Serial.println(ppm_out);
    Serial.print("CH3: ");
    Serial.println(ch3);
    printChannels();
    lastPrint = now;
  }
  // No blocking delay here
}