#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/aeat8800q24/MagneticSensorAEAT8800Q24.h>

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <esp_now_telemetry.h>

#define PIN_RX 1
#define PIN_TX 0

// Define the PPM pin for the VESC
#define VESC_PPM_PIN 19

Servo vescPPM; // Create a Servo object for the VESC PPM pin

MagneticSensorAEAT8800Q24 encoder(7);

// Set up a new Serial object
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

//Elecrow 7in HMI display 18:8B:0E:FF:02:E4
const uint8_t peer_addr[6] = {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4};

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
  // Initialize Serial for debugging
  Serial.begin(115200);

  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);  

  encoder.init(); // Initialize the encoder with the SPI interface
  Serial.println("Encoder initialized. Waiting for signal...");

  // Initialize CRSF serial communication
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
  crsf.begin(crsfSerial); // Pass the serial port by reference
  Serial.println("CRSF initialized. Waiting for receiver...");
  
  vescPPM.attach(VESC_PPM_PIN); // Attach the VESC PPM pin to the Servo object
  vescPPM.setPeriodHertz(50); // Set PWM frequency to 50Hz (20ms period)

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-500, 500); // Output range for PPM offset

  setTelemetryPeer(peer_addr); // Initialize ESP-NOW and add the telemetry peer
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
    Serial.print(angle);
    Serial.print(" rad, Target: ");
    Serial.print(target_deg);
    Serial.print(" deg, PPM: ");
    Serial.println(ppm_out);
    Serial.print("CH3: ");
    Serial.println(ch3);
    printChannels();
    lastPrint = now;
  }
  
  // Example: send telemetry every second
  static unsigned long lastSend = 0;
  if (now - lastSend > 1000) {
    sendTelemetry("Left_Claw", pos_deg, target_deg, pid_output);
    lastSend = now;
  }
  // No blocking delay here
}