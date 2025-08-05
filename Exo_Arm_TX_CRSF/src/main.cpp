#include <Arduino.h>
#include <SimpleFOC.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
// CRSF protocol for communicating with ELRS transmitter module
#include "crsf.h"

// For ESP32-S3, we need to create a HardwareSerial object
// ESP32-S3 has UART0 (Serial), UART1 available for general use
HardwareSerial crsfUart(1); // Use UART1

// CRSF Serial interface for ELRS TX module
// Alternative approach: Use separate TX/RX pins but wire them together externally
// This avoids ESP32-S3 pin management issues
// GPIO5 = TX, GPIO6 = RX (wire together externally for half-duplex)
CrsfSerial crsf(crsfUart, 6, 5, true); // RX=6, TX=5, half-duplex mode

// ELRS TX module expects 400kbaud by default, but can auto-detect other rates
#define CRSF_BAUDRATE 400000
#define CRSF_RC_CHANNEL_COUNT 16
#define CRSF_UPDATE_INTERVAL 14 // milliseconds, for ~70Hz update rate

// Select TCA9548A bus channel
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

Preferences preferences;
bool calibrationMode = false;

MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor3 = MagneticSensorI2C(AS5600_I2C);

uint8_t broadcastAddress1[] = {0xA8, 0x42, 0xE3, 0xE4, 0x06, 0xA4};   //  UPDATE TO CORRECT ESP32 RECEIVERS, IF STILL USING ESPNOW IN CODE
uint8_t broadcastAddress2[] = {0xC4, 0xDE, 0xE2, 0xF5, 0x72, 0xC0};   //  UPDATE TO CORRECT ESP32 RECEIVERS, IF STILL USING ESPNOW IN CODE
uint8_t broadcastAddress3[] = {0x64, 0xE8, 0x33, 0x73, 0xD3, 0xB4};   //  UPDATE TO CORRECT ESP32 RECEIVERS, IF STILL USING ESPNOW IN CODE

float ch1_angle, ch2_angle, ch3_angle, ch4_angle = 0.0;
float ch1_offset, ch2_offset, ch3_offset, ch4_offset = 0.0;

// Channel range calibration variables
float ch1_min, ch1_max;
float ch2_min, ch2_max;
float ch3_min, ch3_max;
float ch4_min, ch4_max;

#define ARMED_SWITCH_PIN 16  //  Toggle switch activated for TRUE FALSE logic.
#define CALIBRATION_SWITCH_PIN 18  //  Resets angle sensor readings to 0 in whatever position held when this button is held down.
#define RANGE_CALIBRATION_PIN 19  //  Button to calibrate channel ranges (min/max values)

bool lastPowerState_CALIBRATION_SWITCH = false;
bool lastPowerState_RANGE_CALIBRATION = false;
bool rangeCalibrationActive = false;

// Channel output variables
float ch1_normalized = 0.0;
float ch2_normalized = 0.0;
float ch3_normalized = 0.0;
float ch4_normalized = 0.0;

int count = 0;

typedef struct struct_message {
  float upperarm_rad;
  float lowerarm_rad;
  float shoulder_rad;
  bool power;   //  Links to value from ARMED_SWITCH_PIN 16  //  Toggle switch activated for TRUE FALSE logic.
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

float toDegrees(float radians) {
  return radians * (180.0 / PI);
}

// Function to map channel values to normalized range (-1.0 to 1.0)
float mapToNormalized(float value, float minVal, float maxVal) {
  // Ensure min/max are valid
  if (minVal >= maxVal) {
    return 0.0; // Return neutral if invalid range
  }
  
  // Clamp value to the range
  if (value < minVal) value = minVal;
  if (value > maxVal) value = maxVal;
  
  // Map to -1.0 to 1.0 range
  return ((value - minVal) / (maxVal - minVal)) * 2.0 - 1.0;
}

// Function to update all normalized channel values
void updateNormalizedChannels() {
  ch1_normalized = mapToNormalized(ch1_angle, ch1_min, ch1_max);  // shoulder -> ch1
  ch2_normalized = mapToNormalized(ch2_angle, ch2_min, ch2_max);  // upper arm -> ch2
  ch3_normalized = mapToNormalized(ch3_angle, ch3_min, ch3_max);  // elbow -> ch3
  ch4_normalized = mapToNormalized(ch4_angle, ch4_min, ch4_max);  // claw -> ch4
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Packet to: ");
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(2000); // Give time for serial monitor to connect

  Serial.println("=== CRSF TX Half-Duplex Test ===");
  Serial.print("ESP32-S3 Chip Model: ");
  Serial.println(ESP.getChipModel());
  Serial.println("Using GPIO5(TX) and GPIO6(RX) - wire together for half-duplex");

  // Initialize CRSF communication with ELRS TX module
  crsf.begin(CRSF_BAUDRATE);
  Serial.println("CRSF initialized for ELRS TX module");

  Serial.println("Starting CRSF channel transmission...");
  Serial.print("Update interval: ");
  Serial.print(CRSF_UPDATE_INTERVAL);
  Serial.println("ms");

  pinMode(ARMED_SWITCH_PIN, INPUT_PULLUP);
  pinMode(CALIBRATION_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RANGE_CALIBRATION_PIN, INPUT_PULLUP);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  esp_now_add_peer(&peerInfo);
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  esp_now_add_peer(&peerInfo);
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  esp_now_add_peer(&peerInfo);

  Wire.setClock(400000);
  Wire.begin(6, 7, (uint32_t)400000);

  TCA9548A(0); sensor1.init();
  TCA9548A(1); sensor2.init();
  TCA9548A(2); sensor3.init();

  preferences.begin("calibration", true);
  ch1_offset = preferences.getFloat("ch1_offset", 0.0);
  ch2_offset = preferences.getFloat("ch2_offset", 0.0);
  ch3_offset = preferences.getFloat("ch3_offset", 0.0);
  
  // Load channel range calibration values
  ch1_min = preferences.getFloat("ch1_min", -1.57); // Default to ~-90 degrees
  ch1_max = preferences.getFloat("ch1_max", 1.57);  // Default to ~90 degrees
  ch2_min = preferences.getFloat("ch2_min", -1.57);
  ch2_max = preferences.getFloat("ch2_max", 1.57);
  ch3_min = preferences.getFloat("ch3_min", -1.57);
  ch3_max = preferences.getFloat("ch3_max", 1.57);
  ch4_min = preferences.getFloat("ch4_min", -1.0);
  ch4_max = preferences.getFloat("ch4_max", 1.0);
  preferences.end();

  if (calibrationMode) {
    TCA9548A(0); sensor1.update(); ch1_offset = sensor1.getAngle();
    TCA9548A(1); sensor2.update(); ch2_offset = sensor2.getAngle();
    TCA9548A(2); sensor3.update(); ch3_offset = sensor3.getAngle();

    preferences.begin("calibration", false);
    preferences.putFloat("ch1_offset", ch1_offset);
    preferences.putFloat("ch2_offset", ch2_offset);
    preferences.putFloat("ch3_offset", ch3_offset);
    preferences.end();

    Serial.println("Calibration complete. Offsets saved.");
  }
}

void loop() {
  TCA9548A(0); sensor1.update();
  TCA9548A(1); sensor2.update();
  TCA9548A(2); sensor3.update();

  ch1_angle = (sensor1.getAngle() - ch1_offset) / 4;
  ch2_angle = sensor2.getAngle() - ch2_offset;
  ch3_angle = sensor3.getAngle() - ch3_offset;

  myData.upperarm_rad = -ch2_angle;
  myData.lowerarm_rad = ch3_angle;
  myData.shoulder_rad = ch1_angle;

  bool switchState1 = digitalRead(ARMED_SWITCH_PIN);
  myData.power = (switchState1 == LOW);

  Serial.print("ARMED SWITCH: ");
  Serial.println(switchState1 == LOW ? "ON (grounded)" : "OFF (high)");

  // Update normalized channel values
  updateNormalizedChannels();

  // Set channel values
  crsf.setChannelFloat(1, ch1_normalized);
  crsf.setChannelFloat(2, ch2_normalized);
  crsf.setChannelFloat(3, ch3_normalized);
  crsf.setChannelFloat(4, ch4_normalized);
  
  // Channel 5: Power switch (explicit high/low)
  crsf.setChannelUs(5, switchState1 ? 2000 : 1000);

  // Channels 6-16: Set to neutral/center position
  for (int i = 6; i <= 16; i++) {
    crsf.setChannelFloat(i, 0.0);
  }

  // Update CRSF transmission (this maintains the proper frame rate)
  crsf.update();

  esp_err_t result = esp_now_send(0, (uint8_t *)&myData, sizeof(myData));

  Serial.print(ch1_angle); Serial.print(" SHOULDER RAD / "); Serial.println(toDegrees(ch1_angle));
  Serial.print(ch2_angle); Serial.print(" UPPER ARM RAD / "); Serial.println(toDegrees(ch2_angle));
  Serial.print(ch3_angle); Serial.print(" LOWER ARM RAD / "); Serial.println(toDegrees(ch3_angle));
  Serial.print("Normalized channels: CH1="); Serial.print(ch1_normalized);
  Serial.print(" CH2="); Serial.print(ch2_normalized);
  Serial.print(" CH3="); Serial.print(ch3_normalized);
  Serial.print(" CH4="); Serial.println(ch4_normalized);
  Serial.println(result == ESP_OK ? "---   Sent with success" : "---   Error sending the data");

  // Serial-based calibration
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'c') {
      TCA9548A(0); sensor1.update(); ch1_offset = sensor1.getAngle();
      TCA9548A(1); sensor2.update(); ch2_offset = sensor2.getAngle();
      TCA9548A(2); sensor3.update(); ch3_offset = sensor3.getAngle();

      preferences.begin("calibration", false);
      preferences.putFloat("ch1_offset", ch1_offset);
      preferences.putFloat("ch2_offset", ch2_offset);
      preferences.putFloat("ch3_offset", ch3_offset);
      preferences.end();

      Serial.println("New offsets saved to flash.");
    }
    else if (input == 'r') {
      rangeCalibrationActive = !rangeCalibrationActive;
      if (rangeCalibrationActive) {
        Serial.println("Range calibration mode STARTED. Move through full range of motion, then press 'r' again to save.");
        // Initialize with current values
        ch1_min = ch1_max = ch1_angle;
        ch2_min = ch2_max = ch2_angle;
        ch3_min = ch3_max = ch3_angle;
      } else {
        // Save the calibrated ranges
        preferences.begin("calibration", false);
        preferences.putFloat("ch1_min", ch1_min);
        preferences.putFloat("ch1_max", ch1_max);
        preferences.putFloat("ch2_min", ch2_min);
        preferences.putFloat("ch2_max", ch2_max);
        preferences.putFloat("ch3_min", ch3_min);
        preferences.putFloat("ch3_max", ch3_max);
        preferences.end();
        
        Serial.println("Range calibration COMPLETE. Min/Max values saved to flash.");
        Serial.print("CH1 range: "); Serial.print(ch1_min); Serial.print(" to "); Serial.println(ch1_max);
        Serial.print("CH2 range: "); Serial.print(ch2_min); Serial.print(" to "); Serial.println(ch2_max);
        Serial.print("CH3 range: "); Serial.print(ch3_min); Serial.print(" to "); Serial.println(ch3_max);
      }
    }
  }

  // Update range calibration if active
  if (rangeCalibrationActive) {
    // Track min/max values during calibration
    if (ch1_angle < ch1_min) ch1_min = ch1_angle;
    if (ch1_angle > ch1_max) ch1_max = ch1_angle;
    if (ch2_angle < ch2_min) ch2_min = ch2_angle;
    if (ch2_angle > ch2_max) ch2_max = ch2_angle;
    if (ch3_angle < ch3_min) ch3_min = ch3_angle;
    if (ch3_angle > ch3_max) ch3_max = ch3_angle;
    
    Serial.print("CALIBRATING - CH1: ["); Serial.print(ch1_min); Serial.print(","); Serial.print(ch1_max);
    Serial.print("] CH2: ["); Serial.print(ch2_min); Serial.print(","); Serial.print(ch2_max);
    Serial.print("] CH3: ["); Serial.print(ch3_min); Serial.print(","); Serial.print(ch3_max); Serial.println("]");
  }

  // Calibration via switch
  bool switchState2 = digitalRead(CALIBRATION_SWITCH_PIN);
  if (switchState2 == LOW && !lastPowerState_CALIBRATION_SWITCH) {
    Serial.println("Calibration button pressed. Saving new offsets...");

    TCA9548A(0); sensor1.update(); ch1_offset = sensor1.getAngle();
    TCA9548A(1); sensor2.update(); ch2_offset = sensor2.getAngle();
    TCA9548A(2); sensor3.update(); ch3_offset = sensor3.getAngle();

    preferences.begin("calibration", false);
    preferences.putFloat("ch1_offset", ch1_offset);
    preferences.putFloat("ch2_offset", ch2_offset);
    preferences.putFloat("ch3_offset", ch3_offset);
    preferences.end();

    Serial.println("New offsets saved to flash.");
  }
  lastPowerState_CALIBRATION_SWITCH = (switchState2 == LOW);

  // Range calibration via hardware button
  bool switchState3 = digitalRead(RANGE_CALIBRATION_PIN);
  if (switchState3 == LOW && !lastPowerState_RANGE_CALIBRATION) {
    rangeCalibrationActive = !rangeCalibrationActive;
    if (rangeCalibrationActive) {
      Serial.println("Range calibration button pressed. Move through full range of motion, then press button again to save.");
      // Initialize with current values
      ch1_min = ch1_max = ch1_angle;
      ch2_min = ch2_max = ch2_angle;
      ch3_min = ch3_max = ch3_angle;
    } else {
      // Save the calibrated ranges
      preferences.begin("calibration", false);
      preferences.putFloat("ch1_min", ch1_min);
      preferences.putFloat("ch1_max", ch1_max);
      preferences.putFloat("ch2_min", ch2_min);
      preferences.putFloat("ch2_max", ch2_max);
      preferences.putFloat("ch3_min", ch3_min);
      preferences.putFloat("ch3_max", ch3_max);
      preferences.end();
      
      Serial.println("Range calibration COMPLETE via button. Min/Max values saved to flash.");
    }
  }
  lastPowerState_RANGE_CALIBRATION = (switchState3 == LOW);

  delay(50);
}
