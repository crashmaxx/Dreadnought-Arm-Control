/* Exoskeleton Arm Sensor Code
  Reads 3 AS5600 sensors at arm joints, add correction for pulley ratio and sensor offset,
    sends the data as CRSF channels to ELRS transmitter module

  Using a TCA9548A I2C Multiplexer, because all the sensors have the same address

  SimpleFOC isn't necassary, but we are using it for motor control, so we can use the same
    code to read sensors on the exoskeleton and the arm
*/
#include <SimpleFOC.h>

// CRSF protocol for communicating with ELRS transmitter module
#include "crsf.h"

// ESP-NOW for wireless communication to display
#include <esp_now.h>
#include <WiFi.h>

// Function for the Multiplexer to select I2C BUS
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// Declare the AS5600 sensors, numbered to match CRSF channels
// 1 = shoulder flex, 2 = elbow, 3 = shoulder rotation, 4 = claw
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor3 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor4 = MagneticSensorI2C(AS5600_I2C);

// CRSF Serial interface for ELRS TX module
// Using Serial2 with single pin 17 for TX/RX (true half-duplex)
CrsfSerial crsf(Serial2, 17, 17, true); // Single pin 17 for TX/RX half-duplex

// ELRS TX module expects 400kbaud by default, but can auto-detect other rates
#define CRSF_BAUDRATE 400000

// ESP-NOW configuration for wireless display
// MAC Address of your display receiver (update with your display's MAC address)
uint8_t displayAddress[] = {0xA8, 0x42, 0xE3, 0xE4, 0x06, 0xA4}; // Update this with your display's MAC

// Structure for ESP-NOW data to display
typedef struct struct_display_message {
    float ch1_rad;
    float ch2_rad;
    float ch3_rad;
    float ch4_rad;
    bool power;
    uint32_t timestamp;
} struct_display_message;

// Create data structure for display
struct_display_message displayData;

esp_now_peer_info_t peerInfo;

// Variables to store the joint angles with corrections for the arm mechanics
float ch1_angle_pulley;  // Channel 1: Shoulder flex
float ch2_angle_pulley;  // Channel 2: Elbow
float ch3_angle_pulley;  // Channel 3: Shoulder rotation
float ch4_angle_pulley;  // Channel 4: Claw

float ch1_offset;
float ch2_offset;
float ch3_offset;
float ch4_offset;

int count = 0;

bool pwr_bool = 0;

// set swith pin numbers
const int switchPin = 4;  // the number of the switch pin

// variable for storing the switch status 
int switchState = 0;

// Callback when ESP-NOW data is sent to display
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Uncomment for debugging ESP-NOW transmission
  // Serial.print("ESP-NOW to Display Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // initialize the switch pin as an input
  pinMode(switchPin, INPUT);

  // Initialize CRSF communication with ELRS TX module
  crsf.begin(CRSF_BAUDRATE);
  Serial.println("CRSF initialized for ELRS TX module");

  // Initialize ESP-NOW for display communication
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW send callback
  esp_now_register_send_cb(OnDataSent);

  // Register display peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, displayAddress, 6);
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add display peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized for display communication");

  // Set I2C speed
  Wire.setClock(400000);
  // Wire1.setClock(400000); // Extra I2C channel, not used with multiplexer

  // Normally SimpleFOC will call begin for i2c but with esp32 begin() is the only way to set pins!
  // It seems safe to call begin multiple times
  Wire.begin(6, 7, (uint32_t)400000);
  // Wire1.begin(8, 9, (uint32_t)400000); // Extra I2C channel, not used with multiplexer

  // Switch multiplexer channels before initializing each sensor
  TCA9548A(0);
  sensor1.init();
  TCA9548A(1);
  sensor2.init();
  TCA9548A(2);
  sensor3.init();
  TCA9548A(3);
  sensor4.init();

  // Zero out sensors
  TCA9548A(0);
  sensor1.update();
  ch1_offset = sensor1.getAngle();
  TCA9548A(1);
  sensor2.update();
  ch2_offset = sensor2.getAngle();
  TCA9548A(2);
  sensor3.update();
  ch3_offset = sensor3.getAngle();
  TCA9548A(3);
  sensor4.update();
  ch4_offset = sensor4.getAngle();
  
  Serial.println("Sensors initialized and zeroed");
  Serial.println("Starting CRSF channel transmission and ESP-NOW display updates...");
}

void loop() {
  // read the state of the switch value
  switchState = digitalRead(switchPin);

  // check if the switch is on
  // if it is, the switchState is HIGH
  if (switchState == LOW) {
    pwr_bool = 1;
  } else {
    pwr_bool = 0;
  }

  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  // Switch multiplexer channels before reading each sensor
  TCA9548A(0);
  sensor1.update();
  TCA9548A(1);
  sensor2.update();
  TCA9548A(2);
  sensor3.update();
  TCA9548A(3);
  sensor4.update();

  /* UPDATE THIS TO MATCH ARM MECHANICS!!
  Pulley ratio times sensor angle, negative if sensor direction is inverted,
    add offset for diff sensor positions
  */
  ch1_angle_pulley = sensor1.getAngle() - ch1_offset;  // Channel 1: Shoulder flex
  ch2_angle_pulley = sensor2.getAngle() - ch2_offset;  // Channel 2: Elbow
  ch3_angle_pulley = sensor3.getAngle() - ch3_offset;  // Channel 3: Shoulder rotation
  ch4_angle_pulley = sensor4.getAngle() - ch4_offset;  // Channel 4: Claw

  // Convert sensor angles to CRSF channel values
  // Map angles from -PI to +PI to channel range
  // Channel 1: Shoulder flex (sensor 1)
  float ch1_normalized = ch1_angle_pulley / (2 * PI); // -0.5 to +0.5
  ch1_normalized = constrain(ch1_normalized, -0.5, 0.5) * 2; // -1.0 to +1.0
  crsf.setChannelFloat(1, ch1_normalized);
  
  // Channel 2: Elbow (sensor 2)  
  float ch2_normalized = ch2_angle_pulley / (2 * PI); // -0.5 to +0.5
  ch2_normalized = constrain(ch2_normalized, -0.5, 0.5) * 2; // -1.0 to +1.0
  crsf.setChannelFloat(2, ch2_normalized);
  
  // Channel 3: Shoulder rotation (sensor 3)
  float ch3_normalized = ch3_angle_pulley / (2 * PI); // -0.5 to +0.5
  ch3_normalized = constrain(ch3_normalized, -0.5, 0.5) * 2; // -1.0 to +1.0
  crsf.setChannelFloat(3, ch3_normalized);
  
  // Channel 4: Claw (sensor 4)
  float ch4_normalized = ch4_angle_pulley / (2 * PI); // -0.5 to +0.5
  ch4_normalized = constrain(ch4_normalized, -0.5, 0.5) * 2; // -1.0 to +1.0
  crsf.setChannelFloat(4, ch4_normalized);
  
  // Channel 5: Power switch (explicit high/low)
  crsf.setChannelUs(5, pwr_bool ? 2000 : 1000);
  
  // Channels 6-16: Set to neutral/center position
  for (int i = 6; i <= 16; i++) {
    crsf.setChannelFloat(i, 0.0);
  }

  // Update CRSF transmission (this maintains the proper frame rate)
  crsf.update();

  // Prepare data for ESP-NOW display transmission
  displayData.ch1_rad = ch1_angle_pulley;
  displayData.ch2_rad = ch2_angle_pulley;
  displayData.ch3_rad = ch3_angle_pulley;
  displayData.ch4_rad = ch4_angle_pulley;
  displayData.power = pwr_bool;
  displayData.timestamp = millis();

  // Send data to display via ESP-NOW
  esp_err_t result = esp_now_send(displayAddress, (uint8_t *) &displayData, sizeof(displayData));

  // Print data sent to terminal for debugging
  Serial.println();
  Serial.println("Sensor Measurements (degrees)");
  Serial.print("Ch1 (Shoulder Flex): "); 
  Serial.print(ch1_angle_pulley * (180/PI), 2);
  Serial.print(" -> CRSF: ");
  Serial.print(crsf.getChannelUs(1));
  Serial.println("us");
  
  Serial.print("Ch2 (Elbow): "); 
  Serial.print(ch2_angle_pulley * (180/PI), 2);
  Serial.print(" -> CRSF: ");
  Serial.print(crsf.getChannelUs(2));
  Serial.println("us");
  
  Serial.print("Ch3 (Shoulder Rot): "); 
  Serial.print(ch3_angle_pulley * (180/PI), 2);
  Serial.print(" -> CRSF: ");
  Serial.print(crsf.getChannelUs(3));
  Serial.println("us");
  
  Serial.print("Ch4 (Claw): "); 
  Serial.print(ch4_angle_pulley * (180/PI), 2);
  Serial.print(" -> CRSF: ");
  Serial.print(crsf.getChannelUs(4));
  Serial.println("us");
  
  Serial.print("Ch5 (Power): ");
  Serial.print(pwr_bool ? "ON" : "OFF");
  Serial.print(" -> CRSF: ");
  Serial.print(crsf.getChannelUs(5));
  Serial.println("us");

  /*
  Serial.println();
  Serial.println("Sensor Measurements");
  Serial.print(sensor0.getVelocity()); 
  Serial.print(" : "); 
  Serial.print(sensor1.getVelocity());
  Serial.print(" : "); 
  Serial.print(sensor2.getVelocity());
  Serial.println();
  */

  Serial.println("CRSF channels sent to ELRS TX module");
  
  if (result == ESP_OK) {
    Serial.println("Display data sent via ESP-NOW successfully");
  } else {
    Serial.println("Error sending display data via ESP-NOW");
  }

  // Delay to prevent overwhelming the terminal output
  // CRSF update rate is handled internally
  delay(100);
}