/* Exoskeleton Arm Sensor Code
  Reads 3 AS5600 sensors at arm joints, add correction for pulley ratio and sensor offset,
    sends the data to the arm

  Using a TCA9548A I2C Multiplexer, because all the sensors have the same address

  SimpleFOC isn't necassary, but we are using it for motor control, so we can use the same
    code to read sensors on the exoskeleton and the arm
*/
#include <SimpleFOC.h>

// ESP-NOW allows two ESP32 chips to send data directly over wifi without a network
#include <esp_now.h>
#include <WiFi.h>

// Function for the Multiplexer to select I2C BUS
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  // Serial.print(bus);
}

// Declare the AS5600 sensors, 0 is shoulder flex, 1 is elbow, 2 is shoulder rotation
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// REPLACE WITH THE MAC Address of your receiver 84:fc:e6:7d:77:f8 (MotorGo Mini)
uint8_t broadcastAddress1[] = {0x84, 0xFC, 0xE6, 0x7D, 0x77, 0xF8};
uint8_t broadcastAddress2[] = {0x3C, 0x84, 0x27, 0x14, 0x0C, 0xC8};  // ESP32 OLED

// Variables to store the joint angles with corrections for the arm mechanics
float ch0_angle_pulley;
float ch1_angle_pulley;
float ch2_angle_pulley;

float ch0_old;
float ch1_old;
float ch2_old;

float ch0_offset;
float ch1_offset;
float ch2_offset;

int count = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    float ch0_rad;
    float ch1_rad;
    float ch2_rad;
    bool power;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Set I2C speed
  Wire.setClock(400000);
  // Wire1.setClock(400000); // Extra I2C channel, not used with multiplexer

  // Normally SimpleFOC will call begin for i2c but with esp32 begin() is the only way to set pins!
  // It seems safe to call begin multiple times
  Wire.begin(6, 7, (uint32_t)400000);
  // Wire1.begin(8, 9, (uint32_t)400000); // Extra I2C channel, not used with multiplexer

  // Switch multiplexer channels before initializing each sensor
  TCA9548A(0);
  sensor0.init();
  TCA9548A(1);
  sensor1.init();
  TCA9548A(2);
  sensor2.init();

  // Zero out sensors
  TCA9548A(0);
  sensor0.update();
  ch0_offset = sensor0.getAngle();
  TCA9548A(1);
  sensor1.update();
  ch1_offset = sensor1.getAngle();
  TCA9548A(2);
  sensor2.update();
  ch2_offset = sensor2.getAngle();
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  // Switch multiplexer channels before reading each sensor
  TCA9548A(0);
  sensor0.update();
  TCA9548A(1);
  sensor1.update();
  TCA9548A(2);
  sensor2.update();

  /* UPDATE THIS TO MATCH ARM MECHANICS!!
  Pulley ratio times sensor angle, negative if sensor direction is inverted,
    add offset for diff sensor positions
  */
  ch0_angle_pulley = 4.667 * (sensor0.getAngle() - ch0_offset);
  ch1_angle_pulley = -4.667 * (sensor1.getAngle() - ch1_offset);
  ch2_angle_pulley = -2.727 * (sensor2.getAngle() - ch2_offset);

  // Set values to send
  myData.ch0_rad = ch0_angle_pulley;
  myData.ch1_rad = ch1_angle_pulley;
  myData.ch2_rad = ch2_angle_pulley;
  // power will allow turning on and off the motors, not setup yet
  myData.power = true;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(0, (uint8_t *) &myData, sizeof(myData));

  if (ch0_angle_pulley != ch0_old or ch1_angle_pulley != ch1_old or ch2_angle_pulley != ch2_old) {
    if (count > 100) {
      // Print data sent to terminal for debugging
      Serial.println();
      Serial.println("Sensor Measurements");
      Serial.print(ch0_angle_pulley); 
      Serial.print(" : "); 
      Serial.print(ch1_angle_pulley);
      Serial.print(" : "); 
      Serial.print(ch2_angle_pulley);
      Serial.println();
      count = 0;
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
    }
  }

  ch0_old = ch0_angle_pulley;
  ch1_old = ch1_angle_pulley;
  ch2_old = ch2_angle_pulley;

  count++;

  // Delay to prevent overwhelming the rx
  delay(100);
}