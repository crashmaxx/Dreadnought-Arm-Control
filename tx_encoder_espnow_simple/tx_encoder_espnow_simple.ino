/* Exoskeleton Arm Sensor Code
  Reads 1 AS5600 sensor at arm joint, sends the data to the arm

  SimpleFOC isn't necassary, but we are using it for motor control, so we can use the same
    code to read sensors on the exoskeleton and the arm
*/
#include <SimpleFOC.h>

// ESP-NOW allows two ESP32 chips to send data directly over wifi without a network
#include <esp_now.h>
#include <WiFi.h>

// Declare the AS5600 sensor
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress1[] = {0x84, 0xFC, 0xE6, 0x7D, 0x77, 0xF8};

// Variables to store the joint angles with corrections for the arm mechanics
float ch0_angle_pulley;

float ch0_offset;

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

  // Set I2C speed
  Wire.setClock(400000);
  // Wire1.setClock(400000); // Extra I2C channel, not used with multiplexer

  // Normally SimpleFOC will call begin for i2c but with esp32 begin() is the only way to set pins!
  // It seems safe to call begin multiple times
  Wire.begin(6, 7, (uint32_t)400000);
  // Wire1.begin(8, 9, (uint32_t)400000); // Extra I2C channel, not used with multiplexer

  // Zero out sensor
  sensor0.update();
  ch0_offset = sensor0.getAngle();
}

void loop() {
  // iterative function updating the sensor internal variables

  sensor0.update();

  ch0_angle_pulley = sensor0.getAngle() - ch0_offset;

  // Set values to send
  myData.ch0_rad = ch0_angle_pulley;
  myData.ch1_rad = 0;
  myData.ch2_rad = 0;
  // power will allow turning on and off the motors, not setup yet
  myData.power = true;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(0, (uint8_t *) &myData, sizeof(myData));

  // Print data sent to terminal for debugging
  Serial.println();
  Serial.println("Joint Angle in Rads");
  Serial.print(ch0_angle_pulley); 

  // Delay to prevent overwhelming the rx
  delay(100);
}