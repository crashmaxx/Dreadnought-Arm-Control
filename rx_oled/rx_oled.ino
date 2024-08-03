#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <U8g2lib.h>

#define SSD1315_ADDR 0x78
#define I2C_SPEED 400E3
#define mcuSDA 8
#define mcuSCL 9

/*
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "comms/streams/PacketCommander.h"
#include "comms/streams/TextIO.h"
*/
// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0x3C, 0x84, 0x27, 0x29, 0x07, 0xC4};

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

float ch0_angle;
float ch1_angle;
float ch2_angle;
String power_str;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  ch0_angle = myData.ch0_rad;
  ch1_angle = myData.ch1_rad;
  ch2_angle = myData.ch2_rad;

  if (myData.power) {
    power_str = "On";
  }
  else {
    power_str = "Off";
  }
}

//TextIO comms = TextIO(Serial1);
//PacketCommander packetCommander = PacketCommander();
 
void setup() {
  Wire.begin(mcuSDA, mcuSCL, I2C_SPEED);
  u8g2.setI2CAddress(SSD1315_ADDR);
  u8g2.begin();

  // Init Serial Monitor
  Serial.begin(115200);
  // Configure RX and TX pins
  Serial1.begin(9600);

 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  //packetCommander.echo = true;
  //packetCommander.init(comms);
}

void loop() {
  updateDisplay();
  //packetCommander.run();
}

void updateDisplay(){
  // Display Readings on OLED Display
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_helvB12_tf);   // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("Shoulder: ");
  u8g2.println(ch0_angle);
  u8g2.setCursor(0, 32);
  u8g2.print("Elbow: ");
  u8g2.println(ch1_angle);
  u8g2.setCursor(0, 48);
  u8g2.print("Rotation: ");
  u8g2.println(ch2_angle);
  u8g2.setCursor(0, 64);
  u8g2.print("Power: ");
  u8g2.println(power_str);
  u8g2.sendBuffer();                    // transfer internal memory to the display

  // Output data to serial
  /*Serial.print("Ch0 Angle: ");
  Serial.println(ch0_angle);
  Serial.print("Ch1 Angle: ");
  Serial.println(ch1_angle);
  Serial.print("Ch2 Angle: ");
  Serial.println(ch2_angle);
  Serial.print("Power: ");
  Serial.println(power_str);
  Serial.println();*/

  Serial1.write('M');
  Serial1.println(ch2_angle);
}