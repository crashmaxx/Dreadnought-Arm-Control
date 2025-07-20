#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <U8g2lib.h>

#define SSD1315_ADDR 0x78
#define I2C_SPEED 400E3
#define mcuSDA 8
#define mcuSCL 9

 // Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17

#define SUPERD_BAUD 9600

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial SuperDSerial(2);

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

float ch0_angle = 0.0;
float ch1_angle = 0.0;
float ch2_angle = 0.0;
bool pwr_bool = 0;
String power_str = "Off";

String numberString;
float number = 0.0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  ch0_angle = myData.ch0_rad;
  ch1_angle = myData.ch1_rad;
  ch2_angle = myData.ch2_rad;
  pwr_bool = myData.power;

  if (pwr_bool) {
    power_str = "On";
  }
  else {
    power_str = "Off";
  }
}
 
void setup() {
  Wire.begin(mcuSDA, mcuSCL, I2C_SPEED);
  u8g2.setI2CAddress(SSD1315_ADDR);
  u8g2.begin();

  // Init Serial Monitor
  Serial.begin(115200);
  
  SuperDSerial.begin(SUPERD_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");

 
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
}

void loop() {
  updateDisplay();
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

  // Output data to SuperDSerial connected to SuperD rx
  SuperDSerial.print("Ch0 Angle: ");
  SuperDSerial.println(ch0_angle);
  SuperDSerial.print("Ch1 Angle: ");
  SuperDSerial.println(ch1_angle);
  SuperDSerial.print("Ch2 Angle: ");
  SuperDSerial.println(ch2_angle);
  SuperDSerial.print("Power: ");
  SuperDSerial.println(power_str);
  SuperDSerial.println();
}

void extractFloats(String str) {
  for (int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (isDigit(c) || c == '.') {
      numberString += c;  // Collect digits and decimal points
    } else {
      if (numberString.length() > 0) {
        number = numberString.toFloat();  // Convert to float
        // Serial.println(number);  // Print the float
        numberString = "";  // Reset for next number
      }
    }
  }
}