#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#include <Wire.h>
#include <U8g2lib.h>

#define SSD1315_ADDR 0x78
#define I2C_SPEED 400E3
#define mcuSDA 8
#define mcuSCL 9

#define PIN_RX 16
#define PIN_TX 17

// Set up a new Serial object
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

void setup() {
  Wire.begin(mcuSDA, mcuSCL, I2C_SPEED);
  u8g2.setI2CAddress(SSD1315_ADDR);
  u8g2.begin();

  // Init Serial Monitor
  Serial.begin(115200);
  
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(crsfSerial);
}

void loop() {
  // Must call crsf.update() in loop() to process data
  crsf.update();
  //printChannels(); enable for debugging
  updateDisplay();
}

void updateDisplay(){
  // Display Readings on OLED Display
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_helvB12_tf);   // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("CH 1: ");
  u8g2.println(crsf.getChannel(1));
  u8g2.setCursor(0, 32);
  u8g2.print("CH 2: ");
  u8g2.println(crsf.getChannel(2));
  u8g2.setCursor(0, 48);
  u8g2.print("CH 3: ");
  u8g2.println(crsf.getChannel(3));
  u8g2.setCursor(0, 64);
  u8g2.print("CH 4: ");
  u8g2.println(crsf.getChannel(4));
  u8g2.sendBuffer();                    // transfer internal memory to the display
}

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