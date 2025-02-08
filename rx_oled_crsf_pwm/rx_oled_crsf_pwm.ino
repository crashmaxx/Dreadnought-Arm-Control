#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#include <Wire.h>
#include <U8g2lib.h>
#include <SimpleFOC.h>
#include <PID_v1.h>

// PID Gain variables
// Fine tune as required
double Kp = 4.0;  // Proportional Gain
double Ki = 1.0;  // Integral Gain
double Kd = 0.05;  // Derivitive Gain
 
// PID Parameter variables
double Setpoint;
double Input;
double Output;
 
// Create PID Object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki , Kd, DIRECT);

float ch3deg = 0.0;
float ch3rad = 0.0;
float ch3joint = 0.0;
float ch3diff = 0.0;
float ch3speed = 1500.0;
bool ch5arm = false;

#include <ESP32Servo.h>
static const int servoPin = 13;

Servo vesc3;

#define SSD1315_ADDR 0x78
#define I2C_SPEED 400E3
#define I2C_SDA 9
#define I2C_SCL 8

#define PIN_RX 16
#define PIN_TX 17

// Set up a new Serial object
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, I2C_SDA, I2C_SCL, U8X8_PIN_NONE);

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);

void setup() {
  u8g2.setI2CAddress(SSD1315_ADDR);
  u8g2.begin();

  Wire1.begin(6, 7, I2C_SPEED);
  as5600.init(&Wire1);

  // Init Serial Monitor
  Serial.begin(115200);
  
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(crsfSerial);

  vesc3.attach(servoPin);

  // Initialize PID Controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-180, 180);
  myPID.SetSampleTime(20);

  _delay(1000);
}

void loop() {
  // Must call crsf.update() in loop() to process data
  crsf.update();
  //ch3deg = mapFloat(crsf.getChannel(3), 989, 2011, 0, 180);
  as5600.update();
  moveMotor();
  //printChannels(); enable for debugging
  updateDisplay();
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateDisplay(){
  // Display Readings on OLED Display
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_helvB12_tf);   // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("Speed: ");
  u8g2.println(ch3speed);
  u8g2.setCursor(0, 32);
  u8g2.print("CH 3 Deg: ");
  u8g2.println(ch3deg);
  u8g2.setCursor(0, 48);
  u8g2.print("En Deg: ");
  u8g2.println(ch3joint);
  u8g2.setCursor(0, 64);
  u8g2.print("CH 5: ");
  u8g2.println(ch5arm);
  u8g2.sendBuffer();                    // transfer internal memory to the display
}

void moveMotor()
{
  ch3deg = mapFloat(crsf.getChannel(3), 989, 2011, 0, 180);
  ch3rad = mapFloat(crsf.getChannel(3), 989, 2011, 0, 3.14);
  ch3joint = ((as5600.getAngle()) * 4068.0) / 71.0;
  Setpoint = ch3deg;
  Input = ch3joint;
  if (crsf.getChannel(5) > 1500)
  {
    ch5arm = true;
  }
  else
  {
    ch5arm = false;
  }

  if (ch5arm)
  {
    myPID.Compute();
    ch3speed = map(Output, -180, 180, 1400, 1600);
  }
  else
  {
    ch3speed = 1500;
  }
  vesc3.writeMicroseconds(ch3speed);
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