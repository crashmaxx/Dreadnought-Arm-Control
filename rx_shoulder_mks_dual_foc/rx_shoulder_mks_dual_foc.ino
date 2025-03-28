/*
MKS ESP32 FOC Closed Loop Position Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS ESP32 FOC V1.0
Enter "T+Position" in the serial monitor to make the two motors rotate in closed loop
For example, input the radian system "T3.14" to let the two motors rotate 180°
When using your own motor, do remember to modify the default number of pole pairs, the value in BLDCMotor()
The default power supply voltage set by the program is 12V
Please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply
The motor targeted by the default PID is the YT2804 motor. To use your own motor.
You need to modify the PID parameters to achieve better results.
*/
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <SimpleFOC.h>

#include <Preferences.h>
Preferences preferences;

bool calibrationMode = false;  // Set to true if you want to force calibration

#define CALIBRATION_SWITCH_PIN 13
bool lastPowerState_CALIBRATION_SWITCH = false;

// A5047P SPI pinout
#define HSPI_MISO 19
#define HSPI_MOSI 23
#define HSPI_SCLK 18
#define HSPI_SS 5

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin 
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5047_SPI, HSPI_SS);

// for esp 32, it has 2 spi interfaces VSPI (default) and HPSI as the second one
// to enable it instatiate the object
SPIClass SPI_2(HSPI);

//Motor Parameters
BLDCMotor motor2 = BLDCMotor(7);                               //According to pole pairs of the selected motor, modify the value of BLDCMotor() here
BLDCDriver3PWM driver2 = BLDCDriver3PWM(32,33,25,22);

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
bool pwr;

float ch0_offset;
float ch1_offset;
float ch2_offset;

float ch0_move = 0.0;
float ch1_move = 0.0;
float ch2_move = 0.0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  ch0_angle = myData.ch0_rad;
  ch1_angle = myData.ch1_rad;
  ch2_angle = myData.ch2_rad;
  pwr = myData.power;
}

//Command Settings
//Enter "T+Position" in the serial monitor to make the two motors rotate in closed loop
//For example, input the radian system "T3.14" to let the two motors rotate 180°
float target_position = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_position, cmd); }

void setup() {
  pinMode(CALIBRATION_SWITCH_PIN, INPUT);

  // Load saved offsets from Preferences
  preferences.begin("calibration", true); // true = read-only
  ch2_offset = preferences.getFloat("ch2_offset", 0.0);
  preferences.end();

  // OPTIONAL: Recalibrate if flag is set or command is received
  if (calibrationMode) {
    sensor2.update();
    ch2_offset = sensor2.getAngle();

    preferences.begin("calibration", false); // false = write mode
    preferences.putFloat("ch2_offset", ch2_offset);
    preferences.end();
  }

  // start the newly defined spi communication
  SPI_2.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS 
  // initialise magnetic sensor hardware
  sensor2.init(&SPI_2);
  motor2.linkSensor(&sensor2);

  //Supply Voltage Setting [V]
  driver2.voltage_power_supply = 24;                   //According to the supply voltage, modify the value of voltage_power_supply here
  driver2.init();

  //Connect the Motor and Driver Objects
  motor2.linkDriver(&driver2);
  
  //FOC Model Selection
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //Motion Control Mode Settings
  motor2.controller = MotionControlType::angle;

  //Speed PID Setting                                     
  motor2.PID_velocity.P = 0.5;             //According to the selected motor, modify the PID parameters here to achieve better results
  motor2.PID_velocity.I = 0.2;

  //Angle PID Setting 
  motor2.P_angle.P = 30;

  //Motor Maximum Limit Voltage
  motor2.voltage_limit = 16;                //According to the supply voltage, modify the value of voltage_limit here
  
  //Speed Low-pass Filter Time Constant
  motor2.LPF_velocity.Tf = 0.01;

  //Maximum Velocity Limit Setting
  motor2.velocity_limit = 1000;

  Serial.begin(115200);
  motor2.useMonitoring(Serial);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.print("ESP Board MAC Address: ");
  WiFi.STA.begin();
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  //Initialize the Motor
  motor2.init();

  //Initialize FOC
  motor2.initFOC();

  command.add('T', doTarget, "target position");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target position using serial terminal:")); 
}

void loop() {
  /**Serial.print(ch2_move); 
  Serial.print(" : "); 
  Serial.println(ch2_angle);**/
  
  motor2.loopFOC();

  if (pwr) {
    ch2_move = -((ch2_angle * 32) - ch2_offset);
  }
  else {
    ch2_move = target_position;
  }

  motor2.move(ch2_move);

  command.run();
  
  // Calibration via switch
  bool switchState2 = digitalRead(CALIBRATION_SWITCH_PIN);
  if (switchState2 == LOW && !lastPowerState_CALIBRATION_SWITCH) {
    Serial.println("Calibration button pressed. Saving new offsets...");

    sensor2.update(); ch2_offset = sensor2.getAngle();

    preferences.begin("calibration", false);
    preferences.putFloat("ch2_offset", ch2_offset);
    preferences.end();

    Serial.println("New offsets saved to flash.");
  }
  lastPowerState_CALIBRATION_SWITCH = (switchState2 == LOW);

}