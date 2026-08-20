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

#define CALIBRATION_SWITCH_PIN 15
bool lastPowerState_CALIBRATION_SWITCH = false;

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//Motor Parameters
BLDCMotor motor0 = BLDCMotor(7);                               //According to pole pairs of the selected motor, modify the value of BLDCMotor() here
BLDCDriver3PWM driver0 = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor1 = BLDCMotor(7);                              //Also modify the value of BLDCMotor() here
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26,27,14,12);

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
  ch0_offset = preferences.getFloat("ch0_offset", 0.0);
  ch1_offset = preferences.getFloat("ch1_offset", 0.0);
  //ch2_offset = preferences.getFloat("ch2_offset", 0.0);
  preferences.end();

  // OPTIONAL: Recalibrate if flag is set or command is received
  if (calibrationMode) {
    sensor0.update();
    ch0_offset = sensor0.getAngle();

    sensor1.update();
    ch1_offset = sensor1.getAngle();

    //sensor2.update();
    //ch2_offset = sensor2.getAngle();

  preferences.begin("calibration", false); // false = write mode
  preferences.putFloat("ch0_offset", ch0_offset);
  preferences.putFloat("ch1_offset", ch1_offset);
  //preferences.putFloat("ch2_offset", ch2_offset);
  preferences.end();
  }

  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  //Connect the Motor Object with the Sensor Object
  motor0.linkSensor(&sensor0);
  motor1.linkSensor(&sensor1);

  //Supply Voltage Setting [V]
  driver0.voltage_power_supply = 24;                   //According to the supply voltage, modify the value of voltage_power_supply here
  driver0.init();

  driver1.voltage_power_supply = 24;                  //Also modify the value of voltage_power_supply here
  driver1.init();
  //Connect the Motor and Driver Objects
  motor0.linkDriver(&driver0);
  motor1.linkDriver(&driver1);
  
  //FOC Model Selection
  motor0.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  motor0.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;

  //Speed PID Setting                                     
  motor0.PID_velocity.P = 0.6;             //According to the selected motor, modify the PID parameters here to achieve better results
  motor1.PID_velocity.P = 0.6;
  motor0.PID_velocity.I = 0.1;
  motor1.PID_velocity.I = 0.1;
  //Angle PID Setting 
  motor0.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //Motor Maximum Limit Voltage
  motor0.voltage_limit = 6;                //According to the supply voltage, modify the value of voltage_limit here
  motor1.voltage_limit = 6;               //Also modify the value of voltage_limit here
  
  //Speed Low-pass Filter Time Constant
  motor0.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //Maximum Velocity Limit Setting
  motor0.velocity_limit = 100;
  motor1.velocity_limit = 100;

  Serial.begin(115200);
  motor0.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

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
  motor0.init();
  motor1.init();
  //Initialize FOC
  motor0.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target position");

  Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target position using serial terminal:")); 
}

void loop() {
  /**Serial.print(ch0_move); 
  Serial.print(" : "); 
  Serial.print(ch0_angle); 
  Serial.print(" :: "); 
  Serial.print(ch1_move);
  Serial.print(" : "); 
  Serial.print(ch1_angle); 
  Serial.print(" :: "); 
  Serial.print(ch2_angle); 
  Serial.println();**/
  
  motor0.loopFOC();
  motor1.loopFOC();

  if (pwr) {
    ch0_move = -((ch0_angle * 4) - ch0_offset);
    ch1_move = -((ch1_angle * 4) - ch1_offset);
  }
  else {
    ch0_move = ch0_offset;
    ch1_move = ch1_offset;
  }

  motor0.move(ch0_move);
  motor1.move(ch1_move);

  //command.run();

  // Calibration via switch
  bool switchState2 = digitalRead(CALIBRATION_SWITCH_PIN);
  if (switchState2 == LOW && !lastPowerState_CALIBRATION_SWITCH) {
    Serial.println("Calibration button pressed. Saving new offsets...");

    sensor0.update(); ch0_offset = sensor0.getAngle();
    sensor1.update(); ch1_offset = sensor1.getAngle();

    preferences.begin("calibration", false);
    preferences.putFloat("ch0_offset", ch0_offset);
    preferences.putFloat("ch1_offset", ch1_offset);
    preferences.end();

    Serial.println("New offsets saved to flash.");
  }
  lastPowerState_CALIBRATION_SWITCH = (switchState2 == LOW);

}