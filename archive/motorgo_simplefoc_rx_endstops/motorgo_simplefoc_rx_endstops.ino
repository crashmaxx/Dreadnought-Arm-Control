#include "motorgo_mini_pins.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "encoders/calibrated/CalibratedSensor.h"

#include <esp_now.h>
#include <WiFi.h>

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

// position / angle sensor instance from SimpleFOCDrivers
MagneticSensorMT6701SSI sensor_ch0(CH0_ENC_CS);
MagneticSensorMT6701SSI sensor_ch1(CH1_ENC_CS);

SPIClass hspi = SPIClass(HSPI);

float ch0_angle;
float ch1_angle;
float ch2_angle;

int hall0 = 1;
int hall1 = 1;

// BLDC motor instance BLDCMotor(polepairs, (R), (KV))
BLDCMotor motor_ch0 = BLDCMotor(7, 2.3, 320);
BLDCMotor motor_ch1 = BLDCMotor(7, 2.3, 320);

// BLDC driver instance BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, (en))
BLDCDriver6PWM driver_ch0 = BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL, CH0_ENC_CS);
BLDCDriver6PWM driver_ch1 = BLDCDriver6PWM(CH1_UH, CH1_UL, CH1_VH, CH1_VL, CH1_WH, CH1_WL, CH1_ENC_CS);

// commander instance
Commander command = Commander(Serial);
void doMotor_ch0(char* cmd) { command.motor(&motor_ch0, cmd); }
void doMotor_ch1(char* cmd) { command.motor(&motor_ch1, cmd); }

void setup() {
    // start serial
    Serial.begin(115200);

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

    SimpleFOCDebug::enable(&Serial);

    pinMode(40, INPUT);
    pinMode(41, INPUT);

    // initialize sensor
    hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
    sensor_ch0.init(&hspi);
    sensor_ch1.init(&hspi);

    // link sensor to motor
    motor_ch0.linkSensor(&sensor_ch0);
    motor_ch1.linkSensor(&sensor_ch1);

    // set power supply voltage
    driver_ch0.voltage_power_supply = 12;
    driver_ch1.voltage_power_supply = 12;
    // set driver voltage limit, this phase voltage
    driver_ch0.voltage_limit = 5;
    driver_ch1.voltage_limit = 5;
    // initialize driver
    driver_ch0.init();
    driver_ch1.init();
    // link driver to motor
    motor_ch0.linkDriver(&driver_ch0);
    motor_ch1.linkDriver(&driver_ch1);

    // aligning voltage
    motor_ch0.voltage_sensor_align = 4;
    motor_ch1.voltage_sensor_align = 4;

    // set motion control type to angle / position
    motor_ch0.controller = MotionControlType::angle;
    motor_ch1.controller = MotionControlType::angle;

    // set torque control type to voltage (default)
    motor_ch0.torque_controller = TorqueControlType::voltage;
    motor_ch1.torque_controller = TorqueControlType::voltage;

    // set FOC modulation type to sinusoidal
    motor_ch0.foc_modulation = FOCModulationType::SinePWM;
    motor_ch1.foc_modulation = FOCModulationType::SinePWM;

    // velocity PID controller// velocity PID controller parameters
    // default P=0.5 I = 10 D =0    
    motor_ch0.PID_velocity.P = 0.1;
    motor_ch0.PID_velocity.I = 20.0;
    motor_ch0.PID_velocity.D = 0.0002;

    // ch1 velocity PID controller
    motor_ch1.PID_velocity.P = 0.1;
    motor_ch1.PID_velocity.I = 20.0;
    motor_ch1.PID_velocity.D = 0.0002;

    // jerk control using voltage voltage ramp
    // default value is 300 volts per sec  ~ 0.3V per millisecond
    motor_ch0.PID_velocity.output_ramp = 1000;
    motor_ch1.PID_velocity.output_ramp = 1000;

    // velocity low pass filtering
    // default 5ms - try different values to see what is the best. 
    // the lower the less filtered
    motor_ch0.LPF_velocity.Tf = 0.005;
    motor_ch1.LPF_velocity.Tf = 0.005;

    // setting the limits
    // either voltage
    motor_ch0.voltage_limit = 4.1; // Volts - default driver.voltage_limit
    motor_ch1.voltage_limit = 4.1; // Volts - default driver.voltage_limit
    // of current 
    motor_ch0.current_limit = 2; // Amps - default 0.2Amps
    motor_ch1.current_limit = 2; // Amps - default 0.2Amps

    // angle PID controller 
    // default P=20
    motor_ch0.P_angle.P = 12.0;
    motor_ch0.P_angle.I = 0.0;  // usually only P controller is enough 
    motor_ch0.P_angle.D = 0.0;  // usually only P controller is enough 
    // ch 1 angle PID
    motor_ch1.P_angle.P = 12.0;
    motor_ch1.P_angle.I = 0.0;  // usually only P controller is enough 
    motor_ch1.P_angle.D = 0.0;  // usually only P controller is enough 
    // acceleration control using output ramp
    // this variable is in rad/s^2 and sets the limit of acceleration
    motor_ch0.P_angle.output_ramp = 10000; // default 1e6 rad/s^2
    motor_ch1.P_angle.output_ramp = 10000; // default 1e6 rad/s^2

    // angle low pass filtering
    // default 0 - disabled  
    // use only for very noisy position sensors - try to avoid and keep the values very small
    motor_ch0.LPF_angle.Tf = 0; // default 0
    motor_ch1.LPF_angle.Tf = 0; // default 0

    // setting the limits
    //  maximal velocity of the position control
    motor_ch0.velocity_limit = 100;
    motor_ch1.velocity_limit = 100;

    // use monitoring
    motor_ch0.useMonitoring(Serial);
    motor_ch0.monitor_downsample = 0; // disable monitor at first - optional
    motor_ch1.useMonitoring(Serial);
    motor_ch1.monitor_downsample = 0; // disable monitor at first - optional

    // initialize motor
    motor_ch0.init();
    motor_ch1.init();

    // align sensor and start FOC
    motor_ch0.initFOC();
    motor_ch1.initFOC();

    // set the initial motor target
    motor_ch0.target = 0; // rads/s
    motor_ch1.target = 0; // rads/2

    // add command to commander
    command.add('M', doMotor_ch0, "motor_0");
    command.add('N', doMotor_ch1, "motor_1");
//    command.add('H',doHoming,"home motors");

    // VerboseMode::nothing           - display nothing - good for monitoring
    // VerboseMode::on_request        - display only on user request
    // VerboseMode::user_friendly     - display textual messages to the user (default)
    // VerboseMode::machine_readable  - display machine readable messages 
    command.verbose = VerboseMode::machine_readable;

    Serial.println(F("Motors ready."));
    Serial.println(F("Set the target using serial terminal and command M or N:"));

    _delay(1000);

    homing();

    _delay(1000);
}

void loop() {
    // Set angles received from puppet
    motor_ch0.target = ch0_angle;
    motor_ch1.target = ch1_angle;

    // main FOC algorithm function
    // the faster you run this function the better
    motor_ch0.loopFOC();
    motor_ch1.loopFOC();

    // this function can be run at much lower frequency than loopFOC()
    motor_ch0.move();
    motor_ch1.move();

    // significantly slowing the execution down
    motor_ch0.monitor();
    motor_ch1.monitor();

    // user communication
    command.run(Serial);
}

void homing() {
    // set motion control type to velocity
    motor_ch0.controller = MotionControlType::velocity;
    motor_ch1.controller = MotionControlType::velocity;

    // set the initial motor target
    motor_ch0.target = 1; // rads/s
    motor_ch1.target = 0; // rads/s

    while (hall0 == 1) {
      // main FOC algorithm function
      // the faster you run this function the better
      motor_ch0.loopFOC();
      motor_ch1.loopFOC();

      // this function can be run at much lower frequency than loopFOC()
      motor_ch0.move();
      motor_ch1.move();

      hall0 = digitalRead(40);
      if (hall0 == 0) {
        Serial.print("Hall Sensor 0: ");
        Serial.println(sensor_ch0.getAngle());
        motor_ch0.controller = MotionControlType::angle;
        motor_ch0.sensor_offset = motor_ch0.shaft_angle;
        motor_ch0.target = 0; // rads/s
      }
    }

    // set the new motor target
    motor_ch0.target = 0; // rad
    motor_ch1.target = 1; // rads/s

    while (hall1 == 1) {
      // main FOC algorithm function
      // the faster you run this function the better
      motor_ch0.loopFOC();
      motor_ch1.loopFOC();

      // this function can be run at much lower frequency than loopFOC()
      motor_ch0.move();
      motor_ch1.move();

      hall1 = digitalRead(41);
      if (hall1 == 0) {
        Serial.print("Hall Sensor 1: ");
        Serial.println(sensor_ch1.getAngle());
        motor_ch1.controller = MotionControlType::angle;
        motor_ch1.sensor_offset = motor_ch1.shaft_angle;
        motor_ch1.target = 0; // rads/
      }
    }

    // set the motor target
    motor_ch0.target = 0; // rad
    motor_ch1.target = 0; // rad
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  ch0_angle = myData.ch0_rad;
  ch1_angle = myData.ch1_rad;
  ch2_angle = myData.ch2_rad;

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Ch0 Angle: ");
  Serial.print(ch0_angle);
  Serial.print(" : "); 
  Serial.println(motor_ch0.shaft_angle);
  Serial.print("Ch1 Angle: ");
  Serial.print(ch1_angle);
  Serial.print(" : "); 
  Serial.println(motor_ch1.shaft_angle);
  Serial.print("Ch2 Angle: ");
  Serial.println(ch2_angle);
  Serial.print("Power: ");
  Serial.println(myData.power);
  Serial.println();
}
