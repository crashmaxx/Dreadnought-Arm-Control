#include "motorgo_mini_pins.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "encoders/calibrated/CalibratedSensor.h"

// position / angle sensor instance from SimpleFOCDrivers
MagneticSensorMT6701SSI sensor_ch0(CH0_ENC_CS);
MagneticSensorMT6701SSI sensor_ch1(CH1_ENC_CS);

SPIClass hspi = SPIClass(HSPI);

// BLDC motor instance BLDCMotor(polepairs, (R), (KV))
BLDCMotor motor_ch0 = BLDCMotor(7, 2.3, 320);
BLDCMotor motor_ch1 = BLDCMotor(7, 2.3, 320);

// BLDC driver instance BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, (en))
BLDCDriver6PWM driver_ch0 = BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL, CH0_ENC_CS);
BLDCDriver6PWM driver_ch1 = BLDCDriver6PWM(CH1_UH, CH1_UL, CH1_VH, CH1_VL, CH1_WH, CH1_WL, CH1_ENC_CS);

// commander instance
Commander command = Commander(Serial);
void doTarget_ch0(char* cmd){command.motion(&motor_ch0, cmd);}
void doTarget_ch1(char* cmd){command.motion(&motor_ch1, cmd);}

void doLimit_ch0(char* cmd) { command.scalar(&motor_ch0.voltage_limit, cmd); }
void doLimit_ch1(char* cmd) { command.scalar(&motor_ch1.voltage_limit, cmd); }

void setup() {
    // start serial
    Serial.begin(115200);

    SimpleFOCDebug::enable(&Serial);

    // initialize sensor
    hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
    sensor_ch0.init(&hspi);
    sensor_ch1.init(&hspi);

    // set power supply voltage
    driver_ch0.voltage_power_supply = 12;
    driver_ch1.voltage_power_supply = 12;
    // set driver voltage limit, this phase voltage
    driver_ch0.voltage_limit = 3;
    driver_ch1.voltage_limit = 3;
    // initialize driver
    driver_ch0.init();
    driver_ch1.init();
    // link driver to motor
    motor_ch0.linkDriver(&driver_ch0);
    motor_ch1.linkDriver(&driver_ch1);

    // set motion control type to angle / position
    motor_ch0.controller = MotionControlType::velocity_openloop;
    motor_ch1.controller = MotionControlType::velocity_openloop;

    motor_ch0.voltage_limit = 3;
    motor_ch1.voltage_limit = 3;

    // set motor velocity limit
    motor_ch0.velocity_limit = 10000;
    motor_ch1.velocity_limit = 10000;

    // use monitoring
    motor_ch0.useMonitoring(Serial);
    motor_ch1.useMonitoring(Serial);

    // initialize motor
    motor_ch0.init();
    motor_ch1.init();

    // set the target velocity [rad/s]
    motor_ch0.target = 6.28; // one rotation per second
    motor_ch1.target = 6.28; // one rotation per second

    // add command to commander
    command.add('M', doTarget_ch0, "speed_0");
    command.add('N', doTarget_ch1, "speed_1");

        // add command to commander
    command.add('P', doLimit_ch0, "limit_0");
    command.add('Q', doLimit_ch1, "limit_1");

    // VerboseMode::nothing           - display nothing - good for monitoring
    // VerboseMode::on_request        - display only on user request
    // VerboseMode::user_friendly     - display textual messages to the user (default)
    // VerboseMode::machine_readable  - display machine readable messages 
    command.verbose = VerboseMode::user_friendly;

    _delay(1000);
}

void loop() {
    // this function can be run at much lower frequency than loopFOC()
    motor_ch0.move();
    motor_ch1.move();

    // significantly slowing the execution down
    //motor_ch0.monitor();
    //motor_ch1.monitor();

    // user communication
    command.run();
}
