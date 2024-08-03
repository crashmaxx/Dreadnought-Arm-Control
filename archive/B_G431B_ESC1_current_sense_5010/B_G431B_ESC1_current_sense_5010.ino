/** 
 * B-G431B-ESC1 position motion control example with encoder
 *
 */
#include <SimpleFOC.h>

// Motor instance
BLDCMotor motor = BLDCMotor(7, 0.25, 360);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// encoder instance
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
    
  // use monitoring with serial 
  Serial.begin(115200);

  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);


  // configure i2C
  Wire.setClock(400000);

  // initialize encoder sensor hardware
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

    // link driver
  motor.linkDriver(&driver);
  // link driver to cs
  current_sense.linkDriver(&driver);

  // current sense init hardware
  if(!current_sense.init()){
    Serial.println("Current sense init failed!");
    return;
  }

  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  // add target command M
  command.add('M',doMotor,"motor");

  // aligning voltage
  motor.voltage_sensor_align = 1.0;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;

  // set FOC modulation type to sinusoidal
  motor.foc_modulation = FOCModulationType::SinePWM;

  // velocity PID controller// velocity PID controller parameters
  // default P=0.5 I = 10 D =0    
  motor.PID_velocity.P = 0.18;
  motor.PID_velocity.I = 5.0;
  motor.PID_velocity.D = 0.005;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 300;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.02;

  // setting the limits
  // either voltage
  motor.voltage_limit = 5.0; // Volts - default driver.voltage_limit
  // of current 
  motor.current_limit = 15.0; // Amps - default 0.2Amps

  // angle PID controller 
  // default P=20
  motor.P_angle.P = 1.5;
  motor.P_angle.I = 0.0;  // usually only P controller is enough 
  motor.P_angle.D = 0.003;  // usually only P controller is enough 

  // acceleration control using output ramp
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor.P_angle.output_ramp = 10000; // default 1e6 rad/s^2

  // angle low pass filtering
  // default 0 - disabled  
  // use only for very noisy position sensors - try to avoid and keep the values very small
  motor.LPF_angle.Tf = 0; // default 0

  // setting the limits
  //  maximal velocity of the position control
  motor.velocity_limit = 2000;

  // use monitoring with serial
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable monitor at first - optional
  //motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

  motor.init();

  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the initial motor target
  motor.target = 0.0; // Volts 

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // display the currents
  motor.monitor();
  // user communication
  command.run();
}