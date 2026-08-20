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
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <SimpleFOC.h>

#include <Preferences.h>
Preferences preferences;

bool calibrationMode = false;  // Set to true if you want to force calibration
bool motorCalibrationMode = false;  // Set to true if you want to force motor recalibration

// Structure to hold motor calibration data
typedef struct {
    float zero_electric_angle;     // Motor electrical zero angle
    Direction sensor_direction;    // Sensor direction (CW or CCW)
    bool initialized;              // Whether motor has been calibrated
} motor_calibration_t;

motor_calibration_t motor0_cal = {0};
motor_calibration_t motor1_cal = {0};

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

// ESP-NOW bidirectional communication data structure
// Must match the sender structure exactly
typedef struct __attribute__((packed)) {
    char device_role[16];        // Device identifier (e.g., "LEFT_SHOULDER", "LEFT_ARM")
    uint8_t channel_count;       // Number of channels in this packet (1 for shoulder, 2 for arm)
    float channel_2;             // CRSF channel 2 angle (shoulder or upper arm)
    float channel_3;             // CRSF channel 3 angle (elbow)
    uint8_t armed;               // Arm/disarm status (uint8_t instead of bool for C/C++ compatibility)
    uint8_t calibrate_command;   // Calibrate command (uint8_t instead of bool for C/C++ compatibility)
    uint32_t timestamp;          // Timestamp of the measurement
    uint32_t sequence_number;    // Packet sequence number
    uint8_t checksum;            // Simple checksum for data integrity
} struct_message;

// Create a struct_message called myData
struct_message myData;

float ch0_angle;
float ch1_angle;
bool pwr;

float ch0_offset;
float ch1_offset;

float ch0_move = 0.0;
float ch1_move = 0.0;

// Travel range limits for each motor
typedef struct {
    float min_angle;     // Minimum angle at one physical limit
    float max_angle;     // Maximum angle at other physical limit
    bool calibrated;     // Whether range has been calibrated
} motor_range_t;

motor_range_t motor0_range = {0};
motor_range_t motor1_range = {0};

// Function declarations
void saveMotorCalibration();
bool loadMotorCalibration();
void applyMotorCalibration();
void triggerMotorRecalibration();
void saveMotorRangeLimits();
bool loadMotorRangeLimits();
void calibrateMotorTravelRange();
float constrainToRange(float target, motor_range_t range);
float mapInputToAngle(float input, motor_range_t range);
void printMotorRanges();

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Check if received packet size matches expected structure
  if (len != sizeof(myData)) {
    Serial.printf("ESP-NOW: Received packet wrong size: %d bytes, expected: %d\n", len, sizeof(myData));
    return;
  }

  memcpy(&myData, incomingData, sizeof(myData));

  // Verify checksum (simple sum of bytes)
  uint8_t calc_checksum = 0;
  uint8_t *packet_bytes = (uint8_t*)&myData;
  for (int i = 0; i < sizeof(myData) - 1; i++) {  // Exclude checksum byte
    calc_checksum += packet_bytes[i];
  }

  if (calc_checksum != myData.checksum) {
    Serial.println("ESP-NOW: Checksum mismatch - packet corrupted");
    return;
  }

  // Map remote data to local variables based on new structure
  if (myData.channel_count >= 1) {
    ch0_angle = myData.channel_2;               // CRSF channel 2 (upper arm) -> ch0
  }
  if (myData.channel_count >= 2) {
    ch1_angle = myData.channel_3;               // CRSF channel 3 (elbow) -> ch1
  }
  
  // Use armed status to determine power state
  pwr = myData.armed;
  
  // Debug output
  Serial.printf("ESP-NOW RX: %s CH2:%.1f° CH3:%.1f° Armed:%s Cal:%s (seq:%lu)\n", 
               myData.device_role, 
               myData.channel_count >= 1 ? myData.channel_2 : 0.0f,
               myData.channel_count >= 2 ? myData.channel_3 : 0.0f,
               myData.armed ? "YES" : "NO",
               myData.calibrate_command ? "YES" : "NO",
               myData.sequence_number);
}

//Command Settings
//Enter "T+Position" in the serial monitor to make the two motors rotate in closed loop
//For example, input the radian system "T3.14" to let the two motors rotate 180°
float target_position = 0;      // Target for both motors
float target_motor0 = 0;        // Target for motor0 only
float target_motor1 = 0;        // Target for motor1 only
bool manual_control_active = false;  // Track if manual control is active
bool calibration_in_progress = false; // Flag to disable motor control during calibration
bool use_individual_control = false;  // Track if using A/B commands vs T command

Commander command = Commander(Serial);
void doTarget(char* cmd) { 
    command.scalar(&target_position, cmd); 
    manual_control_active = true;
    use_individual_control = false;  // Using T command (both motors)
    target_motor0 = 0;  // Clear individual targets
    target_motor1 = 0;
}
void doMotor0(char* cmd) { 
    command.scalar(&target_motor0, cmd); 
    manual_control_active = true;
    use_individual_control = true;  // Using A/B commands
    target_position = 0;  // Clear combined target
}
void doMotor1(char* cmd) { 
    command.scalar(&target_motor1, cmd); 
    manual_control_active = true;
    use_individual_control = true;  // Using A/B commands
    target_position = 0;  // Clear combined target
}
void doRangeCalibration(char* cmd) { calibrateMotorTravelRange(); }
void doShowRanges(char* cmd) { printMotorRanges(); }
void doMotorRecalibration(char* cmd) { triggerMotorRecalibration(); }
void doDebugMotors(char* cmd) {
    Serial.println("\n========================================");
    Serial.println("MOTOR DEBUG STATUS");
    Serial.println("========================================");
    
    // Check Motor 0
    Serial.println("Motor0:");
    Serial.printf("  Enabled: %s\n", motor0.enabled ? "YES" : "NO");
    sensor0.update();
    Serial.printf("  Sensor angle: %.6f rad (%.2f°)\n", sensor0.getAngle(), sensor0.getAngle() * 180.0 / PI);
    Serial.printf("  Target angle: %.6f rad (%.2f°)\n", ch0_move, ch0_move * 180.0 / PI);
    Serial.printf("  Offset: %.6f rad (%.2f°)\n", ch0_offset, ch0_offset * 180.0 / PI);
    Serial.printf("  Shaft angle: %.6f rad (%.2f°)\n", motor0.shaft_angle, motor0.shaft_angle * 180.0 / PI);
    Serial.printf("  Voltage Q: %.2f V\n", motor0.voltage.q);
    
    Serial.println("\nMotor1:");
    Serial.printf("  Enabled: %s\n", motor1.enabled ? "YES" : "NO");
    sensor1.update();
    Serial.printf("  Sensor angle: %.6f rad (%.2f°)\n", sensor1.getAngle(), sensor1.getAngle() * 180.0 / PI);
    Serial.printf("  Target angle: %.6f rad (%.2f°)\n", ch1_move, ch1_move * 180.0 / PI);
    Serial.printf("  Offset: %.6f rad (%.2f°)\n", ch1_offset, ch1_offset * 180.0 / PI);
    Serial.printf("  Shaft angle: %.6f rad (%.2f°)\n", motor1.shaft_angle, motor1.shaft_angle * 180.0 / PI);
    Serial.printf("  Voltage Q: %.2f V\n", motor1.voltage.q);
    
    Serial.println("\nControl State:");
    Serial.printf("  ESP-NOW Armed: %s\n", pwr ? "YES" : "NO");
    Serial.printf("  Manual control: %s\n", manual_control_active ? "YES" : "NO");
    Serial.printf("  Calibration in progress: %s\n", calibration_in_progress ? "YES" : "NO");
    Serial.printf("  CH0 angle (ESP-NOW): %.1f\n", ch0_angle);
    Serial.printf("  CH1 angle (ESP-NOW): %.1f\n", ch1_angle);
    
    Serial.println("========================================\n");
}

// Function to save motor calibration data to Preferences
void saveMotorCalibration() {
    preferences.begin("motor_cal", false); // false = write mode
    
    // Save motor0 calibration data
    preferences.putFloat("m0_zero_angle", motor0.zero_electric_angle);
    preferences.putInt("m0_sensor_dir", (int)motor0.sensor_direction);  // Cast Direction to int for storage
    preferences.putBool("m0_initialized", true);
    
    // Save motor1 calibration data
    preferences.putFloat("m1_zero_angle", motor1.zero_electric_angle);
    preferences.putInt("m1_sensor_dir", (int)motor1.sensor_direction);  // Cast Direction to int for storage
    preferences.putBool("m1_initialized", true);
    
    preferences.end();
    
    Serial.println("Motor calibration data saved to flash:");
    Serial.printf("  Motor0: Zero=%.6f, Dir=%s\n", motor0.zero_electric_angle, 
                  motor0.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial.printf("  Motor1: Zero=%.6f, Dir=%s\n", motor1.zero_electric_angle,
                  motor1.sensor_direction == Direction::CW ? "CW" : "CCW");
}

// Function to load motor calibration data from Preferences
bool loadMotorCalibration() {
    preferences.begin("motor_cal", true); // true = read-only
    
    // Check if calibration data exists
    bool m0_exists = preferences.getBool("m0_initialized", false);
    bool m1_exists = preferences.getBool("m1_initialized", false);
    
    if (m0_exists && m1_exists) {
        // Load motor0 calibration data
        motor0_cal.zero_electric_angle = preferences.getFloat("m0_zero_angle", 0.0);
        int dir0 = preferences.getInt("m0_sensor_dir", 1);
        motor0_cal.sensor_direction = (dir0 == 1) ? Direction::CW : Direction::CCW;
        motor0_cal.initialized = true;
        
        // Load motor1 calibration data
        motor1_cal.zero_electric_angle = preferences.getFloat("m1_zero_angle", 0.0);
        int dir1 = preferences.getInt("m1_sensor_dir", 1);
        motor1_cal.sensor_direction = (dir1 == 1) ? Direction::CW : Direction::CCW;
        motor1_cal.initialized = true;
        
        preferences.end();
        
        Serial.println("Motor calibration data loaded from flash:");
        Serial.printf("  Motor0: Zero=%.6f, Dir=%s\n", motor0_cal.zero_electric_angle,
                      motor0_cal.sensor_direction == Direction::CW ? "CW" : "CCW");
        Serial.printf("  Motor1: Zero=%.6f, Dir=%s\n", motor1_cal.zero_electric_angle,
                      motor1_cal.sensor_direction == Direction::CW ? "CW" : "CCW");
        
        return true;
    } else {
        preferences.end();
        Serial.println("No saved motor calibration data found - will perform full calibration");
        return false;
    }
}

// Function to apply loaded calibration data to motors
void applyMotorCalibration() {
    if (motor0_cal.initialized) {
        motor0.zero_electric_angle = motor0_cal.zero_electric_angle;
        motor0.sensor_direction = motor0_cal.sensor_direction;
        Serial.printf("Applied saved calibration to Motor0: Zero=%.6f, Dir=%s\n", 
                     motor0.zero_electric_angle, 
                     motor0.sensor_direction == Direction::CW ? "CW" : "CCW");
    }
    
    if (motor1_cal.initialized) {
        motor1.zero_electric_angle = motor1_cal.zero_electric_angle;
        motor1.sensor_direction = motor1_cal.sensor_direction;
        Serial.printf("Applied saved calibration to Motor1: Zero=%.6f, Dir=%s\n", 
                     motor1.zero_electric_angle,
                     motor1.sensor_direction == Direction::CW ? "CW" : "CCW");
    }
}

void setup() {
  pinMode(CALIBRATION_SWITCH_PIN, INPUT);

  // Load saved offsets from Preferences
  preferences.begin("calibration", true); // true = read-only
  ch0_offset = preferences.getFloat("ch0_offset", 0.0);
  ch1_offset = preferences.getFloat("ch1_offset", 0.0);
  preferences.end();

  // Load saved motor calibration data
  bool motorCalDataExists = loadMotorCalibration();
  
  // Load saved motor range limits
  bool rangeDataExists = loadMotorRangeLimits();

  // OPTIONAL: Recalibrate if flag is set or command is received
  if (calibrationMode) {
    sensor0.update();
    ch0_offset = sensor0.getAngle();

    sensor1.update();
    ch1_offset = sensor1.getAngle();

    preferences.begin("calibration", false); // false = write mode
    preferences.putFloat("ch0_offset", ch0_offset);
    preferences.putFloat("ch1_offset", ch1_offset);
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
  
  //Initialize FOC - with saved calibration data if available
  if (motorCalDataExists && !motorCalibrationMode) {
    Serial.println("Using saved motor calibration data...");
    
    // Apply saved calibration before initFOC
    applyMotorCalibration();
    
    // Skip full calibration by using initFOC with minimal calibration
    motor0.initFOC(motor0_cal.zero_electric_angle, motor0_cal.sensor_direction);
    motor1.initFOC(motor1_cal.zero_electric_angle, motor1_cal.sensor_direction);
    
    Serial.println("Motors initialized with saved calibration - skipped full calibration");
  } else {
    Serial.println("Performing full motor calibration (this may take a moment)...");
    
    // Perform full FOC calibration
    motor0.initFOC();
    motor1.initFOC();
    
    // Save the calibration data after successful initialization
    saveMotorCalibration();
    
    Serial.println("Full motor calibration completed and saved");
  }
  
  command.add('T', doTarget, "target position both motors");
  command.add('A', doMotor0, "target position motor0");
  command.add('B', doMotor1, "target position motor1");
  command.add('R', doRangeCalibration, "calibrate motor travel range");
  command.add('S', doShowRanges, "show motor ranges");
  command.add('M', doMotorRecalibration, "recalibrate motors (FOC)");
  command.add('D', doDebugMotors, "debug motor status");

  Serial.println(F("Motor ready."));
  Serial.println(F("Commands (work when ESP-NOW disarmed):"));
  Serial.println(F("  T<angle> - Move BOTH motors (radians, relative to center)"));
  Serial.println(F("  A<angle> - Move MOTOR0 only (radians, relative to center)"));
  Serial.println(F("  B<angle> - Move MOTOR1 only (radians, relative to center)"));
  Serial.println(F("  T0 A0 B0 - Reset to center/home position"));
  Serial.println(F("  Example: T0.1 = move 0.1rad (~5.7°) from center"));
  Serial.println(F("Calibration commands:"));
  Serial.println(F("  R - Calibrate motor travel range"));
  Serial.println(F("  S - Show motor ranges"));
  Serial.println(F("  M - Recalibrate motor FOC"));
  Serial.println(F("  D - Debug motor status"));
}

void loop() {
  // Skip all motor control if calibration is in progress
  if (!calibration_in_progress) {
    motor0.loopFOC();
    motor1.loopFOC();

    if (pwr) {
      // ESP-NOW ARMED: Map 1000-2000 input values to calibrated angle ranges
      ch0_move = mapInputToAngle(ch0_angle, motor0_range);
      ch1_move = mapInputToAngle(ch1_angle, motor1_range);
      manual_control_active = false;  // Clear manual control when ESP-NOW takes over
    }
    else {
      // ESP-NOW DISARMED: Use manual command control if set, otherwise hold at offset
      if (manual_control_active) {
        // Manual control mode - use individual targets or combined target
        if (use_individual_control) {
          // A/B commands: individual motor control (relative to center/offset)
          ch0_move = ch0_offset + target_motor0;
          ch1_move = ch1_offset + target_motor1;
        } else {
          // T command: both motors move together (relative to center/offset)
          ch0_move = ch0_offset + target_position;
          ch1_move = ch1_offset + target_position;
        }
      } else {
        // Default: return to offset/home positions
        ch0_move = ch0_offset;
        ch1_move = ch1_offset;
      }
    }

    motor0.move(ch0_move);
    motor1.move(ch1_move);
  }

  command.run();

  // Calibration via physical switch or ESP-NOW remote command
  bool switchState2 = digitalRead(CALIBRATION_SWITCH_PIN);
  static bool lastRemoteCalibrateState = false;
  
  bool triggerCalibration = false;
  
  // Check physical button trigger (falling edge)
  if (switchState2 == LOW && !lastPowerState_CALIBRATION_SWITCH) {
    triggerCalibration = true;
    Serial.println("Calibration button pressed. Saving new offsets...");
  }
  
  // Check remote calibrate command trigger (rising edge)
  if (myData.calibrate_command && !lastRemoteCalibrateState) {
    triggerCalibration = true;
    Serial.println("Remote calibration command received. Saving new offsets...");
  }
  
  // Perform calibration if triggered by either method
  if (triggerCalibration) {
    sensor0.update(); ch0_offset = sensor0.getAngle();
    sensor1.update(); ch1_offset = sensor1.getAngle();

    preferences.begin("calibration", false);
    preferences.putFloat("ch0_offset", ch0_offset);
    preferences.putFloat("ch1_offset", ch1_offset);
    preferences.end();

    Serial.printf("New offsets saved: CH0=%.3f, CH1=%.3f\n", ch0_offset, ch1_offset);
  }
  
  lastPowerState_CALIBRATION_SWITCH = (switchState2 == LOW);
  lastRemoteCalibrateState = myData.calibrate_command;
}

// Function to trigger motor recalibration (call this manually when needed)
void triggerMotorRecalibration() {
    Serial.println("Starting motor recalibration...");
    
    // Perform full FOC recalibration
    motor0.initFOC();
    motor1.initFOC();
    
    // Save the new calibration data
    saveMotorCalibration();
    
    Serial.println("Motor recalibration completed and saved!");
}

// Function to save motor range limits to Preferences
void saveMotorRangeLimits() {
    preferences.begin("motor_range", false); // false = write mode
    
    // Save motor0 range
    preferences.putFloat("m0_min_angle", motor0_range.min_angle);
    preferences.putFloat("m0_max_angle", motor0_range.max_angle);
    preferences.putBool("m0_range_cal", motor0_range.calibrated);
    
    // Save motor1 range
    preferences.putFloat("m1_min_angle", motor1_range.min_angle);
    preferences.putFloat("m1_max_angle", motor1_range.max_angle);
    preferences.putBool("m1_range_cal", motor1_range.calibrated);
    
    preferences.end();
    
    Serial.println("Motor range limits saved to flash:");
    Serial.printf("  Motor0: Min=%.3f°, Max=%.3f°\n", motor0_range.min_angle, motor0_range.max_angle);
    Serial.printf("  Motor1: Min=%.3f°, Max=%.3f°\n", motor1_range.min_angle, motor1_range.max_angle);
}

// Function to load motor range limits from Preferences
bool loadMotorRangeLimits() {
    preferences.begin("motor_range", true); // true = read-only
    
    bool m0_cal = preferences.getBool("m0_range_cal", false);
    bool m1_cal = preferences.getBool("m1_range_cal", false);
    
    if (m0_cal && m1_cal) {
        motor0_range.min_angle = preferences.getFloat("m0_min_angle", 0.0);
        motor0_range.max_angle = preferences.getFloat("m0_max_angle", 0.0);
        motor0_range.calibrated = true;
        
        motor1_range.min_angle = preferences.getFloat("m1_min_angle", 0.0);
        motor1_range.max_angle = preferences.getFloat("m1_max_angle", 0.0);
        motor1_range.calibrated = true;
        
        preferences.end();
        
        Serial.println("Motor range limits loaded from flash:");
        Serial.printf("  Motor0: Min=%.3f°, Max=%.3f°\n", motor0_range.min_angle, motor0_range.max_angle);
        Serial.printf("  Motor1: Min=%.3f°, Max=%.3f°\n", motor1_range.min_angle, motor1_range.max_angle);
        
        return true;
    } else {
        preferences.end();
        Serial.println("No saved motor range limits found");
        return false;
    }
}

// Function to calibrate motor travel range limits
// This will move each motor to its physical limits and record the angles
void calibrateMotorTravelRange() {
    Serial.println("\n========================================");
    Serial.println("MOTOR TRAVEL RANGE CALIBRATION");
    Serial.println("========================================");
    Serial.println("This will disable motor control so you can manually position motors.");
    Serial.println("Make sure the motion path is clear!");
    Serial.println("Press any key to continue or 'q' to cancel...");
    
    // Wait for user confirmation
    while (!Serial.available()) {
        delay(100);
    }
    
    char input = Serial.read();
    while (Serial.available()) Serial.read(); // Clear buffer
    
    if (input == 'q' || input == 'Q') {
        Serial.println("Range calibration cancelled.");
        return;
    }
    
    // Disable motor control during calibration
    calibration_in_progress = true;
    
    // Actively disable both motors
    motor0.disable();
    motor1.disable();
    
    Serial.println("\nStarting range calibration...");
    Serial.println("MOTOR POWER DISABLED - You can now manually move the motors.");
    delay(1000);
    
    // Step 1: Calibrate BOTH motors at MINIMUM positions
    Serial.println("\n========================================");
    Serial.println("STEP 1: MINIMUM POSITIONS");
    Serial.println("========================================");
    Serial.println("Move BOTH Motor0 AND Motor1 to their MINIMUM positions (lower limits)");
    Serial.println("Then press any key...");
    
    while (!Serial.available()) {
        sensor0.update();
        sensor1.update();
        delay(10);
    }
    while (Serial.available()) Serial.read(); // Clear buffer
    
    sensor0.update();
    sensor1.update();
    motor0_range.min_angle = sensor0.getAngle();
    motor1_range.min_angle = sensor1.getAngle();
    
    Serial.println("\nMINIMUM positions recorded:");
    Serial.printf("  Motor0 MIN: %.6f radians (%.2f degrees)\n", 
                  motor0_range.min_angle, motor0_range.min_angle * 180.0 / PI);
    Serial.printf("  Motor1 MIN: %.6f radians (%.2f degrees)\n", 
                  motor1_range.min_angle, motor1_range.min_angle * 180.0 / PI);
    
    // Step 2: Calibrate BOTH motors at MAXIMUM positions
    Serial.println("\n========================================");
    Serial.println("STEP 2: MAXIMUM POSITIONS");
    Serial.println("========================================");
    Serial.println("Move BOTH Motor0 AND Motor1 to their MAXIMUM positions (upper limits)");
    Serial.println("Then press any key...");
    
    while (!Serial.available()) {
        sensor0.update();
        sensor1.update();
        delay(10);
    }
    while (Serial.available()) Serial.read(); // Clear buffer
    
    sensor0.update();
    sensor1.update();
    motor0_range.max_angle = sensor0.getAngle();
    motor1_range.max_angle = sensor1.getAngle();
    
    Serial.println("\nMAXIMUM positions recorded:");
    Serial.printf("  Motor0 MAX: %.6f radians (%.2f degrees)\n", 
                  motor0_range.max_angle, motor0_range.max_angle * 180.0 / PI);
    Serial.printf("  Motor1 MAX: %.6f radians (%.2f degrees)\n", 
                  motor1_range.max_angle, motor1_range.max_angle * 180.0 / PI);
    
    motor0_range.calibrated = true;
    motor1_range.calibrated = true;
    
    // Calculate center points as new zero/home positions
    ch0_offset = (motor0_range.min_angle + motor0_range.max_angle) / 2.0;
    ch1_offset = (motor1_range.min_angle + motor1_range.max_angle) / 2.0;
    
    // Save the new offsets to Preferences
    preferences.begin("calibration", false); // false = write mode
    preferences.putFloat("ch0_offset", ch0_offset);
    preferences.putFloat("ch1_offset", ch1_offset);
    preferences.end();
    
    // Save the calibration data
    saveMotorRangeLimits();
    
    // Step 3: Confirm before re-enabling motors
    Serial.println("\n========================================");
    Serial.println("STEP 3: PREPARE TO RE-ENABLE MOTORS");
    Serial.println("========================================");
    Serial.println("Calibration data saved to flash.");
    Serial.println("\nMake sure motors are in a safe position!");
    Serial.println("Motors will power on and move to CENTER position when enabled.");
    Serial.printf("  Motor0 will move to: %.2f° (%.6f rad)\n", 
                  ch0_offset * 180.0 / PI, ch0_offset);
    Serial.printf("  Motor1 will move to: %.2f° (%.6f rad)\n", 
                  ch1_offset * 180.0 / PI, ch1_offset);
    Serial.println("\nPress any key to RE-ENABLE motors, or 'q' to keep them disabled...");
    
    // Wait for confirmation
    while (!Serial.available()) {
        delay(100);
    }
    
    char confirm = Serial.read();
    while (Serial.available()) Serial.read(); // Clear buffer
    
    if (confirm == 'q' || confirm == 'Q') {
        Serial.println("\nMotors remain DISABLED.");
        Serial.println("Use 'M' command to recalibrate FOC when ready.");
        // Keep motors disabled and calibration mode active
        return;
    }
    
    // Re-enable motors
    Serial.println("\nRe-enabling motors...");
    motor0.enable();
    motor1.enable();
    
    // Re-enable motor control
    calibration_in_progress = false;
    
    Serial.println("\n========================================");
    Serial.println("CALIBRATION COMPLETE!");
    Serial.println("========================================");
    Serial.printf("Motor0 Range: %.2f° to %.2f° (%.2f° total)\n", 
                  motor0_range.min_angle * 180.0 / PI, 
                  motor0_range.max_angle * 180.0 / PI,
                  (motor0_range.max_angle - motor0_range.min_angle) * 180.0 / PI);
    Serial.printf("Motor0 Center (new zero): %.2f° (%.6f rad)\n",
                  ch0_offset * 180.0 / PI, ch0_offset);
    Serial.printf("Motor1 Range: %.2f° to %.2f° (%.2f° total)\n", 
                  motor1_range.min_angle * 180.0 / PI, 
                  motor1_range.max_angle * 180.0 / PI,
                  (motor1_range.max_angle - motor1_range.min_angle) * 180.0 / PI);
    Serial.printf("Motor1 Center (new zero): %.2f° (%.6f rad)\n",
                  ch1_offset * 180.0 / PI, ch1_offset);
    Serial.println("New offsets saved to flash.");
    Serial.println("MOTOR POWER RE-ENABLED");
    Serial.println("========================================\n");
}

// Helper function to constrain a target angle within calibrated range
float constrainToRange(float target, motor_range_t range) {
    if (!range.calibrated) {
        return target; // No constraints if not calibrated
    }
    
    // Constrain target between min and max
    if (target < range.min_angle) {
        return range.min_angle;
    } else if (target > range.max_angle) {
        return range.max_angle;
    }
    return target;
}

// Function to map RC input (1000-2000 µs) to motor angle range
// input: PWM value from ESP-NOW (1000-2000)
// range: calibrated motor range limits
// Returns: target angle in radians, mapped and constrained to physical limits
float mapInputToAngle(float input, motor_range_t range) {
    // Constrain input to valid RC range
    float constrained_input = constrain(input, 1000.0, 2000.0);
    
    if (!range.calibrated) {
        // If not calibrated, use a default mapping (e.g., -PI to PI)
        return map(constrained_input, 1000.0, 2000.0, -PI, PI);
    }
    
    // Map 1000-2000 input to calibrated min_angle to max_angle range
    float mapped_angle = map(constrained_input, 1000.0, 2000.0, range.min_angle, range.max_angle);
    
    return mapped_angle;
}

// Function to display current motor ranges
void printMotorRanges() {
    Serial.println("\n========================================");
    Serial.println("MOTOR TRAVEL RANGES");
    Serial.println("========================================");
    
    if (motor0_range.calibrated) {
        Serial.printf("Motor0: %.2f° to %.2f° (Range: %.2f°)\n", 
                      motor0_range.min_angle * 180.0 / PI, 
                      motor0_range.max_angle * 180.0 / PI,
                      (motor0_range.max_angle - motor0_range.min_angle) * 180.0 / PI);
    } else {
        Serial.println("Motor0: NOT CALIBRATED");
    }
    
    if (motor1_range.calibrated) {
        Serial.printf("Motor1: %.2f° to %.2f° (Range: %.2f°)\n", 
                      motor1_range.min_angle * 180.0 / PI, 
                      motor1_range.max_angle * 180.0 / PI,
                      (motor1_range.max_angle - motor1_range.min_angle) * 180.0 / PI);
    } else {
        Serial.println("Motor1: NOT CALIBRATED");
    }
    
    Serial.println("========================================\n");
}