# Exoskeleton Arm CRSF Transmitter

A sophisticated exoskeleton arm motion capture system that transmits joint angle data via CRSF (Crossfire) protocol to ELRS (ExpressLRS) transmitter modules. This system captures shoulder, upper arm, and lower arm movements using magnetic encoders and transmits normalized channel data for robotic control applications.

## Features

- **Multi-Joint Motion Capture**: Captures 3 joint angles (shoulder, upper arm, lower arm) with provision for 4th channel (claw/gripper)
- **CRSF Protocol Support**: Native ELRS transmitter module communication at 400kbaud
- **Dual Calibration System**: Zero-point calibration and range calibration for accurate motion mapping
- **Real-time Normalization**: Maps raw sensor data to normalized -1.0 to 1.0 range for consistent control
- **Persistent Storage**: Saves calibration data to ESP32 flash memory
- **ESP-NOW Backup**: Simultaneous wireless transmission to multiple ESP32 receivers
- **Hardware Controls**: Physical buttons for calibration and arming
- **Serial Interface**: Debug output and serial-based calibration commands

## Hardware Requirements

### Core Components
- **ESP32-S3 Development Board** (primary controller)
- **ELRS TX Module** (400kbaud CRSF communication)
- **3x AS5600 Magnetic Encoders** (I2C magnetic angle sensors)
- **TCA9548A I2C Multiplexer** (for multiple sensor support)

### Connections

#### I2C Bus (Sensors)
- **SDA**: GPIO6
- **SCL**: GPIO7
- **I2C Clock**: 400kHz

#### CRSF Communication (ELRS TX Module)
- **TX**: GPIO5 (to ELRS RX)
- **RX**: GPIO6 (from ELRS TX)
- **Wiring**: Connect GPIO5 and GPIO6 together externally for half-duplex operation

#### Control Buttons
- **Armed Switch**: GPIO16 (INPUT_PULLUP) - System enable/disable
- **Calibration Button**: GPIO18 (INPUT_PULLUP) - Zero-point calibration
- **Range Calibration Button**: GPIO19 (INPUT_PULLUP) - Min/max range calibration

#### AS5600 Sensors via TCA9548A
```
TCA9548A Channel 0 → Sensor1 (Shoulder)
TCA9548A Channel 1 → Sensor2 (Upper Arm)  
TCA9548A Channel 2 → Sensor3 (Lower Arm)
```

## Software Setup

### PlatformIO Configuration
This project uses PlatformIO with the ESP32-S3 platform. Required libraries:
- `SimpleFOC` - For magnetic sensor support
- `WiFi` - For ESP-NOW communication
- `Preferences` - For persistent storage
- Custom CRSF library (included)

### Installation
1. Clone this repository
2. Open in PlatformIO
3. Install dependencies: `pio lib install`
4. Upload to ESP32-S3: `pio run -t upload`

## Operation

### Initial Setup
1. Power on the system
2. Perform zero-point calibration (see Calibration section)
3. Perform range calibration for full motion mapping
4. System is ready for operation

### Channel Mapping
- **CRSF Channel 1**: Shoulder movement (ch1_normalized)
- **CRSF Channel 2**: Upper arm movement (ch2_normalized)
- **CRSF Channel 3**: Lower arm movement (ch3_normalized)
- **CRSF Channel 4**: Reserved for claw/gripper (ch4_normalized)
- **CRSF Channel 5**: Armed switch state (1000μs = OFF, 2000μs = ON)
- **CRSF Channels 6-16**: Neutral position (0.0)

### Data Output
- **CRSF Protocol**: 70Hz update rate to ELRS TX module
- **ESP-NOW**: Simultaneous broadcast to configured ESP32 receivers
- **Serial Debug**: Real-time angle and channel data at 115200 baud

## Calibration

### Zero-Point Calibration
Sets the neutral position for all joints.

**Methods:**
1. **Hardware Button**: Press and release GPIO18 button
2. **Serial Command**: Send `'c'` via serial monitor

**Process:**
1. Position arm in desired neutral/zero position
2. Trigger calibration
3. System saves current angles as zero offsets
4. Confirmation message displayed

### Range Calibration
Defines the full range of motion for accurate normalization.

**Methods:**
1. **Hardware Button**: Press GPIO19 button to start/stop
2. **Serial Command**: Send `'r'` to start/stop

**Process:**
1. Start range calibration mode
2. Move each joint through its complete range of motion
3. System tracks minimum and maximum values
4. Stop calibration to save ranges
5. Values normalized to -1.0 to 1.0 range

## Serial Commands

Connect at **115200 baud** for debug output and control:

- `'c'` - Perform zero-point calibration
- `'r'` - Start/stop range calibration mode

### Debug Output Example
```
=== CRSF TX Half-Duplex Test ===
ESP32-S3 Chip Model: ESP32-S3
ARMED SWITCH: ON (grounded)
0.245 SHOULDER RAD / 14.04°
-0.523 UPPER ARM RAD / -29.96°
1.047 LOWER ARM RAD / 59.99°
Normalized channels: CH1=0.156 CH2=-0.333 CH3=0.667 CH4=0.000
---   Sent with success
```

## Data Storage

All calibration data is automatically saved to ESP32 flash memory:

### Saved Parameters
- **Zero Offsets**: `ch1_offset`, `ch2_offset`, `ch3_offset`
- **Range Minimums**: `ch1_min`, `ch2_min`, `ch3_min`, `ch4_min`
- **Range Maximums**: `ch1_max`, `ch2_max`, `ch3_max`, `ch4_max`

### Default Values
- **Offsets**: 0.0 radians
- **Ranges**: ±1.57 radians (±90°) for joints, ±1.0 for channel 4

## Network Configuration

### ESP-NOW Receivers
Update the MAC addresses in the code for your specific receivers:
```cpp
uint8_t broadcastAddress1[] = {0xA8, 0x42, 0xE3, 0xE4, 0x06, 0xA4};
uint8_t broadcastAddress2[] = {0xC4, 0xDE, 0xE2, 0xF5, 0x72, 0xC0}; 
uint8_t broadcastAddress3[] = {0x64, 0xE8, 0x33, 0x73, 0xD3, 0xB4};
```

### Data Structure
```cpp
typedef struct struct_message {
  float upperarm_rad;    // Upper arm angle in radians
  float lowerarm_rad;    // Lower arm angle in radians  
  float shoulder_rad;    // Shoulder angle in radians
  bool power;           // Armed switch state
} struct_message;
```

## Troubleshooting

### Common Issues
1. **No CRSF Output**: Check GPIO5/6 wiring to ELRS module
2. **Sensor Not Detected**: Verify I2C connections and TCA9548A wiring
3. **Calibration Not Saving**: Ensure preferences namespace is accessible
4. **Erratic Readings**: Check for magnetic interference near AS5600 sensors

### Verification Steps
1. Monitor serial output for sensor readings
2. Verify normalized values are within -1.0 to 1.0 range
3. Test calibration functions via serial commands
4. Check ESP-NOW transmission success messages

## Advanced Configuration

### Timing Adjustments
- **CRSF Update Rate**: Modify `CRSF_UPDATE_INTERVAL` (default: 14ms = ~70Hz)
- **Loop Delay**: Adjust `delay(50)` at end of main loop
- **I2C Clock**: Current setting 400kHz, can be reduced if needed

### Sensor Scaling
- **Shoulder**: Currently divided by 4 for reduced sensitivity
- **Upper/Lower Arm**: Direct 1:1 mapping
- Modify scaling factors in the angle calculation section

## License

This project is open source. Please refer to the license file for details.

## Contributing

Contributions are welcome! Please submit pull requests with:
- Clear description of changes
- Tested functionality
- Updated documentation if needed

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review serial debug output
3. Open an issue with detailed description and debug logs
