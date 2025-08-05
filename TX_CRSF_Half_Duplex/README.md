# Exoskeleton Arm CRSF Transmitter with ESP-NOW Display

This project reads 4 AS5600 magnetic sensors from an exoskeleton arm and transmits the joint angle data via two methods:
1. **CRSF channels** to an ExpressLRS (ELRS) transmitter module for EdgeTX/OpenTX radio systems
2. **ESP-NOW wireless** to a display device for real-time monitoring

## Features

- Reads 4 AS5600 magnetic position sensors via I2C with TCA9548A multiplexer
- **Dual communication**:
  - CRSF protocol at 250Hz to ELRS TX module via half-duplex UART
  - ESP-NOW wireless communication to display device
- Compatible with EdgeTX/OpenTX radio systems
- Real-time sensor zeroing and angle calculations
- Power switch input mapped to both CRSF channel and ESP-NOW data

## Hardware Requirements

### Components
- ESP32-S3 development board
- 4x AS5600 magnetic position sensors
- 1x TCA9548A I2C multiplexer
- 1x ExpressLRS transmitter module
- 1x ESP32 display device (for wireless monitoring)
- Power switch (optional)
- Connecting wires

### Wiring Connections

#### I2C Sensors (via TCA9548A Multiplexer)
```
ESP32-S3    TCA9548A
GPIO 6  ->  SDA
GPIO 7  ->  SCL
3.3V    ->  VCC
GND     ->  GND

TCA9548A    AS5600 Sensors
SD0/SC0 ->  Sensor 0 (Channel 0: Shoulder Flex) SDA/SCL
SD1/SC1 ->  Sensor 1 (Channel 1: Elbow) SDA/SCL  
SD2/SC2 ->  Sensor 2 (Channel 2: Shoulder Rotation) SDA/SCL
SD3/SC3 ->  Sensor 3 (Channel 3: Claw) SDA/SCL
```

#### CRSF Communication to ELRS TX Module
```
ESP32-S3    ELRS TX Module
GPIO 17 ->  TX (CRSF input)
GPIO 16 ->  RX (CRSF output) - optional for half-duplex
GND     ->  GND
```

#### Power Switch (Optional)
```
ESP32-S3    Switch
GPIO 4  ->  Switch input (active LOW)
GND     ->  Switch GND
```

#### ESP-NOW Display Communication
```
ESP32-S3 Main Unit    ESP32 Display Unit
WiFi (2.4GHz)     <-> WiFi (2.4GHz)
MAC: Source       <-> MAC: A8:42:E3:E4:06:A4 (update as needed)
```

**Note**: Update the `displayAddress[]` array in the code with your display device's actual MAC address.

## CRSF Channel Mapping

The sensor data is mapped to CRSF channels as follows:

| Channel | Function | Range | Notes |
|---------|----------|-------|--------|
| 0 | Shoulder Flex (Sensor 0) | -180° to +180° | Mapped to 1000-2000μs |
| 1 | Elbow (Sensor 1) | -180° to +180° | Mapped to 1000-2000μs |
| 2 | Shoulder Rotation (Sensor 2) | -180° to +180° | Mapped to 1000-2000μs |
| 3 | Claw (Sensor 3) | -180° to +180° | Mapped to 1000-2000μs |
| 4 | Unused | Center | Set to 1500μs (neutral) |
| 5 | Power Switch | ON/OFF | 2000μs = ON, 1000μs = OFF |
| 6-15 | Unused | Center | All set to 1500μs (neutral) |

## Communication Protocols

### CRSF to ELRS TX Module
- **Baud Rate**: 400,000 bps (ELRS standard)
- **Frame Rate**: 250Hz (4ms intervals)
- **Frame Type**: 0x16 (RC Channels Packed)
- **Channel Resolution**: 11-bit (172-1811 range, center=992)
- **CRC**: 8-bit CRC with polynomial 0xD5

### ESP-NOW to Display Device
- **Protocol**: ESP-NOW (direct WiFi communication)
- **Data Structure**: 
  ```cpp
  struct {
      float ch0_rad;      // Shoulder flex angle (radians)
      float ch1_rad;      // Elbow angle (radians)  
      float ch2_rad;      // Shoulder rotation angle (radians)
      float ch3_rad;      // Claw angle (radians)
      bool power;         // Power switch state
      uint32_t timestamp; // Millisecond timestamp
  }
  ```
- **Update Rate**: Same as main loop (~10Hz with 100ms delay)

## Configuration

### ELRS TX Module Setup
1. Connect the ESP32 to your ELRS TX module via UART
2. The ELRS module should auto-detect the 400kbps CRSF signal
3. In your EdgeTX/OpenTX radio, set up the model to receive CRSF input
4. Map the incoming channels to your desired functions

### ESP-NOW Display Setup
1. Get the MAC address of your display device ESP32
2. Update the `displayAddress[]` array in main.cpp with the correct MAC address:
   ```cpp
   uint8_t displayAddress[] = {0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY}; // Your display MAC
   ```
3. Ensure the display device is configured to receive ESP-NOW data
4. The display will receive angle data in radians for all 4 joints and power state

### EdgeTX/OpenTX Configuration
1. Create a new model in EdgeTX/OpenTX
2. Set external RF protocol to CRSF
3. Configure channel mapping for the 5 active channels (4 joints + power)
4. Set up mixers/curves as needed for your exoskeleton control

## Code Structure

- `main.cpp`: Main application logic, sensor reading, and CRSF transmission
- `crsf.h`: CRSF protocol class declaration
- `crsf.cpp`: CRSF protocol implementation with frame packing and CRC
- `crsf_protocol.h`: CRSF protocol constants and structures

## Key Functions

### CrsfSerial Class Methods
- `begin(baudrate)`: Initialize UART communication
- `setChannelFloat(channel, value)`: Set channel with float value (-1.0 to +1.0)
- `setChannelUs(channel, microseconds)`: Set channel with microsecond value (1000-2000)
- `update()`: Maintain proper transmission frame rate (call in loop)
- `sendChannels()`: Manually send CRSF frame

### Sensor Functions
- `TCA9548A(bus)`: Select I2C multiplexer channel
- Sensor initialization and angle reading via SimpleFOC library

## Customization

### Changing Pin Assignments
Modify these lines in `main.cpp`:
```cpp
// CRSF Serial interface pins
CrsfSerial crsf(Serial1, 16, 17, true); // RX, TX, half-duplex

// I2C pins for sensors
Wire.begin(6, 7, (uint32_t)400000); // SDA, SCL

// Power switch pin
const int switchPin = 4;
```

### Adjusting Sensor Mapping
Modify the channel mapping logic in the main loop:
```cpp
// Example: Invert sensor direction
float ch0_normalized = -ch0_angle_pulley / (2 * PI);

// Example: Scale sensor range
float ch1_normalized = ch1_angle_pulley / PI; // ±180° to ±1.0
```

### Changing CRSF Frame Rate
Modify the update interval in `crsf.cpp`:
```cpp
// For 500Hz (2ms intervals)
if (millis() - _lastChannelUpdate >= 2) {
    sendChannels();
}
```

## Troubleshooting

### No CRSF Signal
1. Check UART wiring between ESP32 and ELRS module
2. Verify baud rate matches (400kbps default)
3. Ensure ELRS module is powered and in correct mode
4. Check serial monitor for "CRSF channels sent" messages

### Sensor Issues
1. Verify I2C wiring and multiplexer connections
2. Check sensor power supply (3.3V)
3. Monitor sensor readings in serial output
4. Ensure magnets are properly positioned on sensors

### ESP-NOW Issues
1. Check that both ESP32 devices are on the same WiFi channel
2. Verify the MAC address in the code matches your display device
3. Ensure both devices have ESP-NOW properly initialized
4. Check for "Display data sent via ESP-NOW successfully" in serial monitor
5. If using multiple ESP-NOW devices, ensure unique MAC addresses

## Serial Monitor Output

The code provides detailed debugging information for both communication methods:
```
CRSF initialized for ELRS TX module
ESP-NOW initialized for display communication
Sensors initialized and zeroed
Starting CRSF channel transmission and ESP-NOW display updates...

Sensor Measurements (degrees)
Ch0 (Shoulder Flex): 23.45 -> CRSF: 1623us
Ch1 (Elbow): -45.67 -> CRSF: 1234us
Ch2 (Shoulder Rot): 0.12 -> CRSF: 1501us
Ch3 (Claw): 15.30 -> CRSF: 1574us
Ch5 (Power): ON -> CRSF: 2000us
CRSF channels sent to ELRS TX module
Display data sent via ESP-NOW successfully
```

### Channel Range Issues
1. Check sensor offset calibration during startup
2. Verify angle-to-channel conversion logic
3. Adjust channel scaling if needed
4. Monitor channel microsecond values in serial output

## Dependencies

- Arduino ESP32 core
- SimpleFOC library (for AS5600 sensor support)
- Standard Arduino libraries (Wire, HardwareSerial)
- ESP-NOW (built into ESP32 core)
- WiFi library (built into ESP32 core)

## License

This project is open source. Please refer to the license file for details.
