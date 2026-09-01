# VESC Express Encoder CRSF Control

This project provides CRSF (Crossfire) remote control integration with modular encoder support for VESC motor controllers using an ESP32-C3.

## Features

- **CRSF Protocol Support**: Receives 16-channel RC data at 420kbps from ExpressLRS/Crossfire transmitters
- **Modular Encoder System**: Support for multiple encoder types with board configuration switching
- **ESP-NOW Telemetry**: Wireless telemetry transmission for real-time monitoring and data logging
- **VESC CAN Integration**: Sends motor control commands via CAN bus to VESC controllers
- **Safety Features**: Arm/disarm switch, failsafe handling, connection monitoring
- **Board Configuration**: Support for Claw_Control board configurations and parameters

## Supported Encoder Types

The system supports multiple encoder types that can be selected via board configuration:

### 1. PWM Magnetic Encoders
- **Use Case**: Absolute position sensing with magnetic sensors
- **Interface**: Single PWM signal (pulse width encodes angle)
- **Configuration**: Set `ENCODER_TYPE` to `ENCODER_TYPE_PWM` in `board_config.h`
- **Pins**: Configure `ENCODER_PWM_PIN` in board config

### 2. Quadrature Encoders  
- **Use Case**: High-resolution incremental position sensing
- **Interface**: Two-phase quadrature signals (A/B channels)
- **Configuration**: Set `ENCODER_TYPE` to `ENCODER_TYPE_QUADRATURE` in `board_config.h`
- **Pins**: Configure `ENCODER_A_PIN` and `ENCODER_B_PIN` in board config
- **Features**: 4x encoding, overflow handling, configurable PPR

### 3. Dual Hybrid Encoders
- **Use Case**: Combines PWM absolute reference with quadrature precision (like Claw_Control project)
- **Interface**: PWM + Quadrature (3 pins total)
- **Configuration**: Set `ENCODER_TYPE` to `ENCODER_TYPE_DUAL_HYBRID` in `board_config.h`
- **Benefits**: Absolute position reference + high-resolution incremental tracking

### 4. Extensible Interface
- Additional encoder types can be easily added:
  - SPI encoders (magnetic, optical)
  - I2C encoders (magnetic, IMU-based)
  - Hall sensor arrays
  - Custom protocols

## Board Configuration

Encoder type and pin assignments are configured in `src/board_config.h`:

```c
// Select your board type
#define BOARD_TYPE BOARD_VESC_EXPRESS_DEFAULT

// This automatically configures:
// - ENCODER_TYPE (PWM, Quadrature, Dual Hybrid, etc.)
// - GPIO pin assignments
// - Encoder-specific parameters (PPR, pulse width ranges, etc.)
// - Control parameters (PID gains, angle limits)
// - ESP-NOW peer MAC address for telemetry
```

### Predefined Board Configurations

#### VESC Express Boards
1. **BOARD_VESC_EXPRESS_DEFAULT**: PWM encoder on GPIO 4
2. **BOARD_VESC_EXPRESS_QUAD**: Quadrature encoder on GPIO 4/5
3. **BOARD_VESC_EXPRESS_DUAL**: Dual hybrid encoder on GPIO 4/5/6

#### Claw Control Boards (from original project)
4. **BOARD_LEFT_CLAW**: Dual hybrid encoder, Channel 4 control, Kp=1.2
5. **BOARD_RIGHT_CLAW**: Dual hybrid encoder, Channel 4 control, Kp=1.2
6. **BOARD_LEFT_ELBOW**: Dual hybrid encoder, Channel 3 control, Kp=1.0
7. **BOARD_RIGHT_ELBOW**: Dual hybrid encoder, Channel 3 control, Kp=1.0
8. **BOARD_LEFT_UPPER**: Dual hybrid encoder, Channel 2 control, Kp=0.8, ±90° range
9. **BOARD_RIGHT_UPPER**: Dual hybrid encoder, Channel 2 control, Kp=0.8, ±90° range
10. **BOARD_CUSTOM**: Define your own configuration

### Adding Custom Board Configurations

```c
#elif BOARD_TYPE == BOARD_YOUR_CUSTOM_BOARD
    #define BOARD_NAME "Your Custom Board"
    #define ENCODER_TYPE ENCODER_TYPE_QUADRATURE
    #define ENCODER_A_PIN 10
    #define ENCODER_B_PIN 11
    #define ENCODER_PPR 1024
    
    // Control parameters
    #define CONTROL_CHANNEL 3
    #define DEFAULT_KP 1.0f
    #define DEFAULT_KI 0.02f
    #define DEFAULT_KD 0.10f
    #define MIN_ANGLE 0.0f
    #define MAX_ANGLE 359.0f
    
    // ESP-NOW peer MAC address
    #define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
    // Add any other board-specific settings
#endif
```

## ESP-NOW Telemetry

The system includes wireless telemetry using ESP-NOW protocol for real-time monitoring:

### Telemetry Features
- **Real-time Data Transmission**: Encoder position, velocity, system status
- **Low Latency**: ESP-NOW provides faster transmission than WiFi
- **Multiple Peers**: Support for multiple receiving devices
- **Configurable Data Types**: Encoder, control, status, CRSF channel data

### Telemetry Configuration
Configure telemetry in `src/esp_now_telemetry_config.h`:

```c
#define ESP_NOW_TELEMETRY_ENABLE    1
#define ESP_NOW_WIFI_CHANNEL        1
#define ESP_NOW_TELEMETRY_RATE_MS   100  // Send every 100ms
```

### Predefined Telemetry Types
- **ENCODER**: Position (degrees), velocity (deg/s), validity status
- **STATUS**: Armed state, CRSF connection, error count
- **CONTROL**: Setpoint, PID output, position error
- **CRSF**: Channel values from remote control

### Usage Example
```c
// Send encoder data
TELEMETRY_SEND_ENCODER(encoder_get_angle_deg(), 
                      encoder_get_velocity_deg_s(), 
                      encoder_is_valid());

// Send system status
TELEMETRY_SEND_STATUS(crsf_is_armed(), crsf_is_connected(), error_count);
```

## CRSF Configuration

CRSF settings are configured in `src/crsf_config.h`:

- **Channel 5**: Arm/disarm switch (confirmed)
- **Throttle Channel**: Configurable for current control
- **Baudrate**: 420000 bps (standard for ExpressLRS)
- **GPIO Pins**: TX/RX pins for UART communication

## Usage Example

The main control loop integrates both CRSF input and encoder feedback:

```c
// Update both systems
crsf_update();
encoder_update();

// Check encoder status
if (encoder_is_valid()) {
    float position = encoder_get_angle_deg();
    float velocity = encoder_get_velocity_deg_s();
    
    // Use encoder data for closed-loop control
    // Example: Position control based on RC input
    float target_position = crsf_channel_to_normalized(channel) * 360.0f;
    float error = target_position - position;
    // Apply PID control...
}

// Check CRSF status and send motor commands
if (crsf_is_connected() && crsf_is_armed()) {
    float current = calculate_motor_current(); // Your control logic
    comm_can_set_current(VESC_CONTROLLER_ID, current);
}
```

## Building and Flashing

This project uses PlatformIO with ESP-IDF framework:

```bash
# Build the project
pio run

# Flash to ESP32-C3
pio run --target upload

# Monitor serial output
pio device monitor
```

## Hardware Setup

1. **ESP32-C3 Development Board**
2. **CRSF/ExpressLRS Receiver**: Connect to configured UART pins
3. **Encoder**: Connect based on selected encoder type and board configuration
4. **CAN Transceiver**: For VESC communication (if using CAN)
5. **Power Supply**: 3.3V for ESP32, appropriate voltage for peripherals

## Debugging

The system provides comprehensive logging:

- **CRSF Status**: Connection state, channel values, failsafe status
- **Encoder Status**: Position, velocity, error counts, validity
- **Control Output**: Motor commands, safety state

Enable/disable logging levels in the respective config files.

## Safety Features

- **Arm/Disarm Switch**: Channel 5 controls system arming
- **Connection Monitoring**: Detects CRSF link loss
- **Failsafe Behavior**: Configurable response to signal loss
- **Encoder Validation**: Monitors encoder health and error counts

## Extending the System

### Adding New Encoder Types

1. Create new encoder implementation file (e.g., `encoder_spi.c`)
2. Implement the `encoder_interface_t` structure
3. Add new encoder type to `encoder_interface.h`
4. Update board configuration options
5. Update the encoder selection logic in `encoder_interface.c`

### Adding Control Algorithms

The modular design makes it easy to add:
- PID position control
- Velocity control
- Advanced control algorithms (LQR, MPC, etc.)
- Multi-axis control
- Custom safety logic

## Based On

This project extends the VESC Express firmware and incorporates encoder functionality from the Claw_Control_Rev_Encoder_PPM_AutoTune project, making it modular and configurable for different hardware setups.
