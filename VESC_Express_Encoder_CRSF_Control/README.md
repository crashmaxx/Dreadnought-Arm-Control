# VESC Express Encoder CRSF Control

A comprehensive ESP32-S3 based control system for robotic joints using VESC motor controllers, featuring CRSF receiver integration, SPI magnetic encoders, and ESP-NOW telemetry.

## 🎯 Project Overview

This project implements a complete position control system for robotic joints with the following key features:

- **Precise Position Control**: Closed-loop control using SPI magnetic encoders with gear ratio compensation
- **CRSF Integration**: Full Crossfire (CRSF) receiver support with failsafe and arming systems
- **CAN Bus Communication**: Complete VESC CAN protocol implementation with extended commands
- **ESP-NOW Telemetry**: Real-time wireless telemetry transmission to monitoring systems
- **Multi-Board Support**: Configurable for different robot joints (shoulder, elbow, claw, etc.)
- **Safety Systems**: Comprehensive fault detection, failsafe, and emergency stop functionality

## 🏗️ System Architecture

### Hardware Platform
- **MCU**: ESP32-S3 Super Mini (4MB Flash, 320KB RAM)
- **Framework**: ESP-IDF 5.4.1 with PlatformIO
- **Communication**: CAN bus, UART (CRSF), SPI (encoder), ESP-NOW (telemetry)

### Key Components
1. **Position Control Loop**: Real-time encoder feedback with PID control
2. **CRSF Receiver**: 16-channel Crossfire protocol with arming logic
3. **CAN Interface**: VESC motor controller communication
4. **ESP-NOW Telemetry**: Wireless data transmission to monitoring systems
5. **Safety Systems**: Multi-layer fault detection and emergency stop

## 📋 Features

### ✅ Implemented Features

#### Core Control System
- **Position Control**: Closed-loop position control with encoder feedback
- **Gear Ratio Compensation**: Automatic compensation for mechanical gear ratios
- **Safety Monitoring**: VESC fault detection and emergency stop
- **Failsafe Operation**: Automatic brake/stop on communication loss

#### Communication Protocols
- **CRSF Receiver**: Complete Crossfire protocol implementation
  - 16-channel support with failsafe
  - Connection monitoring and timeout handling
  - CH5 arming switch for safety
- **CAN Bus**: Full VESC CAN protocol support
  - Position commands (floating-point precision)
  - Current control commands
  - Status message parsing
  - Fault monitoring and reporting
- **ESP-NOW Telemetry**: Wireless telemetry system
  - Unicast transmission to specific peer
  - Real-time position, velocity, current data
  - Robust initialization with proper task management

#### Encoder Support
- **SPI Magnetic Encoders**: AS504x series absolute encoders
- **Dual Hybrid Encoders**: PWM + Quadrature encoder combination
- **Quadrature Encoders**: Standard incremental encoder support
- **PWM Magnetic Encoders**: Single-wire absolute encoder support

#### Debug and Monitoring
- **Selective Debug System**: Individual debug flags for each subsystem
- **Stack Health Monitoring**: Task stack usage monitoring
- **Serial Output**: Clean, categorized debug information
- **Real-time Telemetry**: Live system status via ESP-NOW

### 🚧 Extended CAN Commands (Framework Ready)
Extended VESC CAN commands from ElwinBoots/bldc repository:
- Position PID parameter tuning (Kp, Ki, Kd, filter)
- Speed control limits (velocity, acceleration, deceleration)
- Combined current/position control
- Dynamic CAN baud rate updates

*Note: These commands have parsing framework implemented but need integration with motor control configuration.*

## 🛠️ Board Configurations

The system supports multiple robot joint configurations:

### Right Shoulder (Current)
```c
#define BOARD_RIGHT_SHOULDER
- Encoder: SPI Magnetic (AS504x)
- Range: 0° to 180°
- Gear Ratio: 89.6:1
- Control Channel: CH1
- ESP-NOW: Enabled
```

### Other Supported Joints
- **Left/Right Shoulder**: SPI magnetic encoders, 0-180° range
- **Left/Right Upper Arm**: Dual hybrid encoders, full rotation
- **Left/Right Elbow**: Dual hybrid encoders, full rotation  
- **Left/Right Claw**: Dual hybrid encoders, 20-300° range

## 🚀 Getting Started

### Prerequisites
- PlatformIO IDE or CLI
- ESP32-S3 Super Mini development board
- CAN transceiver (MCP2515 or similar)
- SPI magnetic encoder (AS5047, AS5048, etc.)
- CRSF receiver (TBS Crossfire, ELRS, etc.)

### Hardware Connections

#### ESP32-S3 Super Mini Pinout
```
CAN Bus:
- CAN_TX: GPIO4 (configurable)
- CAN_RX: GPIO5 (configurable)

SPI Encoder (AS504x):
- CS:   GPIO4
- MISO: GPIO5  
- MOSI: GPIO6
- CLK:  GPIO7

CRSF Receiver:
- TX: GPIO1 (configurable)
- RX: GPIO2 (configurable)
```

### Software Setup

1. **Clone Repository**
```bash
git clone <repository-url>
cd VESC_Express_Encoder_CRSF_Control
```

2. **Configure Board**
Edit `src/board_config.h` and uncomment your target board:
```c
#define BOARD_RIGHT_SHOULDER  // or your target board
```

3. **Build and Upload**
```bash
pio run -t upload
pio device monitor
```

### Configuration

#### Debug Options (src/main.c)
```c
#define DEBUG_POSITION_CONTROL      0  // Position control debugging
#define DEBUG_ENCODER_DATA          0  // Encoder data debugging  
#define DEBUG_VESC_STATUS           0  // VESC status debugging
#define DEBUG_CRSF_CHANNELS         0  // CRSF channel data
#define DEBUG_CAN_COMMANDS          0  // CAN command transmission
#define DEBUG_ESPNOW_TELEMETRY      1  // ESP-NOW telemetry debugging
```

#### ESP-NOW Peer Configuration (src/board_config.h)
```c
#define PEER_MAC_ADDR {0x18, 0x8B, 0x0E, 0xFF, 0x02, 0xE4}
```

## 📊 System Performance

### Memory Usage
- **RAM**: 12.3% (40,236 / 327,680 bytes)
- **Flash**: 73.9% (775,336 / 1,048,576 bytes)

### Task Architecture
- **Main Task**: System initialization
- **CRSF Control Task**: Position control loop (6KB stack)
- **ESP-NOW Tasks**: Telemetry transmission (8KB stack)

### Control Loop Performance
- **Position Update Rate**: 50Hz (20ms)
- **Encoder Read Rate**: 100Hz (10ms)
- **CAN Status Rate**: 50Hz
- **Telemetry Rate**: 1Hz

## 🔧 Troubleshooting

### Common Issues

#### ESP-NOW Stack Overflow
**Symptoms**: System crashes with "stack overflow in task telemetry_espno"
**Solution**: Fixed in latest version with 8KB stack allocation

#### Flash Size Mismatch
**Symptoms**: Upload failures or boot loops
**Solution**: Use `lolin_s3_mini` board configuration for 4MB flash

#### CAN Communication Issues
**Symptoms**: No VESC responses
**Solution**: Check CAN wiring, baud rate (500kbps), and VESC CAN ID configuration

#### CRSF Connection Problems
**Symptoms**: No channel data received
**Solution**: Verify UART pins, baud rate (420000), and receiver binding

## 📚 API Reference

### Main Control Functions
```c
// Position control
void comm_can_set_pos_floatingpoint(uint8_t controller_id, float position);

// Current control  
void comm_can_set_current(uint8_t controller_id, float current);

// CRSF channel reading
float crsf_channel_to_normalized(int channel);  // Returns -1.0 to +1.0
bool crsf_is_connected(void);
bool crsf_is_armed(void);

// Encoder reading
float encoder_get_angle_deg(void);
float encoder_get_velocity_deg_s(void);
bool encoder_is_valid(void);
```

### Extended CAN Commands (Framework)
```c
// PID tuning
void comm_can_set_pos_kp(uint8_t controller_id, float kp);
void comm_can_set_pos_ki(uint8_t controller_id, float ki);
void comm_can_set_pos_kd(uint8_t controller_id, float kd);

// Speed limits
void comm_can_set_max_sp_vel(uint8_t controller_id, float max_vel);
void comm_can_set_max_sp_accel(uint8_t controller_id, float max_accel);
void comm_can_set_max_sp_decel(uint8_t controller_id, float max_decel);
```

## 🤝 Contributing

This project is part of a larger robotic control system. Contributions are welcome for:

- Additional encoder type support
- Enhanced safety features
- Performance optimizations
- Extended CAN command implementations
- Documentation improvements

## 📄 License

This project incorporates code from the VESC firmware project:
- Copyright 2022 Benjamin Vedder (benjamin@vedder.se)
- Copyright 2023 Rasmus Söderhielm (rasmus.soderhielm@gmail.com)

Licensed under GPL v3 - see individual files for specific license information.

## 🔗 Related Documentation

- [CAN Commands Summary](CAN_COMMANDS_SUMMARY.md) - Extended CAN protocol documentation
- [CRSF Implementation](CRSF_IMPLEMENTATION_SUMMARY.md) - CRSF protocol details
- [Encoder Documentation](ENCODER_README.md) - Encoder system documentation
- [Implementation Status](IMPLEMENTATION_STATUS.md) - Current project status

## 📞 Support

For issues, questions, or contributions, please refer to the project documentation or create an issue in the repository.
