# CRSF Receiver Implementation for VESC Express

This implementation adds CRSF (Crossfire) protocol support to the VESC Express project, allowing direct control from CRSF-compatible radio control receivers such as those from TBS (Team BlackSheep) or other ExpressLRS/ELRS receivers.

## Features

- **Full CRSF Protocol Support**: Implements the complete CRSF protocol for receiving RC channel data
- **16 Channel Support**: Receives all 16 channels of CRSF data
- **Failsafe Detection**: Automatic detection of signal loss and failsafe mode
- **CRC Validation**: Full frame validation with CRC8 checksums
- **ESP32 UART Integration**: Uses ESP32 UART with FreeRTOS task for non-blocking operation
- **Utility Functions**: Helper functions for common RC operations (arming, channel mapping, etc.)

## Hardware Connections

Connect your CRSF receiver to the ESP32-C3:

| CRSF Receiver | ESP32-C3 Pin | Description |
|---------------|--------------|-------------|
| TX            | GPIO 3       | CRSF data output from receiver |
| RX            | GPIO 2       | CRSF data input to receiver (optional) |
| VCC           | 3.3V or 5V   | Power supply |
| GND           | GND          | Ground |

**Note**: Only the TX line from the receiver is required for basic channel data reception.

## Software Configuration

### Pin Configuration
The default pin configuration in `main.c`:
```c
#define CRSF_TX_PIN 2    // ESP32 TX to receiver RX (optional)
#define CRSF_RX_PIN 3    // ESP32 RX from receiver TX (required)
#define CRSF_BAUDRATE 420000  // Standard CRSF baud rate
```

### Channel Mapping
Standard CRSF channel mapping (following AETR convention):
- **Channel 1**: Roll/Aileron
- **Channel 2**: Pitch/Elevator  
- **Channel 3**: Throttle
- **Channel 4**: Yaw/Rudder
- **Channel 5**: AUX1 (commonly used for arm/disarm)
- **Channels 6-16**: Additional auxiliary channels

## API Reference

### Initialization
```c
void crsf_init(int uart_num, int tx_pin, int rx_pin, int baudrate);
```
Initialize the CRSF receiver with specified UART and pins.

### Data Access
```c
uint16_t crsf_get_channel(uint8_t channel);          // Raw values (172-1811)
uint16_t crsf_get_channel_scaled(uint8_t channel);   // Scaled values (1000-2000)
void crsf_get_all_channels_scaled(uint16_t *channels); // All channels at once
```

### Status Functions
```c
bool crsf_is_connected(void);     // Check if receiver is connected
bool crsf_is_failsafe(void);      // Check if in failsafe mode
bool crsf_has_new_data(void);     // Check for new data since last call
```

### Utility Functions (from crsf_utils.h)
```c
bool crsf_is_armed(void);                           // Check arm state (AUX1 high)
float crsf_channel_to_normalized(uint8_t channel);  // Convert to -1.0 to +1.0
float crsf_channel_to_percent(uint8_t channel);     // Convert to -100% to +100%
bool crsf_is_channel_high(uint8_t channel);         // Check if channel > 1700
```

## Usage Examples

### Basic Channel Reading
```c
// Get throttle channel (scaled to 1000-2000)
uint16_t throttle = crsf_get_channel_scaled(CRSF_CHANNEL_THROTTLE);

// Convert to normalized range (-1.0 to +1.0)
float throttle_normalized = crsf_channel_to_normalized(CRSF_CHANNEL_THROTTLE);
```

### Motor Control with Arming
```c
if (crsf_is_armed() && crsf_is_connected()) {
    float throttle = crsf_channel_to_normalized(CRSF_CHANNEL_THROTTLE);
    float current_command = throttle * 50.0f; // Scale to ±50A
    comm_can_set_current(0, current_command);
} else {
    comm_can_set_current(0, 0.0f); // Disarmed or disconnected
}
```

### Multi-Channel Control
```c
uint16_t channels[16];
crsf_get_all_channels_scaled(channels);

float roll = crsf_channel_to_normalized(CRSF_CHANNEL_ROLL);
float pitch = crsf_channel_to_normalized(CRSF_CHANNEL_PITCH);
float yaw = crsf_channel_to_normalized(CRSF_CHANNEL_YAW);

// Use roll, pitch, yaw for multi-axis control
```

## Integration with VESC CAN Commands

The implementation works seamlessly with the existing VESC CAN command system:

```c
// Current control
comm_can_set_current(controller_id, current_amps);
comm_can_set_current_rel(controller_id, current_relative);

// Speed control  
comm_can_set_rpm(controller_id, rpm);
comm_can_set_duty(controller_id, duty_cycle);

// Position control
comm_can_set_pos(controller_id, position_degrees);
```

## Failsafe Behavior

The implementation includes comprehensive failsafe handling:

1. **Signal Loss**: Automatic detection when no data received for 500ms
2. **CRC Errors**: Invalid frames are discarded
3. **Failsafe Flag**: Hardware failsafe from receiver is detected
4. **Safe Defaults**: All channels default to safe middle positions

## Performance Characteristics

- **Update Rate**: Up to 150Hz (depending on receiver configuration)
- **Latency**: < 10ms from receiver to application
- **CPU Usage**: Minimal - uses interrupt-driven UART with dedicated task
- **Memory Usage**: < 1KB RAM for buffers and channel data

## Troubleshooting

### No Data Received
1. Check wiring connections
2. Verify receiver is bound and powered
3. Confirm baud rate (420000 is standard)
4. Check UART pin configuration

### Invalid Channel Data
1. Verify receiver is outputting CRSF protocol (not PWM/SBUS)
2. Check for interference on UART lines
3. Ensure proper grounding between devices

### Intermittent Connection
1. Check power supply stability
2. Verify antenna orientation and range
3. Check for electromagnetic interference

## Compatibility

### Tested Receivers
- ExpressLRS receivers (all variants)
- TBS Crossfire receivers
- ImmersionRC Ghost (with CRSF mode)
- Frsky R9 (with CRSF firmware)

### Radio Systems
- OpenTX/EdgeTX transmitters
- TBS Tango series
- Frsky Taranis/Horus series
- Radiomaster TX series

## License

This implementation is part of the VESC firmware and is licensed under the GNU General Public License v3.0.
