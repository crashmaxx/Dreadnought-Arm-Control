# Multi-Board Configuration System

This project supports multiple board configurations and persistent configuration management.

## Quick Start

### 1. Select Board Configuration

Edit `include/board_config.h` and uncomment ONE of the following lines:

```cpp
// #define BOARD_LEFT_CLAW      // Left claw servo
// #define BOARD_RIGHT_CLAW     // Right claw servo  
// #define BOARD_LEFT_ELBOW     // Left elbow servo
// #define BOARD_RIGHT_ELBOW    // Right elbow servo
// #define BOARD_LEFT_UPPER     // Left upper arm servo
// #define BOARD_RIGHT_UPPER    // Right upper arm servo
#define BOARD_CUSTOM          // Custom configuration
```

### 2. Upload Firmware

Compile and upload to your ESP32 board.

### 3. Configuration Management

#### Channel Controls (via RC transmitter):
- **Channel 5**: Arm/Disarm system (>1700µs)
- **Channel 6**: Start/Stop AutoTune (>1700µs)
- **Channel 7**: Save config to EEPROM (>1700µs)
- **Channel 8**: Load config from EEPROM (>1700µs)

#### Serial Commands (via Serial Monitor):
- `export` - Export current configuration as JSON
- `import` - Import configuration from JSON
- `status` - Show current configuration

## Configuration Workflow

### For New Boards:

1. **Select Configuration**: Edit `board_config.h` for your board type
2. **Upload Firmware**: Flash to ESP32
3. **Run AutoTune**: Use Channel 6 to auto-tune PID parameters
4. **Export Config**: Type `export` in Serial Monitor
5. **Save JSON**: Copy the JSON output to `configs/your_board_config.json`

### For Existing Boards:

1. **Import Config**: 
   - Type `import` in Serial Monitor
   - Paste your saved JSON configuration
   - Press Enter
2. **Or Load from EEPROM**: Use Channel 8 if config is already saved

## Pre-configured Boards

Example configurations are provided in the `configs/` folder:

- `left_claw_config.json` - Left claw servo (Channel 4)
- `right_claw_config.json` - Right claw servo (Channel 3)
- `left_elbow_config.json` - Left elbow servo (Channel 5)
- `right_elbow_config.json` - Right elbow servo (Channel 6)
- `left_upper_config.json` - Left upper arm servo (Channel 7)
- `right_upper_config.json` - Right upper arm servo (Channel 8)

## Board Configuration Details

### Left/Right Claw
- **Control Channel**: 4 (Left), 3 (Right)
- **Angle Range**: 0-359 degrees
- **Default PID**: Kp=1.2, Ki=0.05, Kd=0.15

### Left/Right Elbow
- **Control Channel**: 5 (Left), 6 (Right)
- **Angle Range**: 0-180 degrees
- **Default PID**: Kp=1.0, Ki=0.03, Kd=0.12

### Left/Right Upper Arm
- **Control Channel**: 7 (Left), 8 (Right)
- **Angle Range**: -90 to +90 degrees
- **Default PID**: Kp=0.8, Ki=0.02, Kd=0.10

## AutoTune Process

1. Set your RC transmitter Channel 6 high (>1700µs)
2. The system will automatically oscillate around the setpoint
3. Monitor Serial output for progress
4. AutoTune completes automatically (or times out after 5 minutes)
5. New PID parameters are automatically saved to EEPROM

## JSON Configuration Format

```json
{
  "board_name": "Board_Name",
  "firmware_version": "AutoTune_v1.0",
  "config_version": 2,
  "timestamp": 12345678,
  "pid": {
    "Kp": 1.200,
    "Ki": 0.050,
    "Kd": 0.150
  },
  "settings": {
    "channel_number": 4,
    "min_angle": 0,
    "max_angle": 359
  },
  "hardware": {
    "encoder_ppr": 2048,
    "pwm_min": 1,
    "pwm_max": 1060
  }
}
```

## Troubleshooting

- **No board selected error**: Uncomment a board configuration in `board_config.h`
- **Config not loading**: Check EEPROM with `status` command, re-import if needed
- **AutoTune not working**: Ensure system is armed (Channel 5 > 1700µs)
- **Import fails**: Verify JSON format and try again

## Adding Custom Boards

1. Edit `include/board_config.h`
2. Add a new `#ifdef BOARD_YOUR_NAME` section
3. Define your board's parameters
4. Update the board selection at the top of the file
