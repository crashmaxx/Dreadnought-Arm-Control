# MOSI Configuration Option

## Overview
You can now configure the SPI MOSI pin to be held high all the time instead of sending dynamic SPI commands to the AS504x encoder.

## Configuration

### Option 1: Normal SPI Commands (Default)
```c
#define ENCODER_SPI_MOSI_ALWAYS_HIGH 0  // Default behavior
```
- MOSI sends actual SPI commands (0x7FFD, 0x7FFE, etc.)
- Full diagnostic capabilities available
- Can read magnetic field strength, error flags, etc.
- More complex communication protocol

### Option 2: MOSI Always High
```c
#define ENCODER_SPI_MOSI_ALWAYS_HIGH 1  // Keep MOSI high
```
- MOSI pin is set to logic HIGH (3.3V) and stays there
- Simplified communication - position reading only
- No diagnostic commands sent
- May be more reliable for problematic connections

## How to Change

1. Edit `src/board_config.h`
2. Find your board configuration section (e.g., `#ifdef BOARD_LEFT_SHOULDER`)
3. Change the line:
   ```c
   #define ENCODER_SPI_MOSI_ALWAYS_HIGH 1  // Enable always-high mode
   ```
4. Recompile and upload

## When to Use Always-High Mode

- **Connection Issues**: If you're getting inconsistent SPI communication
- **Debugging**: To isolate whether MOSI commands are causing problems
- **Simple Position Reading**: When you only need angle data, not diagnostics
- **Electrical Noise**: If dynamic MOSI switching is causing interference

## Behavior Differences

### Normal Mode (MOSI_ALWAYS_HIGH = 0):
- Sends specific commands to AS504x
- Gets diagnostic data every 100 samples
- Can detect magnetic field issues
- More robust error detection

### Always-High Mode (MOSI_ALWAYS_HIGH = 1):
- MOSI stays at 3.3V constantly
- Only reads position data
- No diagnostic commands
- Simpler, potentially more stable communication

## Current Setting
Currently set to: **Normal Mode** (ENCODER_SPI_MOSI_ALWAYS_HIGH = 0)

To enable always-high mode, change the setting in your board configuration and recompile.
