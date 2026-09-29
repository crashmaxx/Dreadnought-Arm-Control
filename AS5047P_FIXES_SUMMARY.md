# AS5047P Encoder Issues and Fixes

## Issues Found and Fixed

### 🔧 **Critical Bug #1: Wrong Bit Shift in 16-bit SPI Transfer**
**Problem:** The 16-bit SPI transfer function was using `send >> 7` instead of `send >> 15`
- This was sending bit 7 instead of bit 15 (MSB) for each bit
- Completely corrupted the MOSI data transmission
- Would cause communication failure with any SPI encoder

**Fix Applied:**
```c
// OLD (WRONG):
WRITE_PIN(s->mosi_pin, send >> 7);

// NEW (FIXED):
WRITE_PIN(s->mosi_pin, send >> 15);  // Use bit 15 for 16-bit data
```

### 🔧 **Critical Issue #2: AS5047P vs AS504x Register Differences**
**Problem:** Code was using AS5040/AS5041 addresses, missing AS5047P angle register
- AS5047P has a specific angle register at `0x3FFF`
- Previous code was not using the correct register for direct angle reading

**Fix Applied:**
```c
// Added AS5047P angle register:
#define AS504x_SPI_ANGLE_ADR     0x3FFF  // AS5047P angle register
#define AS504x_SPI_READ_ANGLE_MSG (AS504x_SPI_ANGLE_ADR | AS504x_SPI_READ_BIT)

// Updated MOSI always-high mode to use proper AS5047P protocol:
uint16_t angle_cmd = AS504x_SPI_READ_ANGLE_MSG;  // 0x7FFF for AS5047P
spi_bb_transfer_16(&(cfg->sw_spi), 0, &angle_cmd, 1);  // Send command
// ... delay ...
spi_bb_transfer_16(&(cfg->sw_spi), &pos, 0, 1);        // Read response
```

### 🔧 **Issue #3: Improved Debug Logging**
**Problem:** Limited visibility into what's happening with encoder communication

**Fix Applied:**
- Added debug logging for invalid SPI responses
- Added debug logging for valid SPI responses  
- Added debug logging for successful angle calculations
- Added raw data logging to track SPI communication

## AS5047P Communication Protocol

### **With MOSI (Normal Mode):**
1. Send diagnostic commands (0x7FFD, 0x7FFE)
2. Read responses with full error checking
3. Perform parity verification
4. Get angle from diagnostic data

### **With MOSI Always High Mode:**
1. Send angle read command (0x7FFF)
2. Wait for response preparation
3. Read angle response directly
4. Require valid response parity before accepting the angle

## Current Position Handling

The AS5047P provides a single-turn reading from 0 degrees up to (but not including) 360 degrees. `encoder_spi.c` extends it to a multi-turn joint angle by counting verified crossings of the 0/360 degree boundary. A parity failure does not change the tracked angle or turn count.

`MIN_ANGLE` and `MAX_ANGLE` limit the requested transmitter target, not measured feedback. Feedback may temporarily exceed the target range after an overshoot so the controller can command a correction back to the target.

## Expected Behavior After Fixes

### **Debug Output to Look For:**
```
[AS504x] Valid SPI response: 0x????
[AS504x] Angle calculated: raw=0x????, angle=???.??°
[SPI_ENCODER] Success on attempt 1! Initial angle: ????? degrees
```

### **Error Conditions to Watch:**
```
[AS504x] Invalid SPI response: 0x0000 or 0xFFFF
[AS504x] Parity check failed
[SPI_ENCODER] All initial read attempts failed
```

## Configuration Status

Encoder pins and `ENCODER_SPI_MOSI_ALWAYS_HIGH` are defined by the selected board block in `src/board_config.h`. Check that file rather than relying on fixed pin values in this historical note.

## Next Steps
1. **Upload the fixed firmware**
2. **Monitor serial output** for debug messages
3. **Check for:** "Valid SPI response" and "Angle calculated" messages
4. **If still failing:** Try setting `ENCODER_SPI_MOSI_ALWAYS_HIGH = 0` for full diagnostic mode

## Hardware Checklist
- ✅ AS5047P power supply (3.3V or 5V depending on variant)
- ✅ SPI wiring: CS, CLK, MISO, MOSI properly connected
- ✅ Ground connection between ESP32 and AS5047P
- ✅ Magnet positioned correctly over AS5047P (centered, proper distance)
- ✅ No loose connections or short circuits

The main issue was likely the wrong bit shift causing completely garbled SPI communication!
