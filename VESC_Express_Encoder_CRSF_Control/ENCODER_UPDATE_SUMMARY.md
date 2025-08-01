# Encoder System Updates Summary

## Overview
Updated the VESC Express Encoder CRSF Control firmware to support multiple encoder types based on board configuration, with conditional compilation for different encoder implementations.

## Changes Made

### 1. Updated `main.c` - Encoder Initialization Section

**Before:** 
- Fixed initialization code that only supported DUAL_HYBRID encoders
- Hard-coded logging messages and pin configurations

**After:**
- Conditional compilation based on `ENCODER_TYPE` from `board_config.h`
- Support for multiple encoder types:
  - `ENCODER_TYPE_DUAL_HYBRID` - PWM + Quadrature encoders (original implementation)
  - `ENCODER_TYPE_SPI_MAGNETIC` - SPI magnetic encoders (AS504x series)
  - `ENCODER_TYPE_PWM_MAGNETIC` - PWM-only magnetic encoders
  - `ENCODER_TYPE_QUADRATURE` - Quadrature-only encoders
  - `ENCODER_TYPE_NONE` - No encoder (current control only)

**Key Features:**
- Encoder-specific initialization messages and pin reporting
- Appropriate stabilization delays (200ms for SPI vs 100ms for others)
- Detailed error reporting with encoder-specific pin information
- Graceful handling of unknown encoder types

### 2. Created `encoder_spi.c` - New SPI Encoder Implementation

**New File:** `src/drivers/encoder_spi.c`

**Features:**
- Complete SPI magnetic encoder interface implementation
- Built on top of existing AS504x driver (`enc_as504x.c`)
- Encoder interface compliance with standard API:
  - `init()`, `update()`, `get_angle_deg()`, `get_angle_rad()`
  - `get_velocity_deg_s()`, `get_velocity_rad_s()`, `is_valid()`
  - `get_error_count()`, `reset_errors()`, `get_type_name()`

**SPI Configuration:**
- Uses board-defined SPI pins: `ENCODER_SPI_CS_PIN`, `ENCODER_SPI_MISO_PIN`, etc.
- Configures software SPI bit-banging interface
- Handles AS504x encoder diagnostics and error detection

**Data Processing:**
- Angle normalization (0-360 degrees)
- Velocity calculation with wraparound handling
- Connection status monitoring
- Error counting and rate limiting

### 3. Updated `encoder_interface.c` - Added SPI Encoder Include

**Enhancement:**
- Added conditional include comment for SPI encoder implementation
- Ensures proper linking of SPI encoder interface when `ENCODER_TYPE_SPI_MAGNETIC` is selected

## Board Configuration Support

The system now fully supports the existing board configurations:

### SPI Magnetic Encoder Boards:
- `BOARD_LEFT_SHOULDER` 
- `BOARD_RIGHT_SHOULDER` (currently selected)

**SPI Pin Configuration Example:**
```c
#define ENCODER_SPI_CS_PIN 7
#define ENCODER_SPI_MISO_PIN 8  
#define ENCODER_SPI_MOSI_PIN 9
#define ENCODER_SPI_CLK_PIN 10
```

### Dual Hybrid Encoder Boards:
- `BOARD_LEFT_CLAW`, `BOARD_RIGHT_CLAW`
- `BOARD_LEFT_ELBOW`, `BOARD_RIGHT_ELBOW`  
- `BOARD_LEFT_UPPER`, `BOARD_RIGHT_UPPER`
- `BOARD_CUSTOM`

**Dual Hybrid Pin Configuration Example:**
```c
#define ENCODER_PWM_PIN 5
#define ENCODER_A_PIN 7
#define ENCODER_B_PIN 6
#define ENCODER_PPR 4096
```

## Runtime Behavior

### Initialization Messages

**For SPI Magnetic Encoders:**
```
Encoder config - Type: SPI_MAGNETIC (AS504x series)
SPI pins - CS: GPIO7, MISO: GPIO8, MOSI: GPIO9, CLK: GPIO10
SPI magnetic encoder system initialized successfully
SPI Encoder init: Valid=YES, Initial angle=123.45°
```

**For Dual Hybrid Encoders:**
```
Encoder config - Type: DUAL_HYBRID, PPR: 4096
Encoder pins - PWM: GPIO5, A: GPIO7, B: GPIO6  
PWM range: 0-1020 us
Dual hybrid encoder system initialized successfully
Encoder init: Valid=YES, Initial angle=123.45°
```

### Error Handling

**SPI Encoder Errors:**
- Connection monitoring via AS504x diagnostics
- SPI communication error detection
- Magnetic field strength validation (COMP_HIGH/COMP_LOW)
- Detailed pin configuration error messages

**General Encoder Errors:**
- Type-specific pin validation
- Hardware connection verification
- Graceful system shutdown on initialization failure

## Compilation Verification

✅ **SPI Magnetic Configuration:** Compiles successfully with `BOARD_RIGHT_SHOULDER`
✅ **Dual Hybrid Configuration:** Compiles successfully with `BOARD_CUSTOM`  
✅ **Cross-compatibility:** All encoder implementations link properly

## Future Expandability

The conditional compilation structure makes it easy to add new encoder types:
1. Add new `ENCODER_TYPE_*` definition in `board_config.h`
2. Create new encoder implementation file (e.g., `encoder_i2c.c`)
3. Add new case in `main.c` initialization section
4. Update `encoder_interface.h` with extern declaration

## Files Modified

- `src/main.c` - Updated encoder initialization with conditional compilation
- `src/drivers/encoder_spi.c` - **NEW** - SPI magnetic encoder implementation  
- `src/drivers/encoder_interface.c` - Added SPI encoder include reference

## Testing Status

- ✅ Compilation test with SPI magnetic encoder configuration
- ✅ Compilation test with dual hybrid encoder configuration
- ✅ Code review for proper conditional compilation
- ✅ Verification of encoder interface compliance

The system is now ready to support different encoder types based on board configuration, with proper initialization, error handling, and runtime behavior for each encoder type.
