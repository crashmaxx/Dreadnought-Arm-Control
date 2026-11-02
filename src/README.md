# Source Layout

This folder contains the ESP-IDF firmware built by the `lolin_s3_mini` PlatformIO environment.

## Configuration

`board_config.h` selects exactly one joint board and supplies its encoder type and pins, CAN IDs, CRSF pins, target range, motion limits, gear ratio, and optional FRAM configuration. Do not copy pin values from this document; use the selected board block.

`debug_config.h` controls optional runtime logging.

## Control Path

1. `comm/crsf_receiver.c` receives CRSF channels.
2. `main.c` maps the selected control channel to a target in `MIN_ANGLE..MAX_ANGLE`.
3. `drivers/encoder_interface.c` selects and updates the configured joint encoder.
4. `main.c` compares calibrated joint feedback with the target, converts the joint error through `GEAR_RATIO`, and sends a floating-point VESC position command through `comm/comm_can.c`.

For SPI magnetic encoders, `drivers/enc_as504x.c` reads and validates single-turn AS504x samples, while `drivers/encoder_spi.c` tracks 0/360 degree crossings to report a multi-turn joint position. The target range does not clamp feedback: temporary overshoot remains visible to the control loop so it can correct back to the target.

## Safety Behavior

Normal position commands require a connected and armed CRSF receiver, fresh VESC position data, and valid encoder feedback or a fresh VESC position fallback. Disarm or CRSF connection loss sends zero current. Invalid AS504x samples are rejected and do not advance the multi-turn position.

### Elbow Four-Bar RC Interlock

The elbow configurations include an optional RC-only baseline for protecting a four-bar linkage. When `LINKAGE_RC_INTERLOCK_ENABLE` is `1`, the elbow target is constrained so that:

```text
LINKAGE_MIN_RELATIVE_ANGLE <= elbow_target - upper_arm_target <= LINKAGE_MAX_RELATIVE_ANGLE
```

`upper_arm_target` is calculated from the shared RC upper-arm channel and its configured target range. The interlock is disabled by default because the safe relative limits must be measured on the physical linkage. Configure the `LINKAGE_*` values in the applicable elbow block in `board_config.h`, validate the limits at low torque, then enable it. This is a command-based safeguard only; it does not replace a future measured-angle interlock between the upper-arm and elbow boards.

## Build

```powershell
pio run
pio run --target upload
pio device monitor
```

See the repository [encoder guide](../ENCODER_README.md), [CRSF guide](../CRSF_README.md), and [AS5047P note](../AS5047P_FIXES_SUMMARY.md) for implementation details.
