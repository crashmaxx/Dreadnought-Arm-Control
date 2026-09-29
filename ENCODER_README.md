# Encoder And Joint Position Control

The ESP32 reads joint feedback, maps the active CRSF channel to a joint-angle target, and sends a position target in motor revolutions to the VESC over CAN. The VESC performs the motor-side position control; the ESP32 repeatedly corrects the VESC target from joint feedback.

## Configuration

Select exactly one board at the top of `src/board_config.h`. Each board block defines its encoder type and pins, CAN IDs, CRSF pins, control channel, joint range, motion limits, gear ratio, and optional FRAM settings.

The encoder types currently selected by board configuration are:

- `ENCODER_TYPE_SPI_MAGNETIC` for AS504x SPI encoders
- `ENCODER_TYPE_PWM_MAGNETIC` for PWM magnetic encoders
- `ENCODER_TYPE_QUADRATURE` for incremental encoders
- `ENCODER_TYPE_DUAL_HYBRID` for PWM plus quadrature feedback
- `ENCODER_TYPE_VESC_INTERNAL` for VESC position feedback

## SPI AS504x Position Tracking

An AS504x reports one absolute turn, from 0 degrees up to (but not including) 360 degrees. The SPI encoder wrapper extends this to a multi-turn angle by counting boundary crossings:

- A change from near 360 degrees to near 0 degrees increments the turn count.
- A change from near 0 degrees to near 360 degrees decrements the turn count.
- The reported joint angle is the raw angle plus `turn_count * 360` degrees.

Every sample must pass the AS504x parity check before it can change the raw angle, turn count, velocity, or position. A failed sample makes the encoder update fail so the control loop can use its existing VESC-feedback fallback only when that fallback is fresh. The hardware reading of exactly 0 degrees remains a legitimate angle.

The boundary detector assumes adjacent valid samples differ by less than 270 degrees. It tracks turns while powered, but it cannot know how many complete turns occurred while the controller was unpowered.

## Targets, Calibration, And Overshoot

The active CRSF control channel is normalized to $[-1, 1]$, then mapped linearly to the configured target range:

$$
target = MIN\_ANGLE + \frac{input + 1}{2}(MAX\_ANGLE - MIN\_ANGLE)
$$

`MIN_ANGLE` and `MAX_ANGLE` bound this target only. Encoder feedback is intentionally not clamped to the target range, because feedback outside that range can be a normal transient after an overshoot. `REST_ANGLE` is used only when the operator calibrates the encoder, setting the current physical joint position to that reference.

Example with `MIN_ANGLE = 0`, `MAX_ANGLE = 420`, and `REST_ANGLE = 210`:

1. The transmitter requests 410 degrees.
2. The joint moves past the target to 430 degrees.
3. The AS504x raw value is approximately 70 degrees, but multi-turn tracking reports 430 degrees.
4. The control error is $410 - 430 = -20$ degrees, so the next VESC position command corrects 20 degrees back toward 410 degrees.

## Control And Safety Behavior

Normal closed-loop commands require an armed and connected CRSF receiver, fresh VESC position data, and either a valid encoder update or a fresh VESC tracking fallback. A CRSF disconnect or disarm sends zero current. Invalid encoder samples are rejected; they are never used to advance multi-turn position.

The control task runs at the rate configured by `CRSF_CONTROL_UPDATE_RATE_HZ` in `src/comm/crsf_config.h`, currently 100 Hz. Arm/disarm uses CRSF AUX1 (physical channel 5) by default.

## Build And Flash

This repository uses PlatformIO with ESP-IDF. The configured environment is `lolin_s3_mini`.

```powershell
pio run
pio run --target upload
pio device monitor
```
