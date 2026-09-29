# Dreadnought-Arm-Control

![Control Architecture Plan](/images/Armstrong_Control_Architecture.png)

## Channel Mapping

### Left and Right TX
12 channel mixed\
1-4 full rate\
5 on/off\
6-13 half rate


| Channel | Function |
| --- | --- |
| 1 | Shoulder Rotation |
| 2 | Shoulder Joint |
| 3 | Elbow Joint |
| 4 | Claw or Weapon | 
| 5 | Arm/Disarm |
| 6 | Set Zero Positions |
| 7 | Flame Thrower |
| 8 | Bubbles |

9 through 13 are available for future functions

## General overview

Armageddon uses VESC motor controllers as part of a DIY servo closed-loop control system. The VESC controls the motor using Field-Oriented Control (FOC) and holds the position based on the motor encoder. An ESP32 communicates with the VESC over CAN bus. The ESP reads a second encoder after the gearbox, at the joint, and compares it to the target position from the RC receiver. It calculates the motor position needed to reach that joint position and commands the VESC to move the motor the required number of revolutions to get there. This repeats to correct for under or overshoot and continuously follows the target position as it changes from the operator moving the remote inputs.

I am using a fork of the VESC firmware that supports motor position control beyond 360 degrees. The current VESC firmware only allows you to set the motor positon to an angle from 0 deg to 360 deg, this version will let you let the motor position in revolutions, any amount. So I can can command the motor to move, for example, 12.5 revolutions to get the joint after the gearbox to move the intend amount of degrees. You will have to compile the firmware for your particular VESC model.

https://github.com/ElwinBoots/bldc/tree/elwin-dev

The VESC CAN control implementation is in `src/`, with the main control loop in `src/main.c`. The documentation below and in the encoder guide describes the current implementation.

## Joint Position And Limits

For an SPI AS504x joint encoder, the sensor supplies a raw single-turn angle from 0 degrees up to (but not including) 360 degrees. The firmware tracks consecutive verified readings and counts crossings of the 0/360 degree boundary, so the control loop receives a multi-turn joint angle. A reading of 430 degrees is therefore preserved as 430 degrees rather than being treated as 70 degrees.

`MIN_ANGLE` and `MAX_ANGLE` in `src/board_config.h` constrain the target generated from the transmitter; they do not clamp encoder feedback or disable the motor when feedback temporarily exceeds the configured range. This lets the controller correct normal overshoot.

For example, with `MIN_ANGLE = 0`, `MAX_ANGLE = 420`, and `REST_ANGLE = 210`:

1. The transmitter requests a target of 410 degrees.
2. The joint overshoots to 430 degrees. The AS504x raw reading is approximately 70 degrees, while the tracked joint reading remains 430 degrees.
3. The control error is $410 - 430 = -20$ degrees, so the VESC receives a command that moves the joint back toward 410 degrees using the configured gear ratio.

`REST_ANGLE` is used when the operator performs encoder calibration; it is not part of the normal transmitter-to-target mapping. The multi-turn tracker relies on continuous valid samples, so it cannot infer complete turns that happen while the controller is powered off.

## CANBUS

The new CANBUS commands that are important are:

```c
comm_can_set_pos_floatingpoint(CAN_VESC_ID, position_revolutions);
// Commands the motor to move to a position, in revolutions
comm_can_set_pos_floatingpoint_with_vel(CAN_VESC_ID, position_revolutions, max_vel);
// Commands the motor to move to a position, in revolutions, and sets the max velocity for this command, in RPM
comm_can_set_current_pid_pos(CAN_VESC_ID, position_ref, bool store);
// Commands the VESC to set the measured position to equal a reference position, usually zero
// (poorly named, "current" means "present", nothing to do with "current" as in amps)
```
> [!CAUTION]
> This next one is especially important, because the maximum velocity defaults to 1, which is too slow and will mess up smooth movement. It also can't be configured in VESC Tool. Set it to something much higher, at least 1000. I forget if this is based on RPM or ERPM, but I think it's RPM. So the default will slow the motor to 1 RPM, which interrupts the PID control loop. The default should be changed in the firmware, but I have been setting it over CANBUS for now. I set it higher than the max RPM of the motor, so it never activates.
```c
comm_can_set_max_sp_vel(CAN_VESC_ID, 1000.0f);  // Set position movement max velocity
```

The acceleration and deceleration have reasonable defaults. I think these work, but I haven't experimented too much.
```c
comm_can_set_max_sp_accel(CAN_VESC_ID, 500.0f);   // Set position movement max accerleration
comm_can_set_max_sp_decel(CAN_VESC_ID, 1000.0f);  // Set position movement max deceleration
```

These set the position controller's PID. These can be set in the VESC Tool.
```c
comm_can_set_pos_kp(CAN_VESC_ID, 0.5f);    // Sets position Kp value
comm_can_set_pos_ki(CAN_VESC_ID, 0.1f);    // Sets position Ki value
comm_can_set_pos_kd(CAN_VESC_ID, 0.01f);   // Sets position Kd value
comm_can_set_pos_filter(CAN_VESC_ID, 0.2f);   // Sets position PID filter value
```

I think the CANBUS command IDs in the readme are wrong, but I would have to look into it.
