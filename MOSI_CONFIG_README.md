# AS504x SPI Mode Configuration

`ENCODER_SPI_MOSI_ALWAYS_HIGH` selects the AS504x transaction flow in the active board block in `src/board_config.h`.

## Direct Angle Mode

```c
#define ENCODER_SPI_MOSI_ALWAYS_HIGH 1
```

This mode sends the AS5047P angle-read command (`0x7FFF`), waits briefly, then reads the response. Despite the historical macro name, the implementation sends this command on MOSI; it does not hold the pin electrically high for the entire transaction. The response must pass the parity check before it is accepted as a position sample.

Use this mode for the direct angle-read path. The currently selected `BOARD_LEFT_CLAW` configuration enables it.

## Diagnostic Mode

```c
#define ENCODER_SPI_MOSI_ALWAYS_HIGH 0
```

This mode uses the diagnostic and magnitude-register transaction flow and refreshes diagnostics periodically. Position samples still require a valid parity check.

## Changing The Mode

1. Select the intended board at the top of `src/board_config.h`.
2. Change `ENCODER_SPI_MOSI_ALWAYS_HIGH` in that board's block.
3. Build and upload the firmware.

```powershell
pio run
pio run --target upload
```
