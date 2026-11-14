# SpeedyBee F405 V4 AHRS demo

This example uses the onboard ICM-42688-P with DMA-backed SPI and a data-ready
interrupt. Sampling follows the BSP IMU example: latched INT1, level waits with
a 10 ms polling fallback, and FIFO draining in passes of at most 32 samples.
Bus errors and overflows reset the FIFO before sampling resumes. Keep the board still at startup while the demo collects a 1.25-second
window for gyro bias calibration. The Mahony filter then runs at 1 kHz and logs
acceleration, angular velocity, and orientation over USB serial at 2 Hz.
Telemetry uses forward-right-down body axes, as in ArduPilot and PX4. Positive
pitch raises the nose, positive roll lowers the right side, and positive yaw
turns the nose right. Yaw is relative and can drift because the demo uses only
the accelerometer and gyroscope.

The `speedybee-f405-v4-bsp` dependency is fetched from crates.io.
From this directory, install the ARM target and build the firmware.

```sh
rustup target add thumbv7em-none-eabihf
just build
```

To flash over USB, hold the board's BOOT button while plugging it in, then run
`just dfu` with `arm-none-eabi-objcopy` and `dfu-util` installed. This writes
the demo to the board's application flash. After the board restarts, open its
USB CDC serial port in a terminal, for example
`screen /dev/tty.usbmodemXXXX 115200` on macOS or
`screen /dev/ttyACM0 115200` on Linux.
