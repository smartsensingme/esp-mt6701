# Dual MT6701 I2C hardware test

Standalone ESP-IDF 6.0 / ESP32-C3 test, moved from the robot application. It has
no dependency on esp_2wd_robot, LUT, PID or motor bridge. It never initializes or
commands motors. Keep the motor supply disconnected when identifying sensors.

Defaults match the current bench: SDA8, SCL9, 400 kHz, LEFT address 0x06 and
RIGHT address 0x46. Change pins/addresses with `menuconfig` under **Dual MT6701
hardware test** for a different assembly. Do not connect two equal addresses to
one bus; duplicate addresses/pins are rejected. Pin numbers must also be valid
and available on the actual board. LEFT/RIGHT labels describe the configured
assignment, not an automatic inference of physical mounting. Rotate each shaft
manually and compare its labeled raw reading/speed after the run.

From this directory, with the ESP-IDF environment loaded:

```sh
idf.py -D IDF_TARGET=esp32c3 build
idf.py menuconfig
idf.py -p /dev/cu.usbmodem84201 flash monitor
```

The project deliberately disables Component Manager for this local test and
uses this driver plus sibling `esp_rt_diagnostics` through explicit paths. It
requires those local directories, independently of the robot application's
main, board configuration or build. This is not a published standalone package
with remotely resolved diagnostics dependencies. Normal driver consumers do not
need the test or diagnostics component.

After initialization and a one-second output-drain pause, GPTimer releases a
priority-8 owner every 5000 us (200 Hz) for 60 seconds. Reads are sequential with
1-ms timeouts. No application formatting/logging/USB transfer occurs during the
periodic phase. Failed reads are counted; velocity history resets and becomes
valid only after two fresh samples. Raw velocity has no LUT/sign/Kalman correction.
Gaps above 10 ms and exact half-turn differences invalidate speed. Delivered and
inferred missed events are counted; backlog produces one pair of reads, not
catch-up loops. Watchdogs remain enabled.

The timer stops and resources are released before the final report. Per-side
attempts/errors, consecutive failure/recovery counters, raw/speed validity and
I2C/interval timing are printed. The measured 5-ms cycle includes both reads
and numeric accounting, with 1-ms stage budgets per sensor; remaining time is
wake/bookkeeping headroom. Wake latency is measured from ISR capture and does
not prove hardware-deadline latency. One observed maximum is not a worst-case
bound. Reboot to repeat; the test runs once automatically after boot.

`test_encoder_recovery.c` is the native recovery-accounting regression, run by
`sh test_native.sh` (also called by the parent workspace's `tools/test.sh`). Hardware behavior of this extracted
project remains to be checked on the bench; building does not validate wiring.
