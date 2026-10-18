# Change log

## 2026-10-18 — Standalone dual-I2C hardware diagnostic (unreleased)

Changes since eb9e0ac (bounded I2C initialization API already committed):

- Move passive two-sensor identification/acquisition into
  `test_apps/dual_i2c`, outside the robot application. Driver implementation and
  public MT6701 API are unchanged by this revision.
- Provide an independent ESP32-C3 / ESP-IDF 6.0 project, configurable SDA/SCL,
  I2C frequency and per-side addresses. Default bench values are SDA8/SCL9,
  400 kHz, LEFT 0x06 and RIGHT 0x46; physical identity must be checked manually.
- Retain 200-Hz, 60-second quiet acquisition with finite 1-ms I2C waits,
  post-stop reporting, missed-release/timing and read-recovery accounting.
- Include native recovery-accounting tests and their standalone runner.

No robot, motor bridge, LUT or PID dependency is required. The test never drives
motors and writes no NVS data. Ordinary driver consumers do not need diagnostics.
The local hardware test explicitly requires a sibling `esp_rt_diagnostics`
checkout and disables Component Manager; this is documented rather than
presented as a remotely resolved installation. See its README and AGENTS-APP.md
for configuration, timing contract and execution instructions.

Validation: native ASan/UBSan recovery tests and ESP-IDF 6.0 / ESP32-C3 build.
Extraction compilation is not a new hardware qualification; physical wiring,
acquisition timing and peripheral-fault behavior still need bench verification.
