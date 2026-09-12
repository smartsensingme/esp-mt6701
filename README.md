# `esp-mt6701`

Portuguese documentation: [README.pt-br.md](README.pt-br.md).

`esp-mt6701` is an ESP-IDF component for reading the 14-bit absolute angle of
the MagnTek MT6701 over I2C. It also provides software zero and direction,
multi-turn tracking, and a filtered velocity estimate.

The driver uses the modern ESP-IDF master API from `driver/i2c_master.h`. The
application owns the I2C bus and device handles and supplies the registered
device handle to `mt6701_init()`.

## Scope

The component provides:

- one two-byte burst read for the native 14-bit angle;
- software zero and direction without writing MT6701 EEPROM;
- cached angle in native counts or degrees;
- fresh angle reads in counts, degrees, or radians;
- signed multi-turn tracking;
- first-order filtered angular velocity in radians per second;
- optional per-instance mutex protection with no dynamic allocation.

It does not configure the I2C pins or clock, validate magnet placement, read
diagnostic outputs, persist software calibration, or apply an angle-linearity
LUT. Those responsibilities belong to the application or another component.

## Data flow

The normal periodic path is:

```text
MT6701 registers 0x03/0x04
        |
        v
mt6701_update()
        |
        +-- software zero and direction
        +-- cyclic displacement and turn counter
        +-- instantaneous velocity and low-pass filter
        |
        v
cached state
  |-- mt6701_get_last_angle_counts()
  |-- mt6701_get_last_angle_degrees()
  |-- mt6701_get_total_turns()
  |-- mt6701_get_total_angle_radians()
  `-- mt6701_get_velocity()
```

Call `mt6701_update()` once per control iteration, then use cached getters. This
keeps angle, turns, and velocity associated with the same sensor sample.

The functions whose names start with `mt6701_read_` acquire a new I2C sample.
For example, calling `mt6701_read_angle_degrees()` immediately after
`mt6701_update()` performs a second transaction and may return a slightly newer
angle than the cached velocity and turn count.

## Suggested ESP32-S3 wiring

This is the wiring used by the current reference project:

| ESP32-S3 | MT6701 | Purpose |
|---|---|---|
| `3V3` | `VDD` | Sensor and I2C logic supply |
| `GND` | `GND` / `VSS` | Common reference |
| `GPIO8` | `SDA` | I2C data |
| `GPIO9` | `SCL` | I2C clock |

GPIO8 and GPIO9 are application choices, not fixed driver requirements. The
MT6701 I2C address is fixed at `MT6701_I2C_ADDRESS` (`0x06`). The reference
application uses a 1 MHz bus. At that rate, keep wiring short and use suitable
external pull-ups to 3.3 V; internal ESP32-S3 pull-ups are weak and should not
be the first choice for a robust high-speed bus.

The MT6701 analog, ABI, and UVW outputs are not used by this I2C driver.

## Adding the component

Place the repository under the application's `components` directory and add it
as a requirement of the consuming component:

```cmake
idf_component_register(
    SRCS "my_control.c"
    INCLUDE_DIRS "."
    REQUIRES esp-mt6701
)
```

Include the public header:

```c
#include "mt6701.h"
```

## Kconfig

`Component config -> MT6701 Driver Configuration` exposes:

| Option | Meaning |
|---|---|
| `CONFIG_MT6701_THREAD_SAFE=y` | Creates one static FreeRTOS mutex per instance and protects I2C transactions and shared state. |
| `CONFIG_MT6701_THREAD_SAFE=n` | Compiles out mutex operations for a lower-overhead, single-owner real-time path. |

When thread safety is disabled, do not access the same `mt6701_dev_t` instance
concurrently from multiple tasks or ISRs. None of the APIs are intended for ISR
use because sensor reads call the blocking I2C master driver.

## Complete initialization example

```c
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "mt6701.h"

static i2c_master_bus_handle_t sensor_bus;
static i2c_master_dev_handle_t sensor_i2c_device;
static mt6701_dev_t sensor;

esp_err_t sensor_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &sensor_bus),
                        "SENSOR", "create I2C bus");

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MT6701_I2C_ADDRESS,
        .scl_speed_hz = 1000000,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(sensor_bus, &device_config,
                                  &sensor_i2c_device),
        "SENSOR", "register MT6701");

    ESP_RETURN_ON_ERROR(mt6701_init(&sensor, sensor_i2c_device),
                        "SENSOR", "initialize MT6701");

    return mt6701_set_software_direction(&sensor, MT6701_DIR_CW);
}
```

The application remains responsible for removing the I2C device and deleting
the bus when their lifetime ends. The MT6701 driver has no `deinit()` function
because it allocates no dynamic resource of its own.

## Periodic sampling example

```c
esp_err_t control_sample(float *angle_deg, float *velocity_rad_s,
                         int32_t *turns)
{
    ESP_RETURN_ON_ERROR(mt6701_update(&sensor), "SENSOR", "sample MT6701");

    ESP_RETURN_ON_ERROR(mt6701_get_last_angle_degrees(&sensor, angle_deg),
                        "SENSOR", "read cached angle");
    ESP_RETURN_ON_ERROR(mt6701_get_velocity(&sensor, velocity_rad_s),
                        "SENSOR", "read cached velocity");
    return mt6701_get_total_turns(&sensor, turns);
}
```

These getters do not access I2C. If an application needs only one independent
angle and does not use tracking, it may instead call a fresh-read function such
as `mt6701_read_angle_degrees()`.

## Native counts and `esp_angle_lut`

`mt6701_get_last_angle_counts()` returns the cached 14-bit angle after software
zero and direction have been applied. This is the appropriate interface for a
count-domain linearization:

```c
uint16_t angle_counts;
ESP_ERROR_CHECK(mt6701_update(&sensor));
ESP_ERROR_CHECK(mt6701_get_last_angle_counts(&sensor, &angle_counts));

uint16_t corrected_counts = esp_angle_lut_apply(angle_counts);
float corrected_degrees = mt6701_counts_to_degrees(corrected_counts);
```

This direct example assumes `ESP_ANGLE_LUT_FULL_SCALE_COUNTS == 16384`. If the
LUT component is configured for another resolution, the application must scale
between MT6701 counts and LUT counts before and after correction.

The correction belongs after MT6701 software zero/direction processing and
before angle unwrapping, velocity estimation, or control calculations. The
MT6701 driver's internal velocity estimate is based on its uncorrected cached
angle; an application that requires LUT-corrected velocity should estimate it
from the corrected angle outside this driver.

## Velocity estimator

For two consecutive calibrated samples, `mt6701_update()` calculates

```text
delta = shortest cyclic difference in counts
instantaneous_velocity = delta * (2*pi/16384) / dt
filtered_velocity = alpha * instantaneous_velocity
                  + (1 - alpha) * previous_filtered_velocity
```

`velocity_filter_alpha` is initialized to `0.20f`:

- a value closer to 1 follows new measurements more quickly but passes more
  quantization and timing noise;
- a value closer to 0 smooths more strongly but adds lag;
- 0 holds the previous filtered value;
- 1 disables smoothing and uses instantaneous velocity.

The current API exposes this coefficient as a field of `mt6701_dev_t`. If it
must be changed, do so after `mt6701_init()` and before starting concurrent
sampling. The driver does not clamp or validate assignments made directly by
the application.

## Multi-turn sampling limit

The unwrapping algorithm assumes the shaft moves by less than half a revolution
between consecutive successful `mt6701_update()` calls. Otherwise it cannot
distinguish the real motion from the shorter displacement in the opposite
direction.

For a maximum speed `N` in RPM, use an update frequency satisfying

```text
f_update > N / 30
```

This is an aliasing limit, not a recommended operating margin. Scheduling
jitter, I2C errors, and acceleration require additional margin.

## Software zero and direction

`mt6701_set_software_zero()` reads the current raw angle and stores it as the
new origin. It also clears turns and velocity and resets the timing baseline.

`mt6701_set_software_direction()` changes whether native counts are preserved
or inverted. It keeps the existing zero but clears turns and velocity and reads
a new baseline so the coordinate change is not interpreted as shaft movement.

Both settings live only in RAM and return to zero/CW after `mt6701_init()` or a
restart. The component deliberately avoids MT6701 EEPROM programming.

## API summary and call relationships

| Function | New I2C read? | Called inside the component by | Main result |
|---|:---:|---|---|
| `mt6701_init` | Yes | Nobody | Initialize and verify communication |
| `mt6701_init_from_raw_angle` | No | Nobody | Initialize from a caller-acquired sample |
| `mt6701_read_raw_angle` | Yes | `init`, `update`, calibrated reads, zero, direction | Native count |
| `mt6701_counts_to_degrees` | No | Fresh/cached degree functions | Pure conversion |
| `mt6701_update` | Yes | Nobody | Update all tracking state |
| `mt6701_update_from_raw_angle` | No | `mt6701_update` | Update tracking from a caller-acquired sample |
| `mt6701_read_calibrated_angle_counts` | Yes | Fresh degree/radian functions | Fresh processed count |
| `mt6701_read_angle_degrees` | Yes | Nobody | Fresh angle in degrees |
| `mt6701_read_angle_radians` | Yes | Nobody | Fresh angle in radians |
| `mt6701_get_last_angle_counts` | No | Nobody | Cached processed count |
| `mt6701_get_last_angle_degrees` | No | Nobody | Cached angle in degrees |
| `mt6701_get_total_angle_radians` | No | Nobody | Cached continuous angle |
| `mt6701_get_total_turns` | No | Nobody | Cached full-turn count |
| `mt6701_get_velocity` | No | Nobody | Cached filtered velocity |
| `mt6701_set_software_zero` | Yes | Nobody | Rebase origin and tracking |
| `mt6701_set_software_direction` | Yes | Nobody | Change direction and reset tracking |

Detailed parameters, return codes, internal callees, and caller relationships
are documented in [`include/mt6701.h`](include/mt6701.h). The implementation in
[`mt6701.c`](mt6701.c) is divided into commented functional blocks.

Applications that own an asynchronous acquisition pipeline can use
`mt6701_init_from_raw_angle()` and `mt6701_update_from_raw_angle()`. The caller
must retain the receive buffer until the transfer completes and supply the
timestamp associated with that sample. The component then performs the same
software calibration, unwrapping, turn tracking, and velocity update without
starting another I2C transaction.

## Practical limitations

- Angle resolution is fixed at 14 bits: 16384 counts per revolution.
- Multi-turn state is held in RAM and is not retained across resets.
- Software zero and direction are held in RAM and are not stored in the sensor.
- I2C read errors leave cached tracking state unchanged.
- The driver does not detect magnetic-field weakness, excessive air gap, or
  magnet eccentricity.
- The filtered velocity is a simple first-order estimate, not a Kalman filter.
- An angle-linearity LUT is intentionally kept in the separate reusable
  `esp_angle_lut` component.
