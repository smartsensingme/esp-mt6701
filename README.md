# MT6701 Magnetic Encoder Driver - ESP-IDF Component

*Read in other languages: [Português](README.pt-br.md)*

This directory contains a clean, decoupled, and highly optimized component for the **MagnTek MT6701** 14-bit magnetic rotary encoder, designed for **ESP-IDF v6** and targeting high-speed operations.

It utilizes the modern ESP-IDF master I2C driver (`driver/i2c_master.h`) and features software-driven calibrations, velocity estimation, multi-turn accumulation, and optional thread safety.

---

## 🛠️ Features

1.  **Software-Driven Design (Read-Only I2C):** To bypass the hardware EEPROM requirements (which mandate a 4.5V–5.5V VDD supply and a 600ms blocking delay), this driver handles Zero Position and Rotation Direction entirely in software. The I2C bus is read-only during execution, ensuring maximum speed and safety.
2.  **14-bit Resolution:** Supports the full 14-bit absolute angle output (`0` to `16383` steps) of the MT6701.
3.  **Thread Safety Toggle (Kconfig):**
    *   **`CONFIG_MT6701_THREAD_SAFE=y`** (Default): Protects internal states and I2C registers using a FreeRTOS Mutex, ensuring safe concurrent accesses.
    *   **`CONFIG_MT6701_THREAD_SAFE=n`**: Compiles out all mutex operations, providing lock-free, zero-overhead routines for high-frequency control loops.
4.  **Multi-Turn Accumulation:** Automatically tracks wrapping (crossover transitions at half-resolution) to monitor total turns and continuous accumulated rotation.
5.  **Velocity Estimation:** Computes angular speed in radians per second (`rad/s`) using hardware timestamps (`esp_timer_get_time()`) combined with an adjustable low-pass filter to smooth out discretization noise.
6.  **Optimized Burst Reads:** Read routines fetch both angle registers (`0x03` and `0x04`) in a single, continuous 2-byte I2C transaction.

---

## 📈 MT6701 Hardware Specifications & Update Limits

### Hardware Performance
*   **Resolution:** 14-bit (16,384 positions per 360° revolution, approx $0.022^\circ$ per LSB).
*   **Integral Non-Linearity (INL):** $\pm 0.05^\circ$ (typical under ideal magnet centering and air gap).
*   **Transition Noise (Jitter):** $0.01^\circ$ RMS (typical at $25^\circ\text{C}$).
*   **Propagation Latency:** $< 100\,\mu\text{s}$ internal processing delay.

### Maximum `mt6701_update` Frequency
The maximum rate at which you can call `mt6701_update` is limited by I2C bus speeds and microcontroller overhead:
*   **At 400 kHz (I2C Fast Mode):** A single 2-byte burst transaction takes $\approx 73\,\mu\text{s}$. Including driver overhead, the theoretical maximum update rate is **$10\text{ kHz}$** ($100\,\mu\text{s}$ loop).
*   **At 1 MHz (I2C Fast Mode Plus):** The transaction takes $\approx 29\,\mu\text{s}$. The theoretical maximum update rate is **$20\text{ kHz}$** ($50\,\mu\text{s}$ loop).
*   **Recommended Update Rate:** A sampling rate of **$1\text{ kHz}$ to $5\text{ kHz}$** is recommended. This keeps CPU usage low, leaves bus bandwidth for other devices, and delivers highly responsive speed estimates.

> [!TIP]
> **Anti-Aliasing Limit (Max Motor Speed):**
> For the software turn-counter (multi-turn) to detect direction correctly, the motor must not rotate more than $180^\circ$ (half-revolution) between two consecutive calls to `mt6701_update`.
> The relationship between maximum motor speed $N$ (in RPM) and required update frequency $f_{update}$ is:
> $$f_{update} > \frac{N}{30}$$
> *   At **1 kHz** sampling rate, the driver supports motor speeds up to **30,000 RPM**.
> *   At **5 kHz** sampling rate, the driver supports motor speeds up to **150,000 RPM**.

---

## ⚙️ Configuration Properties

Via `menuconfig` (`Component config` -> `MT6701 Driver Configuration`):
*   **`CONFIG_MT6701_THREAD_SAFE`**: Enable or disable Mutex protection.

---

## 🚀 How to Add to Your Project

Add this repository as a Git submodule in your ESP-IDF project's `components` directory:
```bash
git submodule add https://github.com/smartsensingme/esp-mt6701.git components/esp-mt6701
```
Then, update your component `CMakeLists.txt` to require it:
```cmake
idf_component_register(SRCS "main.c"
                       REQUIRES esp-mt6701)
```

---

## 📖 API Usage Example

Include the driver header:
```c
#include "mt6701.h"
```

Initialize the device:
```c
// 1. Initialize your I2C master bus handle
i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 8,
    .scl_io_num = 9,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_config, &bus_handle);

// 2. Add the MT6701 device to the bus (Address 0x06)
i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = MT6701_I2C_ADDRESS,
    .scl_speed_hz = 400000, // Supports up to 1MHz Fast Mode Plus
};
i2c_master_dev_handle_t i2c_dev;
i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);

// 3. Initialize the driver device handle
mt6701_dev_t mt6701_device;
ESP_ERROR_CHECK(mt6701_init(&mt6701_device, i2c_dev));

// 4. Configure software-driven properties (Optional)
mt6701_set_software_direction(&mt6701_device, MT6701_DIR_CW);
```

Read sensor data in a loop:
```c
void control_loop_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Query sensor and update multi-turn & velocity tracking
        if (mt6701_update(&mt6701_device) == ESP_OK) {
            float deg, velocity;
            int32_t turns;
            
            mt6701_read_angle_degrees(&mt6701_device, &deg);
            mt6701_get_total_turns(&mt6701_device, &turns);
            mt6701_get_velocity(&mt6701_device, &velocity);
            
            printf("Angle: %.2f deg | Turns: %ld | Velocity: %.2f rad/s\n", deg, turns, velocity);
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1)); // Run at 1 kHz
    }
}
```

## 🗄️ API Reference

### `mt6701_init`
```c
esp_err_t mt6701_init(mt6701_dev_t *dev, i2c_master_dev_handle_t i2c_dev);
```
*   **Description:** Initializes the MT6701 device handler, sets up the default software values (zero-offset = 0, direction = CW, alpha = 0.20), creates the static mutex semaphore (if `CONFIG_MT6701_THREAD_SAFE` is enabled), and tests the physical I2C link by requesting a read.
*   **Parameters:**
    *   `dev`: Pointer to the `mt6701_dev_t` device structure instance (pre-allocated).
    *   `i2c_dev`: The ESP-IDF `i2c_master_dev_handle_t` registered device handle on the master I2C bus.
*   **Return Value:**
    *   `ESP_OK` on success (device responding and initialized).
    *   `ESP_ERR_INVALID_ARG` if `dev` or `i2c_dev` is `NULL`.
    *   `ESP_ERR_NO_MEM` if mutex allocation failed.
    *   I2C bus errors (e.g. `ESP_ERR_TIMEOUT`) if the sensor does not acknowledge on address `0x06`.

### `mt6701_read_raw_angle`
```c
esp_err_t mt6701_read_raw_angle(mt6701_dev_t *dev, uint16_t *raw_angle);
```
*   **Description:** Performs a continuous 2-byte burst transaction over the I2C bus starting at register `0x03` and ending at `0x04` to read the raw 14-bit angle directly from the hardware CORDIC processor.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `raw_angle`: Pointer to a `uint16_t` where the raw 14-bit angle (`0` to `16383`) will be stored.
*   **Return Value:**
    *   `ESP_OK` on successful read.
    *   `ESP_ERR_INVALID_ARG` if `dev` or `raw_angle` is `NULL`.
    *   I2C transmission error codes on failure.

### `mt6701_update`
```c
esp_err_t mt6701_update(mt6701_dev_t *dev);
```
*   **Description:** Reads the raw angle from the hardware, applies the software offset/direction configurations, and updates the internal multi-turn counter (detecting wrapping crossovers) and velocity estimation filter. This function must be called periodically in a constant time interval task loop (e.g. 1 kHz).
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` is `NULL`.
    *   I2C transmission error codes on failure.

### `mt6701_counts_to_degrees`
```c
float mt6701_counts_to_degrees(uint16_t angle_counts);
```
*   **Description:** Converts a raw or calibrated 14-bit angle count (`0` to `16383`) to degrees without accessing the I2C bus.
*   **Return Value:** Angle from `0.0f` to approximately `359.978f` degrees.

### `mt6701_read_calibrated_angle_counts`
```c
esp_err_t mt6701_read_calibrated_angle_counts(mt6701_dev_t *dev,
                                               uint16_t *angle_counts);
```
*   **Description:** Performs a new sensor read, applies the software zero-offset and direction calibration, and returns the angle as 14-bit counts.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `angle_counts`: Pointer to a `uint16_t` where the calibrated counts (`0` to `16383`) will be written.
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` or `angle_counts` is `NULL`.
    *   I2C transmission error codes on failure.

### `mt6701_read_angle_degrees`
```c
esp_err_t mt6701_read_angle_degrees(mt6701_dev_t *dev, float *degrees);
```
*   **Description:** Re-reads the sensor, calibrates it, and returns the current position mapped to a floating-point degree representation.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `degrees`: Pointer to a `float` to store the angle value (`0.0f` to `360.0f`).
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` or `degrees` is `NULL`.
    *   I2C transmission error codes on failure.

### `mt6701_read_angle_radians`
```c
esp_err_t mt6701_read_angle_radians(mt6701_dev_t *dev, float *radians);
```
*   **Description:** Re-reads the sensor, calibrates it, and returns the current position mapped to a floating-point radian representation.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `radians`: Pointer to a `float` to store the angle value (`0.0f` to `2*PI`).
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` or `radians` is `NULL`.
    *   I2C transmission error codes on failure.

### `mt6701_get_last_angle_degrees`
```c
esp_err_t mt6701_get_last_angle_degrees(mt6701_dev_t *dev, float *degrees);
```
*   **Description:** Returns the last calibrated sample stored by `mt6701_update` in degrees without performing another I2C read.
*   **Return Value:** `ESP_OK` on success, `ESP_ERR_INVALID_ARG` for null pointers, or `ESP_ERR_TIMEOUT` if the mutex cannot be acquired.

### `mt6701_get_total_angle_radians`
```c
esp_err_t mt6701_get_total_angle_radians(mt6701_dev_t *dev, float *total_radians);
```
*   **Description:** Calculates the total accumulated rotation in radians, including all turns computed during the periodic execution of `mt6701_update`.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `total_radians`: Pointer to a `float` to store the total accumulated radians.
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` or `total_radians` is `NULL`.

### `mt6701_get_total_turns`
```c
esp_err_t mt6701_get_total_turns(mt6701_dev_t *dev, int32_t *turns);
```
*   **Description:** Reads the multi-turn counter value accumulated during execution.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `turns`: Pointer to an `int32_t` to store the count of full rotations (turns can be negative).
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` or `turns` is `NULL`.

### `mt6701_get_velocity`
```c
esp_err_t mt6701_get_velocity(mt6701_dev_t *dev, float *velocity);
```
*   **Description:** Retrieves the low-pass filtered angular velocity estimated during calls to `mt6701_update`.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `velocity`: Pointer to a `float` to store the speed in radians per second (`rad/s`).
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` or `velocity` is `NULL`.

### `mt6701_set_software_zero`
```c
esp_err_t mt6701_set_software_zero(mt6701_dev_t *dev);
```
*   **Description:** Reads the current hardware angle and sets it as the zero-offset value (`zero_offset`), resetting the turns and velocity states. All subsequent angle calls will measure relative to this reference point.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` is `NULL`.
    *   I2C transmission error codes on failure.

### `mt6701_set_software_direction`
```c
esp_err_t mt6701_set_software_direction(mt6701_dev_t *dev, mt6701_direction_t dir);
```
*   **Description:** Configures the software rotation direction behavior. Modifies the internal direction setting and resets turns and velocity states to prevent configuration transition glitches.
*   **Parameters:**
    *   `dev`: Pointer to the initialized `mt6701_dev_t` structure.
    *   `dir`: Rotation direction (`MT6701_DIR_CW` for standard, `MT6701_DIR_CCW` for inverted).
*   **Return Value:**
    *   `ESP_OK` on success.
    *   `ESP_ERR_INVALID_ARG` if `dev` is `NULL`.
    *   I2C transmission error codes on failure.

---
![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Description

This project is part of the **SmartSensing.me** ecosystem. We apply real fundamentals of instrumentation engineering and high-performance embedded systems.

Unlike superficial, clickbait content, this repository delivers:
- **Originality:** Unique implementations based on nearly 30 years of academic experience.
- **Technical Depth:** Professional usage of the ESP-IDF framework and FreeRTOS.
- **Pedagogy:** Documented and structured code for those seeking genuine technical growth.

> "We transform signals from the physical world into digital intelligence, with no shortcuts."

---

## 👤 About the Author

**José Alexandre de França** *Associate Professor at the Department of Electrical Engineering of UEL*

Electrical Engineer with nearly three decades of experience in undergraduate and postgraduate teaching. PhD in Electrical Engineering, researcher in electronic instrumentation, and embedded systems developer. SmartSensing.me is my commitment to raising the bar of technology education in Brazil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
