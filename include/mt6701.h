#ifndef MT6701_H_
#define MT6701_H_

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#if CONFIG_MT6701_THREAD_SAFE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

#define MT6701_I2C_ADDRESS 0x06
#define MT6701_COUNTS_PER_REVOLUTION 16384U

// Registers
#define MT6701_REG_ANGLE_H 0x03
#define MT6701_REG_ANGLE_L 0x04

typedef enum {
    MT6701_DIR_CW = 0, // Clockwise (standard)
    MT6701_DIR_CCW = 1 // Counter-Clockwise (inverted)
} mt6701_direction_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;

    // Software configurations
    uint16_t zero_offset;         // Offset subtract from raw angle (0 to 16383)
    mt6701_direction_t direction; // Rotation direction

    // Multi-turn tracking
    int32_t total_turns; // Number of full rotations
    uint16_t
        last_calibrated_angle; // Previous angle with offset and direction applied

    // Velocity estimation
    int64_t last_timestamp_us;   // esp_timer_get_time() of the last update
    float velocity_rad_s;        // Filtered angular velocity in radians/second
    float velocity_filter_alpha; // Low-pass filter coefficient for velocity (0.0
                                 // to 1.0)

#if CONFIG_MT6701_THREAD_SAFE
    SemaphoreHandle_t lock;
    StaticSemaphore_t lock_buffer;
#endif
} mt6701_dev_t;

/**
 * @brief Initialize the MT6701 device handle
 *
 * @param dev Pointer to the device structure
 * @param i2c_dev ESP-IDF I2C master device handle
 * @return esp_err_t ESP_OK on success, or appropriate error code
 */
esp_err_t mt6701_init(mt6701_dev_t *dev, i2c_master_dev_handle_t i2c_dev);

/**
 * @brief Read the raw 14-bit angle directly from the MT6701 hardware registers
 *
 * @param dev Pointer to the device structure
 * @param raw_angle Pointer to store the 14-bit raw angle (0 to 16383)
 * @return esp_err_t ESP_OK on success, or I2C communication error code
 */
esp_err_t mt6701_read_raw_angle(mt6701_dev_t *dev, uint16_t *raw_angle);

/**
 * @brief Convert a 14-bit MT6701 angle count to degrees
 *
 * This is a pure conversion and does not access the I2C bus.
 *
 * @param angle_counts 14-bit angle count (0 to 16383), raw or calibrated
 * @return Angle in degrees (0.0 to approximately 359.978)
 */
float mt6701_counts_to_degrees(uint16_t angle_counts);

/**
 * @brief Query the sensor and update internal software tracking states
 * (multi-turn & velocity)
 *
 * This should be called regularly (e.g. at 100Hz–1000Hz) in a control loop to
 * calculate velocity and accumulate turns correctly without wrap-around
 * aliasing.
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success, or update/communication error code
 */
esp_err_t mt6701_update(mt6701_dev_t *dev);

/**
 * @brief Read the calibrated angle as 14-bit counts
 *
 * @param dev Pointer to the device structure
 * @param angle_counts Pointer to store the calibrated angle counts
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_read_calibrated_angle_counts(mt6701_dev_t *dev,
                                              uint16_t *angle_counts);

/**
 * @brief Read the latest calibrated angle in degrees (0.0 to 360.0)
 *
 * @param dev Pointer to the device structure
 * @param degrees Pointer to store the angle in degrees
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_read_angle_degrees(mt6701_dev_t *dev, float *degrees);

/**
 * @brief Get the last calibrated angle stored by mt6701_update(), in degrees
 *
 * This function returns the cached sample and does not access the I2C bus.
 *
 * @param dev Pointer to the device structure
 * @param degrees Pointer to store the cached calibrated angle in degrees
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_get_last_angle_degrees(mt6701_dev_t *dev, float *degrees);

/**
 * @brief Get the last calibrated angle cached by mt6701_update(), in counts
 *
 * The returned 14-bit value already has the software zero offset and direction
 * applied. This function does not access the I2C bus.
 *
 * @param dev Pointer to the device structure
 * @param angle_counts Pointer to store the cached angle (0 to 16383)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_get_last_angle_counts(mt6701_dev_t *dev,
                                       uint16_t *angle_counts);

/**
 * @brief Read the latest calibrated angle in radians (0.0 to 2*PI)
 *
 * @param dev Pointer to the device structure
 * @param radians Pointer to store the angle in radians
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_read_angle_radians(mt6701_dev_t *dev, float *radians);

/**
 * @brief Get the total accumulated rotation in radians (including multi-turns)
 *
 * @param dev Pointer to the device structure
 * @param total_radians Pointer to store the total accumulated radians
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_get_total_angle_radians(mt6701_dev_t *dev,
                                         float *total_radians);

/**
 * @brief Get the total accumulated turns (full rotations)
 *
 * @param dev Pointer to the device structure
 * @param turns Pointer to store the total turns
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_get_total_turns(mt6701_dev_t *dev, int32_t *turns);

/**
 * @brief Get the filtered angular velocity in radians per second (rad/s)
 *
 * @param dev Pointer to the device structure
 * @param velocity Pointer to store the filtered velocity
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_get_velocity(mt6701_dev_t *dev, float *velocity);

/**
 * @brief Set the current mechanical position as the zero position in software
 *
 * @param dev Pointer to the device structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_set_software_zero(mt6701_dev_t *dev);

/**
 * @brief Configure the sensor rotation direction in software
 *
 * @param dev Pointer to the device structure
 * @param dir CW or CCW direction
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mt6701_set_software_direction(mt6701_dev_t *dev,
                                        mt6701_direction_t dir);

#endif // MT6701_H_
