#ifndef MT6701_H_
#define MT6701_H_

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#if CONFIG_MT6701_THREAD_SAFE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

/** Fixed 7-bit I2C address of the MT6701. */
#define MT6701_I2C_ADDRESS 0x06

/** Native 14-bit positions in one complete revolution. */
#define MT6701_COUNTS_PER_REVOLUTION 16384U

/** Register containing angle bits 13 through 6. */
#define MT6701_REG_ANGLE_H 0x03

/** Register containing angle bits 5 through 0 in its upper six bits. */
#define MT6701_REG_ANGLE_L 0x04

/** Software interpretation of increasing native angle counts. */
typedef enum {
  /** Preserve the sensor's native clockwise-positive count direction. */
  MT6701_DIR_CW = 0,
  /** Invert the native count direction in software. */
  MT6701_DIR_CCW = 1,
} mt6701_direction_t;

/** Configuration, tracking state, and optional synchronization for one sensor.
 */
typedef struct {
  /** ESP-IDF I2C device handle supplied by the application. */
  i2c_master_dev_handle_t i2c_dev;

  /** Raw native count subtracted before direction processing. */
  uint16_t zero_offset;
  /** Current software interpretation of positive rotation. */
  mt6701_direction_t direction;

  /** Signed number of full wrap crossings accumulated by mt6701_update(). */
  int32_t total_turns;
  /** Latest offset- and direction-corrected 14-bit sample. */
  uint16_t last_calibrated_angle;

  /** esp_timer_get_time() timestamp associated with the cached sample. */
  int64_t last_timestamp_us;
  /** Low-pass-filtered angular velocity in radians per second. */
  float velocity_rad_s;
  /** Weight of the newest instantaneous velocity sample, from 0 to 1. */
  float velocity_filter_alpha;

#if CONFIG_MT6701_THREAD_SAFE
  /** Mutex handle used to serialize I2C access and shared-state access. */
  SemaphoreHandle_t lock;
  /** Static storage for the mutex; the driver performs no heap allocation. */
  StaticSemaphore_t lock_buffer;
#endif
} mt6701_dev_t;

/**
 * @brief Initialize one MT6701 instance and verify I2C communication.
 *
 * The function initializes software zero, direction, multi-turn state, velocity
 * state, and the default velocity-filter coefficient of 0.20. When thread
 * safety is enabled it creates the instance's static mutex. It then reads one
 * raw sample, converts it using the initial software settings, and uses that
 * sample and its timestamp as the tracking baseline.
 *
 * This public entry point is called only by application code. No function in
 * this component calls it. Internally it calls mt6701_read_raw_angle(). The
 * application must create and register the ESP-IDF I2C bus/device first.
 *
 * @param[out] dev Caller-owned instance that remains valid while in use.
 * @param[in] i2c_dev Registered ESP-IDF I2C master device handle.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null argument,
 *         ESP_ERR_NO_MEM if static mutex creation fails, or an I2C error from
 *         the initial communication check.
 */
esp_err_t mt6701_init(mt6701_dev_t *dev, i2c_master_dev_handle_t i2c_dev);

/**
 * @brief Initialize an instance from an angle sample acquired by the caller.
 *
 * This variant initializes the same software state as mt6701_init(), but does
 * not perform an I2C transaction. It is intended for applications that own an
 * asynchronous or otherwise externally scheduled acquisition pipeline.
 * This public entry point is called only by application code. No function in
 * this component calls it.
 *
 * @param[out] dev Caller-owned instance that remains valid while in use.
 * @param[in] i2c_dev Registered ESP-IDF I2C device handle retained by the
 *                   instance for the regular synchronous APIs.
 * @param[in] raw_angle Native unsigned 14-bit angle sample.
 * @param[in] timestamp_us Positive timestamp associated with the sample.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for invalid arguments, or
 *         ESP_ERR_NO_MEM if optional mutex initialization fails.
 */
esp_err_t mt6701_init_from_raw_angle(mt6701_dev_t *dev,
                                     i2c_master_dev_handle_t i2c_dev,
                                     uint16_t raw_angle, int64_t timestamp_us);

/**
 * @brief Read the native 14-bit angle directly from the sensor registers.
 *
 * A single transmit/receive transaction selects register 0x03 and reads the two
 * consecutive angle bytes. The function reconstructs bits 13:0 without applying
 * software zero, software direction, multi-turn tracking, or velocity
 * filtering.
 *
 * This public function is called internally by mt6701_init(), mt6701_update(),
 * mt6701_read_calibrated_angle_counts(), mt6701_set_software_zero(), and
 * mt6701_set_software_direction(). Application code may also call it directly.
 * It calls no other function from this component.
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] raw_angle Destination for a value from 0 through 16383.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null pointer,
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken, or an I2C
 *         transaction error.
 */
esp_err_t mt6701_read_raw_angle(mt6701_dev_t *dev, uint16_t *raw_angle);

/**
 * @brief Convert a valid native angle count to degrees without I2C access.
 *
 * This pure conversion is called internally by mt6701_read_angle_degrees() and
 * mt6701_get_last_angle_degrees(). Application code may also call it directly.
 * It calls no other function from this component.
 *
 * @param[in] angle_counts Valid 14-bit raw or calibrated count, 0 through
 * 16383.
 * @return Corresponding angle from 0 degrees through approximately 359.978
 *         degrees.
 */
float mt6701_counts_to_degrees(uint16_t angle_counts);

/**
 * @brief Acquire one sample and update cached angle, turns, and velocity.
 *
 * The function reads the sensor, applies software zero and direction, unwraps
 * the shortest angular displacement, accumulates a wrap crossing when needed,
 * computes instantaneous velocity from the measured time interval, and updates
 * the first-order low-pass velocity estimate. Call it periodically and often
 * enough that the shaft moves less than half a revolution between samples.
 *
 * This public entry point is called only by application code. No function in
 * this component calls it. Internally it calls mt6701_read_raw_angle() and
 * esp_timer_get_time().
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null instance,
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken, or an I2C
 *         transaction error.
 */
esp_err_t mt6701_update(mt6701_dev_t *dev);

/**
 * @brief Incorporate one caller-acquired native sample into cached state.
 *
 * Applies software zero and direction, updates wrap tracking and the driver's
 * velocity estimate, and publishes the calibrated angle using the supplied
 * measurement timestamp. No I2C transaction is performed.
 *
 * This public function is called internally by mt6701_update(). An application
 * that owns the acquisition schedule may also call it directly.
 *
 * @param[in,out] dev Initialized sensor instance.
 * @param[in] raw_angle Native unsigned 14-bit angle sample.
 * @param[in] timestamp_us Positive timestamp associated with the sample and
 *                         newer than the previous accepted timestamp.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG for invalid input or a
 *         non-increasing timestamp.
 */
esp_err_t mt6701_update_from_raw_angle(mt6701_dev_t *dev, uint16_t raw_angle,
                                       int64_t timestamp_us);

/**
 * @brief Acquire and return a newly calibrated 14-bit angle sample.
 *
 * This function performs a new I2C read and applies the current software zero
 * and direction. It does not update the cached angle, timestamp, turn counter,
 * or velocity estimate maintained by mt6701_update().
 *
 * This public function is called internally by mt6701_read_angle_degrees() and
 * mt6701_read_angle_radians(). Application code may also call it directly.
 * Internally it calls mt6701_read_raw_angle().
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] angle_counts Destination for the calibrated value, 0 through
 *             16383.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null pointer,
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken, or an I2C
 *         transaction error.
 */
esp_err_t mt6701_read_calibrated_angle_counts(mt6701_dev_t *dev,
                                              uint16_t *angle_counts);

/**
 * @brief Acquire and return a newly calibrated angle in degrees.
 *
 * This public function is called only by application code. No function in this
 * component calls it. Internally it calls
 * mt6701_read_calibrated_angle_counts() and mt6701_counts_to_degrees(). Because
 * it initiates a new I2C transaction, use mt6701_get_last_angle_degrees() when
 * the sample already acquired by mt6701_update() is desired.
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] degrees Destination for an angle in [0, 360) degrees.
 * @return ESP_OK on success or the argument, mutex, or I2C error from the
 *         underlying calibrated read.
 */
esp_err_t mt6701_read_angle_degrees(mt6701_dev_t *dev, float *degrees);

/**
 * @brief Return the cached calibrated angle in degrees without I2C access.
 *
 * The returned value is the sample stored by the latest successful
 * mt6701_update(), or the baseline sample stored by mt6701_init().
 *
 * This public function is called only by application code. No function in this
 * component calls it. Internally it calls mt6701_counts_to_degrees().
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] degrees Destination for an angle in [0, 360) degrees.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null pointer, or
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken.
 */
esp_err_t mt6701_get_last_angle_degrees(mt6701_dev_t *dev, float *degrees);

/**
 * @brief Return the cached calibrated angle in native counts without I2C
 * access.
 *
 * The returned value already includes software zero and direction processing.
 * It is suitable for a count-domain linearization such as esp_angle_lut before
 * conversion to physical units.
 *
 * This public function is called only by application code. No function in this
 * component calls it, and it calls no other function from the component.
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] angle_counts Destination for the cached value, 0 through 16383.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null pointer, or
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken.
 */
esp_err_t mt6701_get_last_angle_counts(mt6701_dev_t *dev,
                                       uint16_t *angle_counts);

/**
 * @brief Acquire and return a newly calibrated angle in radians.
 *
 * This public function is called only by application code. No function in this
 * component calls it. Internally it calls
 * mt6701_read_calibrated_angle_counts(). It does not update tracking state.
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] radians Destination for an angle in [0, 2*pi) radians.
 * @return ESP_OK on success or the argument, mutex, or I2C error from the
 *         underlying calibrated read.
 */
esp_err_t mt6701_read_angle_radians(mt6701_dev_t *dev, float *radians);

/**
 * @brief Return the cached continuous multi-turn angle in radians.
 *
 * The value combines the signed full-turn counter and the latest calibrated
 * cyclic angle. It changes only after mt6701_update(), zeroing, or direction
 * reconfiguration; this getter performs no I2C transaction.
 *
 * This public function is called only by application code. No function in this
 * component calls it, and it calls no other function from the component.
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] total_radians Destination for the signed accumulated angle.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null pointer, or
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken.
 */
esp_err_t mt6701_get_total_angle_radians(mt6701_dev_t *dev,
                                         float *total_radians);

/**
 * @brief Return the cached signed number of complete wrap crossings.
 *
 * This public function is called only by application code. No function in this
 * component calls it, and it calls no other function from the component. It
 * performs no I2C transaction.
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] turns Destination for the signed full-turn counter.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null pointer, or
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken.
 */
esp_err_t mt6701_get_total_turns(mt6701_dev_t *dev, int32_t *turns);

/**
 * @brief Return the cached filtered angular velocity in radians per second.
 *
 * Velocity is calculated only by mt6701_update(). This getter performs no I2C
 * access and does not update the filter.
 *
 * This public function is called only by application code. No function in this
 * component calls it, and it calls no other function from the component.
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[out] velocity Destination for the signed filtered velocity in rad/s.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null pointer, or
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken.
 */
esp_err_t mt6701_get_velocity(mt6701_dev_t *dev, float *velocity);

/**
 * @brief Define the current raw sensor position as software zero.
 *
 * The function acquires a fresh raw sample, stores it as zero_offset, and
 * resets cached angle, turns, velocity, and timing baseline. No MT6701 EEPROM
 * register is written; the setting is lost when the instance is reinitialized.
 *
 * This public function is called only by application code. No function in this
 * component calls it. Internally it calls mt6701_read_raw_angle() and
 * esp_timer_get_time().
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null instance,
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken, or an I2C
 *         transaction error.
 */
esp_err_t mt6701_set_software_zero(mt6701_dev_t *dev);

/**
 * @brief Change the software-positive rotation direction and reset tracking.
 *
 * The function stores the requested direction, clears turns and velocity, then
 * acquires a fresh sample to establish an angle and timestamp baseline in the
 * new coordinate system. The zero offset remains unchanged. No MT6701 EEPROM
 * register is written.
 *
 * This public function is called only by application code. No function in this
 * component calls it. Internally it calls mt6701_read_raw_angle() and
 * esp_timer_get_time().
 *
 * @param[in,out] dev Previously initialized sensor instance.
 * @param[in] dir MT6701_DIR_CW to preserve counts or MT6701_DIR_CCW to invert
 *            them.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for a null instance,
 *         ESP_ERR_TIMEOUT if the optional mutex cannot be taken, or an I2C
 *         transaction error.
 */
esp_err_t mt6701_set_software_direction(mt6701_dev_t *dev,
                                        mt6701_direction_t dir);

#endif /* MT6701_H_ */
