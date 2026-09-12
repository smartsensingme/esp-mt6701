#include "mt6701.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "MT6701";

static esp_err_t initialize_state(mt6701_dev_t *dev,
                                  i2c_master_dev_handle_t i2c_dev) {
  if (!dev || !i2c_dev) {
    return ESP_ERR_INVALID_ARG;
  }

  dev->i2c_dev = i2c_dev;
  dev->zero_offset = 0;
  dev->direction = MT6701_DIR_CW;
  dev->total_turns = 0;
  dev->last_calibrated_angle = 0;
  dev->last_timestamp_us = 0;
  dev->velocity_rad_s = 0.0f;
  dev->velocity_filter_alpha = 0.20f;

#if CONFIG_MT6701_THREAD_SAFE
  dev->lock = xSemaphoreCreateMutexStatic(&dev->lock_buffer);
  if (dev->lock == NULL) {
    ESP_LOGE(TAG, "Failed to create static mutex");
    return ESP_ERR_NO_MEM;
  }
#endif
  return ESP_OK;
}

static void establish_baseline(mt6701_dev_t *dev, uint16_t raw_angle,
                               int64_t timestamp_us) {
  uint16_t calibrated = (raw_angle - dev->zero_offset) & 0x3FFF;
  if (dev->direction == MT6701_DIR_CCW) {
    calibrated = (16384 - calibrated) & 0x3FFF;
  }
  dev->last_calibrated_angle = calibrated;
  dev->last_timestamp_us = timestamp_us;
}

esp_err_t mt6701_init(mt6701_dev_t *dev, i2c_master_dev_handle_t i2c_dev) {
  /* Interface block: both caller-owned objects are required. */
  esp_err_t state_error = initialize_state(dev, i2c_dev);
  if (state_error != ESP_OK) {
    return state_error;
  }

  /* Communication block: verify the registered address and bus by acquiring
   * one native sample through the same path used during normal operation. */
  uint16_t dummy_angle;
  esp_err_t err = mt6701_read_raw_angle(dev, &dummy_angle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to communicate with MT6701 (err: %d)", err);
    return err;
  }

  /* Baseline block: apply the initial zero/direction convention and timestamp
   * this sample so the first update can calculate a bounded displacement. */
  establish_baseline(dev, dummy_angle, esp_timer_get_time());

  ESP_LOGI(TAG, "Initialized successfully. Initial raw angle: %u", dummy_angle);
  return ESP_OK;
}

esp_err_t mt6701_init_from_raw_angle(mt6701_dev_t *dev,
                                     i2c_master_dev_handle_t i2c_dev,
                                     uint16_t raw_angle, int64_t timestamp_us) {
  if (raw_angle >= MT6701_COUNTS_PER_REVOLUTION || timestamp_us <= 0) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t error = initialize_state(dev, i2c_dev);
  if (error != ESP_OK) {
    return error;
  }
  establish_baseline(dev, raw_angle, timestamp_us);
  ESP_LOGI(TAG, "Initialized from supplied raw angle: %u", raw_angle);
  return ESP_OK;
}

esp_err_t mt6701_read_raw_angle(mt6701_dev_t *dev, uint16_t *raw_angle) {
  /* Interface block: require both an initialized instance and destination. */
  if (!dev || !raw_angle) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Synchronization block: serialize the complete I2C transaction with every
   * other operation using this instance. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Transaction block: write the first angle-register address, then read both
   * consecutive registers in one repeated-start transmit/receive operation. */
  uint8_t reg = MT6701_REG_ANGLE_H;
  uint8_t buffer[2];
  esp_err_t err =
      i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2, -1);

  /* Decode block: concatenate the high byte and upper six bits of the low
   * register only after a successful transfer. */
  if (err == ESP_OK) {
    *raw_angle = (((uint16_t)buffer[0] << 6) | (buffer[1] >> 2)) & 0x3FFF;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: make the instance available on both success and I2C error.
   */
  xSemaphoreGive(dev->lock);
#endif
  return err;
}

float mt6701_counts_to_degrees(uint16_t angle_counts) {
  /* Conversion block: one count represents 360/16384 degrees. The caller is
   * responsible for supplying a valid 14-bit value. */
  return (float)angle_counts * (360.0f / 16384.0f);
}

esp_err_t mt6701_update(mt6701_dev_t *dev) {
  /* Interface block: an instance is required before accessing its I2C handle.
   */
  if (!dev) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Acquisition block: read before taking the state mutex because the raw
   * read already owns that same non-recursive mutex when thread safety is on.
   */
  uint16_t raw;
  esp_err_t err = mt6701_read_raw_angle(dev, &raw);
  if (err != ESP_OK) {
    return err;
  }

  return mt6701_update_from_raw_angle(dev, raw, esp_timer_get_time());
}

esp_err_t mt6701_update_from_raw_angle(mt6701_dev_t *dev, uint16_t raw,
                                       int64_t timestamp_us) {
  if (!dev || raw >= MT6701_COUNTS_PER_REVOLUTION || timestamp_us <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Timestamp block: compare shared state while holding the optional mutex so
   * a concurrent reader cannot race this validation and publication. */
  if (timestamp_us <= dev->last_timestamp_us) {
#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_ERR_INVALID_ARG;
  }

  /* Calibration block: subtract software zero with 14-bit wraparound and
   * optionally mirror the coordinate system for counter-clockwise positive. */
  uint16_t calibrated = (raw - dev->zero_offset) & 0x3FFF;
  if (dev->direction == MT6701_DIR_CCW) {
    calibrated = (16384 - calibrated) & 0x3FFF;
  }

  int64_t dt_us = timestamp_us - dev->last_timestamp_us;

  /* Dynamic-state block: a missing or nonpositive interval cannot be used for
   * differentiation, but the sample can still become the next baseline. */
  if (dev->last_timestamp_us > 0 && dt_us > 0) {
    /* Difference block: start with the direct difference of cyclic counts. */
    int32_t delta_angle =
        (int32_t)calibrated - (int32_t)dev->last_calibrated_angle;

    /* Unwrapping block: choose the equivalent displacement within half a
     * revolution and record the associated forward/backward wrap crossing. */
    if (delta_angle < -8192) {
      delta_angle += 16384;
      dev->total_turns++;
    } else if (delta_angle > 8192) {
      delta_angle -= 16384;
      dev->total_turns--;
    }

    /* Velocity block: convert count displacement and measured time to the
     * signed instantaneous angular velocity in radians per second. */
    float dt_sec = (float)dt_us / 1000000.0f;
    float velocity_instant =
        ((float)delta_angle * (2.0f * M_PI / 16384.0f)) / dt_sec;

    /* Filter block: exponential first-order smoothing gives alpha weight to
     * the newest estimate and 1-alpha weight to retained history. */
    dev->velocity_rad_s =
        (dev->velocity_filter_alpha * velocity_instant) +
        ((1.0f - dev->velocity_filter_alpha) * dev->velocity_rad_s);
  }

  /* Publication block: retain this calibrated sample and timestamp as the
   * baseline for getters and the next update. */
  dev->last_calibrated_angle = calibrated;
  dev->last_timestamp_us = timestamp_us;

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: publish the complete state update as one critical section.
   */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_read_calibrated_angle_counts(mt6701_dev_t *dev,
                                              uint16_t *angle_counts) {
  /* Interface block: require a sensor instance and result destination. */
  if (!dev || !angle_counts) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Acquisition block: obtain a fresh hardware sample. This operation does
   * not alter the cached state maintained by mt6701_update(). */
  uint16_t raw;
  esp_err_t err = mt6701_read_raw_angle(dev, &raw);
  if (err != ESP_OK) {
    return err;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Calibration-lock block: protect zero and direction while both are applied
   * to the same raw sample. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Calibration block: subtract the zero modulo one revolution, then mirror
   * the result when the configured positive direction is inverted. */
  uint16_t calibrated = (raw - dev->zero_offset) & 0x3FFF;
  if (dev->direction == MT6701_DIR_CCW) {
    calibrated = (16384 - calibrated) & 0x3FFF;
  }
  *angle_counts = calibrated;

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: finish the coherent calibration snapshot. */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_read_angle_degrees(mt6701_dev_t *dev, float *degrees) {
  /* Interface block: reject missing instance or output storage. */
  if (!dev || !degrees) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Acquisition block: request a new calibrated count-domain measurement. */
  uint16_t calibrated;
  esp_err_t err = mt6701_read_calibrated_angle_counts(dev, &calibrated);

  /* Conversion block: do not overwrite the caller's output after a failed
   * sensor transaction. */
  if (err == ESP_OK) {
    *degrees = mt6701_counts_to_degrees(calibrated);
  }
  return err;
}

esp_err_t mt6701_get_last_angle_degrees(mt6701_dev_t *dev, float *degrees) {
  /* Interface block: require cached-state source and output destination. */
  if (!dev || !degrees) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Snapshot-lock block: prevent an update from changing the cached count
   * while it is being converted. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Conversion block: reuse the pure count-to-degree conversion without
   * issuing another I2C transaction. */
  *degrees = mt6701_counts_to_degrees(dev->last_calibrated_angle);

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: end the cached sample snapshot. */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_get_last_angle_counts(mt6701_dev_t *dev,
                                       uint16_t *angle_counts) {
  /* Interface block: require cached-state source and output destination. */
  if (!dev || !angle_counts) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Snapshot-lock block: keep the cached 16-bit field consistent with a
   * concurrent update and with other state-changing API calls. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Copy block: return the already offset- and direction-corrected sample. */
  *angle_counts = dev->last_calibrated_angle;

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: end the cached sample snapshot. */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_read_angle_radians(mt6701_dev_t *dev, float *radians) {
  /* Interface block: reject missing instance or output storage. */
  if (!dev || !radians) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Acquisition block: request a fresh calibrated count-domain sample. */
  uint16_t calibrated;
  esp_err_t err = mt6701_read_calibrated_angle_counts(dev, &calibrated);

  /* Conversion block: map one full 14-bit count range to 2*pi radians only
   * when acquisition succeeded. */
  if (err == ESP_OK) {
    *radians = (float)calibrated * (2.0f * M_PI / 16384.0f);
  }
  return err;
}

esp_err_t mt6701_get_total_angle_radians(mt6701_dev_t *dev,
                                         float *total_radians) {
  /* Interface block: require cached-state source and output destination. */
  if (!dev || !total_radians) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Snapshot-lock block: turns and cyclic angle must come from one coherent
   * mt6701_update() state. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Composition block: convert the cyclic remainder to radians and add its
   * signed integer number of complete revolutions. */
  float current_rad =
      (float)dev->last_calibrated_angle * (2.0f * M_PI / 16384.0f);
  *total_radians = ((float)dev->total_turns * 2.0f * M_PI) + current_rad;

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: end the multi-field cached-state snapshot. */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_get_total_turns(mt6701_dev_t *dev, int32_t *turns) {
  /* Interface block: require cached-state source and output destination. */
  if (!dev || !turns) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Snapshot-lock block: serialize access to the wrap counter. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Copy block: return the signed number of complete wrap crossings. */
  *turns = dev->total_turns;

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: end the counter snapshot. */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_get_velocity(mt6701_dev_t *dev, float *velocity) {
  /* Interface block: require cached-state source and output destination. */
  if (!dev || !velocity) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Snapshot-lock block: serialize access to the filtered velocity state. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Copy block: return the filter output produced by the latest update. */
  *velocity = dev->velocity_rad_s;

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: end the velocity snapshot. */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_set_software_zero(mt6701_dev_t *dev) {
  /* Interface block: a live instance is required for the acquisition. */
  if (!dev) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Acquisition block: capture the physical position that will define zero.
   * The raw-read routine manages the I2C mutex independently. */
  uint16_t raw;
  esp_err_t err = mt6701_read_raw_angle(dev, &raw);
  if (err != ESP_OK) {
    return err;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* State-lock block: publish the new coordinate origin and all dependent
   * state together. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Rebase block: retain the raw count as the software offset and erase state
   * whose interpretation depended on the previous origin. */
  dev->zero_offset = raw;
  dev->total_turns = 0;
  dev->last_calibrated_angle = 0;
  dev->last_timestamp_us = esp_timer_get_time();
  dev->velocity_rad_s = 0.0f;

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: finish the coordinate-system transition. */
  xSemaphoreGive(dev->lock);
#endif
  return ESP_OK;
}

esp_err_t mt6701_set_software_direction(mt6701_dev_t *dev,
                                        mt6701_direction_t dir) {
  /* Interface block: an instance is required before its state can change. */
  if (!dev) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Transition-lock block: begin changing the coordinate interpretation. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Reset block: store the new convention and discard dynamic states whose
   * signs or wrap history belonged to the old direction. */
  dev->direction = dir;
  dev->total_turns = 0;
  dev->velocity_rad_s = 0.0f;

  /* Reacquisition block: a new raw baseline is needed to prevent the next
   * update from interpreting the direction change as physical movement. */
  uint16_t dummy_angle;
#if CONFIG_MT6701_THREAD_SAFE
  /* mt6701_read_raw_angle() takes the same non-recursive mutex, so release it
   * temporarily to avoid self-deadlock during the I2C transaction. */
  xSemaphoreGive(dev->lock);
#endif
  esp_err_t err = mt6701_read_raw_angle(dev, &dummy_angle);
#if CONFIG_MT6701_THREAD_SAFE
  /* Restore exclusive state access before publishing the acquired baseline. */
  if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Baseline block: on successful acquisition, apply the unchanged zero and
   * new direction, then associate the result with a fresh timestamp. */
  if (err == ESP_OK) {
    uint16_t calibrated = (dummy_angle - dev->zero_offset) & 0x3FFF;
    if (dev->direction == MT6701_DIR_CCW) {
      calibrated = (16384 - calibrated) & 0x3FFF;
    }
    dev->last_calibrated_angle = calibrated;
    dev->last_timestamp_us = esp_timer_get_time();
  }

#if CONFIG_MT6701_THREAD_SAFE
  /* Release block: complete the direction transition on success or I2C error.
   */
  xSemaphoreGive(dev->lock);
#endif
  return err;
}
