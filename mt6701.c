#include "mt6701.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "MT6701";

esp_err_t mt6701_init(mt6701_dev_t *dev, i2c_master_dev_handle_t i2c_dev) {
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
    dev->velocity_filter_alpha =
        0.20f; // Default 20% instant velocity, 80% history

#if CONFIG_MT6701_THREAD_SAFE
    dev->lock = xSemaphoreCreateMutexStatic(&dev->lock_buffer);
    if (dev->lock == NULL) {
        ESP_LOGE(TAG, "Failed to create static mutex");
        return ESP_ERR_NO_MEM;
    }
#endif

    // Verify communication with sensor by performing a raw read
    uint16_t dummy_angle;
    esp_err_t err = mt6701_read_raw_angle(dev, &dummy_angle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with MT6701 (err: %d)", err);
        return err;
    }

    // Initialize tracking variables with the first calibrated reading
    uint16_t calibrated = (dummy_angle - dev->zero_offset) & 0x3FFF;
    if (dev->direction == MT6701_DIR_CCW) {
        calibrated = (16384 - calibrated) & 0x3FFF;
    }

    dev->last_calibrated_angle = calibrated;
    dev->last_timestamp_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Initialized successfully. Initial raw angle: %u", dummy_angle);
    return ESP_OK;
}

esp_err_t mt6701_read_raw_angle(mt6701_dev_t *dev, uint16_t *raw_angle) {
    if (!dev || !raw_angle) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    uint8_t reg = MT6701_REG_ANGLE_H;
    uint8_t buffer[2];
    // Read 2 consecutive bytes from registers 0x03 and 0x04 in a single I2C burst
    // transaction
    esp_err_t err =
        i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2, -1);

    if (err == ESP_OK) {
        // Reconstruct 14-bit angle: (reg_0x03 << 6) | (reg_0x04 >> 2)
        *raw_angle = (((uint16_t)buffer[0] << 6) | (buffer[1] >> 2)) & 0x3FFF;
    }

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return err;
}

float mt6701_counts_to_degrees(uint16_t angle_counts) {
    return (float)angle_counts * (360.0f / 16384.0f);
}

esp_err_t mt6701_update(mt6701_dev_t *dev) {
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw;
    esp_err_t err = mt6701_read_raw_angle(dev, &raw);
    if (err != ESP_OK) {
        return err;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    int64_t now = esp_timer_get_time();

    // Apply software offset and direction to raw reading
    uint16_t calibrated = (raw - dev->zero_offset) & 0x3FFF;
    if (dev->direction == MT6701_DIR_CCW) {
        calibrated = (16384 - calibrated) & 0x3FFF;
    }

    int64_t dt_us = now - dev->last_timestamp_us;

    // If the device timestamp was not initialized properly, skip velocity/turn
    // calculations
    if (dev->last_timestamp_us > 0 && dt_us > 0) {
        int32_t delta_angle =
            (int32_t)calibrated - (int32_t)dev->last_calibrated_angle;

        // Detect wrapping (crossover threshold at half-resolution = 8192 ticks)
        if (delta_angle < -8192) {
            delta_angle += 16384;
            dev->total_turns++;
        } else if (delta_angle > 8192) {
            delta_angle -= 16384;
            dev->total_turns--;
        }

        // Calculate angular velocity (radians per second)
        float dt_sec = (float)dt_us / 1000000.0f;
        float velocity_instant =
            ((float)delta_angle * (2.0f * M_PI / 16384.0f)) / dt_sec;

        // Filter high frequency jitter from velocity estimation
        dev->velocity_rad_s =
            (dev->velocity_filter_alpha * velocity_instant) +
            ((1.0f - dev->velocity_filter_alpha) * dev->velocity_rad_s);
    }

    dev->last_calibrated_angle = calibrated;
    dev->last_timestamp_us = now;

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_read_calibrated_angle_counts(mt6701_dev_t *dev,
                                              uint16_t *angle_counts) {
    if (!dev || !angle_counts) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw;
    esp_err_t err = mt6701_read_raw_angle(dev, &raw);
    if (err != ESP_OK) {
        return err;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    uint16_t calibrated = (raw - dev->zero_offset) & 0x3FFF;
    if (dev->direction == MT6701_DIR_CCW) {
        calibrated = (16384 - calibrated) & 0x3FFF;
    }
    *angle_counts = calibrated;

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_read_angle_degrees(mt6701_dev_t *dev, float *degrees) {
    if (!dev || !degrees) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t calibrated;
    esp_err_t err = mt6701_read_calibrated_angle_counts(dev, &calibrated);
    if (err == ESP_OK) {
        *degrees = mt6701_counts_to_degrees(calibrated);
    }
    return err;
}

esp_err_t mt6701_get_last_angle_degrees(mt6701_dev_t *dev, float *degrees) {
    if (!dev || !degrees) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    *degrees = mt6701_counts_to_degrees(dev->last_calibrated_angle);

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_get_last_angle_counts(mt6701_dev_t *dev,
                                       uint16_t *angle_counts) {
    if (!dev || !angle_counts) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    *angle_counts = dev->last_calibrated_angle;

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_read_angle_radians(mt6701_dev_t *dev, float *radians) {
    if (!dev || !radians) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t calibrated;
    esp_err_t err = mt6701_read_calibrated_angle_counts(dev, &calibrated);
    if (err == ESP_OK) {
        *radians = (float)calibrated * (2.0f * M_PI / 16384.0f);
    }
    return err;
}

esp_err_t mt6701_get_total_angle_radians(mt6701_dev_t *dev,
                                         float *total_radians) {
    if (!dev || !total_radians) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    float current_rad =
        (float)dev->last_calibrated_angle * (2.0f * M_PI / 16384.0f);
    *total_radians = ((float)dev->total_turns * 2.0f * M_PI) + current_rad;

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_get_total_turns(mt6701_dev_t *dev, int32_t *turns) {
    if (!dev || !turns) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    *turns = dev->total_turns;

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_get_velocity(mt6701_dev_t *dev, float *velocity) {
    if (!dev || !velocity) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    *velocity = dev->velocity_rad_s;

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_set_software_zero(mt6701_dev_t *dev) {
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw;
    esp_err_t err = mt6701_read_raw_angle(dev, &raw);
    if (err != ESP_OK) {
        return err;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    dev->zero_offset = raw;
    dev->total_turns = 0;
    dev->last_calibrated_angle = 0;
    dev->last_timestamp_us = esp_timer_get_time();
    dev->velocity_rad_s = 0.0f;

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return ESP_OK;
}

esp_err_t mt6701_set_software_direction(mt6701_dev_t *dev,
                                        mt6701_direction_t dir) {
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    dev->direction = dir;
    dev->total_turns = 0;
    dev->velocity_rad_s = 0.0f;

    // Reset reading variables to correctly align with the new direction
    uint16_t dummy_angle;
    // We unlock the mutex temporarily to prevent deadlocks as
    // mt6701_read_raw_angle will take it
#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    esp_err_t err = mt6701_read_raw_angle(dev, &dummy_angle);
#if CONFIG_MT6701_THREAD_SAFE
    if (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
#endif

    if (err == ESP_OK) {
        uint16_t calibrated = (dummy_angle - dev->zero_offset) & 0x3FFF;
        if (dev->direction == MT6701_DIR_CCW) {
            calibrated = (16384 - calibrated) & 0x3FFF;
        }
        dev->last_calibrated_angle = calibrated;
        dev->last_timestamp_us = esp_timer_get_time();
    }

#if CONFIG_MT6701_THREAD_SAFE
    xSemaphoreGive(dev->lock);
#endif
    return err;
}
