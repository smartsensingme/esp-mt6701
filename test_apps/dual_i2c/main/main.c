#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "encoder_recovery.h"
#include "esp_log.h"
#include "esp_rt_diagnostics.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mt6701.h"
#include "sdkconfig.h"

static TaskHandle_t owner;
static volatile uint32_t release_timestamp;
typedef struct {
  uint32_t attempts[2], errors[2], raw[2];
  float speed[2];
  bool valid[2], speed_valid[2];
  esp_err_t last_error[2];
  uint32_t inferred_missed;
  encoder_recovery_window_t recovery[2];
  uint32_t consecutive[2];
} encoder_window_t;

static bool on_release(gptimer_handle_t timer,
                       const gptimer_alarm_event_data_t *event, void *arg) {
  (void)timer;
  (void)event;
  (void)arg;
  esp_rt_diag_isr_capture(&release_timestamp);
  BaseType_t wake = pdFALSE;
  vTaskNotifyGiveFromISR(owner, &wake);
  return wake == pdTRUE;
}

static void report_encoders(const esp_rt_diag_snapshot_t *diag,
                            const void *payload) {
  const encoder_window_t *w = payload;
  const unsigned addresses[] = {CONFIG_MT6701_TEST_LEFT_ADDRESS,
                                CONFIG_MT6701_TEST_RIGHT_ADDRESS};
  const char *sides[] = {"left", "right"};
  for (unsigned i = 0; i < 2; i++)
    ESP_LOGI("encoders",
             "window=%llu duration_ms=%lu side=%s sensor=0x%02x "
             "valid_reads=%lu errors=%lu attempts=%lu last_error=%s "
             "latest_valid=%d raw=%lu speed_valid=%d speed=%.3f rad/s "
             "inferred_missed=%lu consecutive=%lu max_consecutive=%lu "
             "recoveries=%lu isolated=%lu bursts=%lu max_recovery_us=%llu",
             (unsigned long long)diag->window_id,
             (unsigned long)(diag->window_duration_us / 1000), sides[i],
             addresses[i], (unsigned long)(w->attempts[i] - w->errors[i]),
             (unsigned long)w->errors[i], (unsigned long)w->attempts[i],
             esp_err_to_name(w->last_error[i]), w->valid[i],
             (unsigned long)w->raw[i], w->speed_valid[i], (double)w->speed[i],
             (unsigned long)w->inferred_missed,
             (unsigned long)w->consecutive[i],
             (unsigned long)w->recovery[i].max_consecutive,
             (unsigned long)w->recovery[i].recoveries,
             (unsigned long)w->recovery[i].isolated,
             (unsigned long)w->recovery[i].bursts,
             (unsigned long long)w->recovery[i].max_recovery_us);
}

void app_main(void) {
  ESP_ERROR_CHECK(CONFIG_MT6701_TEST_SDA_GPIO != CONFIG_MT6701_TEST_SCL_GPIO &&
                          CONFIG_MT6701_TEST_LEFT_ADDRESS !=
                              CONFIG_MT6701_TEST_RIGHT_ADDRESS
                      ? ESP_OK
                      : ESP_ERR_INVALID_ARG);
  const unsigned addresses[2] = {CONFIG_MT6701_TEST_LEFT_ADDRESS,
                                 CONFIG_MT6701_TEST_RIGHT_ADDRESS};
  const char *sides[2] = {"left", "right"};
  i2c_master_bus_handle_t bus = NULL;
  i2c_master_dev_handle_t devices[2] = {NULL, NULL};
  mt6701_dev_t sensors[2] = {0};
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = CONFIG_MT6701_TEST_SDA_GPIO,
      .scl_io_num = CONFIG_MT6701_TEST_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true};
  esp_err_t error = i2c_new_master_bus(&bus_config, &bus);
  if (error != ESP_OK) {
    ESP_LOGE("encoders", "bus=shared setup_error=%s", esp_err_to_name(error));
    return;
  }
  for (unsigned i = 0; i < 2; i++) {
    i2c_device_config_t config = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                  .device_address = addresses[i],
                                  .scl_speed_hz = CONFIG_MT6701_TEST_I2C_HZ};
    error = i2c_master_bus_add_device(bus, &config, &devices[i]);
    if (error != ESP_OK) {
      ESP_LOGE("encoders", "side=%s sensor=0x%02x setup_error=%s", sides[i],
               addresses[i], esp_err_to_name(error));
      for (unsigned j = 0; j < i; j++)
        i2c_master_bus_rm_device(devices[j]);
      i2c_del_master_bus(bus);
      return;
    }
  }
  ESP_LOGI("encoders",
           "bus=shared SDA=%d SCL=%d; left=0x%02x right=0x%02x; "
           "bridge not initialized",
           CONFIG_MT6701_TEST_SDA_GPIO, CONFIG_MT6701_TEST_SCL_GPIO,
           addresses[0], addresses[1]);
  /* Initialize before periodic work. A communication failure leaves the
   * registered handle/timeout usable for subsequent independent raw reads. */
  for (unsigned i = 0; i < 2; i++) {
    error = mt6701_init_with_timeout(&sensors[i], devices[i], 1);
    ESP_LOGI("encoders", "side=%s sensor=0x%02x init=%s", sides[i],
             addresses[i], esp_err_to_name(error));
    if (error == ESP_ERR_INVALID_ARG || error == ESP_ERR_NO_MEM)
      ESP_ERROR_CHECK(error);
  }
  static esp_rt_diag_t diagnostics;
  static esp_rt_diag_snapshot_t snapshot;
  static encoder_window_t window;
  esp_rt_diag_config_t diag_config = {.expected_period_us = 5000,
                                      .deadline_us = 5000,
                                      .window_duration_us = 60000000,
                                      .stage_count = 2,
                                      .event_count = 2,
                                      .interval_count = 2,
                                      .stage_budget_us = {1000, 1000}};
  owner = xTaskGetCurrentTaskHandle();
  vTaskPrioritySet(owner, 8);
  gptimer_handle_t timer;
  gptimer_config_t timer_config = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
                                   .direction = GPTIMER_COUNT_UP,
                                   .resolution_hz = 1000000};
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));
  uint32_t resolution;
  ESP_ERROR_CHECK(gptimer_get_resolution(timer, &resolution));
  ESP_ERROR_CHECK(resolution == 1000000 ? ESP_OK : ESP_ERR_INVALID_STATE);
  gptimer_event_callbacks_t callbacks = {.on_alarm = on_release};
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &callbacks, NULL));
  gptimer_alarm_config_t alarm = {.alarm_count = 5000,
                                  .reload_count = 0,
                                  .flags.auto_reload_on_alarm = true};
  ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
  ESP_ERROR_CHECK(gptimer_enable(timer));
  ESP_LOGI("encoders", "test=quiet_60s; no acquisition logs until completion; "
                       "bridge not initialized");
  /* Allow startup output to drain before the quiet measurement begins. */
  vTaskDelay(pdMS_TO_TICKS(1000));
  int64_t started = esp_timer_get_time();
  ESP_ERROR_CHECK(esp_rt_diag_init(&diagnostics, &diag_config, started));
  int64_t next_due = started + 5000;
  ESP_ERROR_CHECK(gptimer_start(timer));
  int64_t previous_time[2] = {0};
  uint16_t previous_raw[2] = {0};
  encoder_recovery_t recovery_history[2] = {0};
  for (;;) {
    uint32_t notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    esp_rt_diag_cycle_begin_from_isr(&diagnostics, &release_timestamp,
                                     notifications);
    int64_t now = esp_timer_get_time();
    uint32_t elapsed = now > next_due ? 1 + (now - next_due) / 5000 : 1;
    if (elapsed > notifications)
      window.inferred_missed += elapsed - notifications;
    next_due += (int64_t)elapsed * 5000;
    /* One acquisition per wake; accumulated releases are counted, never
     * replayed as unlimited catch-up reads. No driver logs in this path. */
    for (unsigned i = 0; i < 2; i++) {
      int64_t stage = esp_rt_diag_stage_begin();
      int64_t start = esp_timer_get_time();
      uint16_t raw;
      error = mt6701_read_raw_angle(&sensors[i], &raw);
      int64_t end = esp_timer_get_time();
      int64_t timestamp = start + (end - start) / 2;
      esp_rt_diag_stage_end(&diagnostics, i, stage);
      window.attempts[i]++;
      window.valid[i] = error == ESP_OK;
      encoder_recovery_record(&recovery_history[i], &window.recovery[i],
                              error == ESP_OK, end);
      window.consecutive[i] = recovery_history[i].consecutive;
      window.speed_valid[i] = false;
      if (error == ESP_OK) {
        window.raw[i] = raw;
        if (previous_time[i]) {
          int64_t dt = timestamp - previous_time[i];
          esp_rt_diag_interval(&diagnostics, i, (uint32_t)dt);
          int delta = (int)raw - previous_raw[i];
          if (delta > 8192)
            delta -= 16384;
          if (delta < -8192)
            delta += 16384;
          if (dt > 0 && dt <= 10000 && delta != 8192 && delta != -8192) {
            window.speed[i] =
                delta * (6.28318530718f / 16384.0f) * (1000000.0f / (float)dt);
            window.speed_valid[i] = true;
          }
        }
        previous_time[i] = timestamp;
        previous_raw[i] = raw;
      } else {
        window.errors[i]++;
        window.last_error[i] = error;
        previous_time[i] = 0;
        esp_rt_diag_event(&diagnostics, i, 1);
      }
    }
    int64_t end = esp_rt_diag_cycle_end_now(&diagnostics);
    if (end - started >= 60000000) {
      /* End the operating phase before any snapshot formatting or output. */
      ESP_ERROR_CHECK(gptimer_stop(timer));
      ESP_ERROR_CHECK(gptimer_disable(timer));
      ESP_ERROR_CHECK(gptimer_del_timer(timer));
      ESP_ERROR_CHECK(esp_rt_diag_take_snapshot(&diagnostics, end, &snapshot));
      vTaskPrioritySet(owner, 1);
      ESP_LOGI("encoders",
               "test=quiet_60s complete; acquisition stopped; "
               "cycles=%lu missed=%lu inferred_missed=%lu overruns=%lu "
               "max_wake_us=%lu max_processing_us=%lu max_cycle_us=%lu",
               (unsigned long)snapshot.cycles,
               (unsigned long)snapshot.missed_events,
               (unsigned long)window.inferred_missed,
               (unsigned long)snapshot.deadline_overruns,
               (unsigned long)snapshot.max_wake_latency_us,
               (unsigned long)snapshot.max_processing_time_us,
               (unsigned long)snapshot.max_cycle_time_us);
      report_encoders(&snapshot, &window);
      for (unsigned i = 0; i < 2; i++) {
        ESP_LOGI("encoders",
                 "side=%s sensor=0x%02x max_i2c_us=%lu "
                 "sample_interval_min_us=%lu sample_interval_max_us=%lu",
                 sides[i], addresses[i],
                 (unsigned long)snapshot.stages[i].max_duration_us,
                 (unsigned long)snapshot.intervals[i].min_us,
                 (unsigned long)snapshot.intervals[i].max_us);
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(devices[i]));
      }
      ESP_ERROR_CHECK(i2c_del_master_bus(bus));
      return;
    }
  }
}
