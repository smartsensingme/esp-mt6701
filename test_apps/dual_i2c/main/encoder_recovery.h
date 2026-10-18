#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Acquisition-owned history survives reporting boundaries. A recovery is the
 * first successful read after failures, not necessarily valid velocity. */
typedef struct {
  uint32_t consecutive;
  int64_t first_failure_us;
} encoder_recovery_t;

typedef struct {
  uint32_t recoveries, isolated, bursts, max_consecutive;
  uint64_t max_recovery_us;
} encoder_recovery_window_t;

static inline void encoder_recovery_record(encoder_recovery_t *history,
                                           encoder_recovery_window_t *window,
                                           bool valid, int64_t timestamp_us) {
  if (!valid) {
    if (history->consecutive == 0)
      history->first_failure_us = timestamp_us;
    if (history->consecutive != UINT32_MAX)
      history->consecutive++;
  } else if (history->consecutive) {
    window->recoveries++;
    if (history->consecutive == 1)
      window->isolated++;
    else
      window->bursts++;
    uint64_t duration = (uint64_t)(timestamp_us - history->first_failure_us);
    if (duration > window->max_recovery_us)
      window->max_recovery_us = duration;
  }
  if (history->consecutive > window->max_consecutive)
    window->max_consecutive = history->consecutive;
  if (valid)
    history->consecutive = 0;
}
