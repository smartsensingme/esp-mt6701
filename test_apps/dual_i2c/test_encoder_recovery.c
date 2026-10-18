#include "encoder_recovery.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
  encoder_recovery_t history = {0};
  encoder_recovery_window_t window = {0};
  encoder_recovery_record(&history, &window, true, 100);
  assert(window.recoveries == 0);
  encoder_recovery_record(&history, &window, false, 5000);
  encoder_recovery_record(&history, &window, true, 10000);
  assert(window.isolated == 1 && window.recoveries == 1);
  assert(window.max_consecutive == 1 && window.max_recovery_us == 5000);
  encoder_recovery_record(&history, &window, false, 15000);
  encoder_recovery_record(&history, &window, false, 20000);
  /* A sequence crosses the window: only its recovery belongs to the next. */
  window = (encoder_recovery_window_t){0};
  encoder_recovery_record(&history, &window, false, 25000);
  encoder_recovery_record(&history, &window, true, 30000);
  assert(window.bursts == 1 && window.isolated == 0);
  assert(window.max_consecutive == 3 && window.max_recovery_us == 15000);
  assert(history.consecutive == 0);
  window = (encoder_recovery_window_t){0};
  encoder_recovery_record(&history, &window, true, 35000);
  assert(window.max_consecutive == 0 && window.recoveries == 0);
  history.consecutive = UINT32_MAX;
  encoder_recovery_record(&history, &window, false, 40000);
  assert(history.consecutive == UINT32_MAX);
  puts("Encoder isolated/burst/cross-window recovery tests passed");
}
