#ifndef TIMEKEEPER_H_
#define TIMEKEEPER_H_

#include <stdint.h>
#include <stdbool.h>

int timekeeper_init(void);
bool timekeeper_is_valid(void);
int timekeeper_now_epoch_ms(int64_t *epoch_ms_out);
int timekeeper_sync_epoch_ms(int64_t phone_epoch_ms);
int timekeeper_validate_and_sync_ms(int64_t phone_epoch_ms, int64_t threshold_ms);

#endif