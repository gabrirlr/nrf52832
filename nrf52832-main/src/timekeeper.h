#ifndef TIMEKEEPER_H  // Deve ser #ifndef (if not defined)
#define TIMEKEEPER_H
#pragma once
#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

int  timekeeper_init(void);
bool timekeeper_is_valid(void);

int  timekeeper_now_epoch_ms(int64_t *epoch_ms_out);

/* chama quando receber do celular */
int  timekeeper_sync_epoch_ms(int64_t phone_epoch_ms);

/* valida e só sincroniza se drift > threshold */
int  timekeeper_validate_and_sync_ms(int64_t phone_epoch_ms, int64_t threshold_ms);

#endif
