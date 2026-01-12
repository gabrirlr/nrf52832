#include "timekeeper.h"
#include <zephyr/settings/settings.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <errno.h>

#define SETTINGS_KEY_VALID   "clock/valid"
#define SETTINGS_KEY_EPOCH   "clock/epoch_ms"

static bool    g_valid = false;
static int64_t g_epoch_ms = 0;   /* persistido */
static int64_t g_off_ms   = 0;   /* derivado */

static bool loaded_valid = false;
static bool loaded_epoch = false;
static bool legacy_off_ms_found = false;

static int64_t mono_ms(void)
{
    return (int64_t)k_uptime_get();
}

static void recompute_offset(void)
{
    g_off_ms = g_epoch_ms - mono_ms();
}

/* <<< NÃO CHAME isso de settings_commit >>> */
static int clock_settings_commit(void)
{
    /* Se achou schema antigo, limpa e força resync */
    if (legacy_off_ms_found) {
        (void)settings_delete("clock/off_ms");
        (void)settings_delete("clock/valid");

        g_valid = false;
        g_epoch_ms = 0;
        g_off_ms = 0;
        return 0;
    }

    /* Só considera válido se tiver valid + epoch_ms e valid==true */
    if (!(loaded_valid && loaded_epoch && g_valid)) {
        g_valid = false;
        g_epoch_ms = 0;
        g_off_ms = 0;
        return 0;
    }

    recompute_offset();
    return 0;
}

static int clock_settings_set(const char *key, size_t len,
                              settings_read_cb read_cb, void *cb_arg)
{
    ARG_UNUSED(len);
    ssize_t rc;

    if (!strcmp(key, "valid")) {
        rc = read_cb(cb_arg, &g_valid, sizeof(g_valid));
        if (rc >= 0) loaded_valid = true;
        return (rc >= 0) ? 0 : rc;
    }

    if (!strcmp(key, "epoch_ms")) {
        rc = read_cb(cb_arg, &g_epoch_ms, sizeof(g_epoch_ms));
        if (rc >= 0) loaded_epoch = true;
        return (rc >= 0) ? 0 : rc;
    }

    /* legado: só marca que existe e consome os bytes */
    if (!strcmp(key, "off_ms")) {
        int64_t dummy;
        rc = read_cb(cb_arg, &dummy, sizeof(dummy));
        if (rc >= 0) legacy_off_ms_found = true;
        return (rc >= 0) ? 0 : rc;
    }

    return -ENOENT;
}

static struct settings_handler sh = {
    .name = "clock",
    .h_set = clock_settings_set,
    .h_commit = clock_settings_commit,
};

int timekeeper_init(void)
{
    /* zera flags a cada boot */
    loaded_valid = false;
    loaded_epoch = false;
    legacy_off_ms_found = false;

    int err = settings_subsys_init();
    if (err && err != -EALREADY) {
        return err;
    }

    return settings_register(&sh);
}

bool timekeeper_is_valid(void)
{
    return g_valid;
}

int timekeeper_now_epoch_ms(int64_t *epoch_ms_out)
{
    if (!epoch_ms_out) return -EINVAL;
    if (!g_valid) return -ENODATA;

    *epoch_ms_out = mono_ms() + g_off_ms;
    return 0;
}

int timekeeper_sync_epoch_ms(int64_t phone_epoch_ms)
{
    g_epoch_ms = phone_epoch_ms;
    g_valid = true;
    recompute_offset();

    int err = settings_save_one(SETTINGS_KEY_EPOCH, &g_epoch_ms, sizeof(g_epoch_ms));
    if (err) return err;

    return settings_save_one(SETTINGS_KEY_VALID, &g_valid, sizeof(g_valid));
}

int timekeeper_validate_and_sync_ms(int64_t phone_epoch_ms, int64_t threshold_ms)
{
    int64_t now;
    int err = timekeeper_now_epoch_ms(&now);

    if (err == 0) {
        int64_t delta = phone_epoch_ms - now;
        if (delta < 0) delta = -delta;
        if (delta <= threshold_ms) return 0;
    }

    return timekeeper_sync_epoch_ms(phone_epoch_ms);
}
