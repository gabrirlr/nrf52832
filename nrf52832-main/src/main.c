/*
 * BLE Lock Service (Zephyr)
 *
 * Serviço 0x1523 (128-bit base UUID)
 *  - Char 0x1524: STATUS  (READ + NOTIFY)  -> App se inscreve (CCC) e recebe "LOCKED"/"UNLOCKED"
 *  - Char 0x1525: CTRL    (WRITE/WNR)      -> App envia comandos (ex: "LOCK", "UNLOCK", "TOGGLE")
 *  - Char 0x1526: TIME_SYNC (WRITE/WNR)    -> App envia epoch_ms (int64 LE) para sync de tempo
 *
 * Hardware (DeviceTree overlay):
 *  - led0: LED de “run” (pisca 1 Hz)
 *  - led1: GPIO que representa “lock” (0/1)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include <zephyr/sys/byteorder.h>

#include <string.h>
#include <time.h>

#include "timekeeper.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ============================================================================
 * Protótipos
 * ========================================================================== */
static void update_lock(uint8_t locked);
static void format_epoch_ms_brt(int64_t epoch_ms, char *out, size_t out_sz);

static void auto_lock_timer_handler(struct k_timer *dummy);

static void lock_notify_work_handler(struct k_work *work);
static void notify_status(void);

/* ============================================================================
 * 1) UUIDs e Globais
 * ========================================================================== */
#define UUID_SERVICE           BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define UUID_LOCK_STATUS_VAL   BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define UUID_LOCK_CTRL_VAL     BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define UUID_TIME_SYNC_VAL     BT_UUID_128_ENCODE(0x00001526, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

static struct bt_uuid_128 svc_uuid       = BT_UUID_INIT_128(UUID_SERVICE);
static struct bt_uuid_128 status_uuid    = BT_UUID_INIT_128(UUID_LOCK_STATUS_VAL);
static struct bt_uuid_128 ctrl_uuid      = BT_UUID_INIT_128(UUID_LOCK_CTRL_VAL);
static struct bt_uuid_128 time_sync_uuid = BT_UUID_INIT_128(UUID_TIME_SYNC_VAL);

static uint8_t lock_state = 0;
static bool lock_notify_enabled = false;

static char lock_status_buf[10] = "UNLOCKED";
static struct bt_conn *current_conn = NULL;

/* Work item para notificar sem travar stack BT */
static struct k_work lock_notify_work;

/* GPIOs via aliases do overlay */
static const struct gpio_dt_spec run_led   = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec lock_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});

/* Timer de auto-lock */
K_TIMER_DEFINE(auto_lock_timer, auto_lock_timer_handler, NULL);

/* ============================================================================
 * 2) Auxiliares
 * ========================================================================== */
static void str_to_upper(char *s)
{
    for (int i = 0; s[i]; i++) {
        if (s[i] >= 'a' && s[i] <= 'z') {
            s[i] -= 32;
        }
    }
}

static void format_epoch_ms_brt(int64_t epoch_ms, char *out, size_t out_sz)
{
    if (!out || out_sz == 0) return;

    const int32_t tz_off_sec = -3 * 3600; /* Fortaleza = UTC-3 */
    int64_t adj_ms = epoch_ms + (int64_t)tz_off_sec * 1000;

    time_t sec = (time_t)(adj_ms / 1000);
    int32_t msec = (int32_t)(adj_ms % 1000);
    if (msec < 0) msec = -msec;

    struct tm tm_brt;
    gmtime_r(&sec, &tm_brt);

    /* Formata TUDO de uma vez diretamente no buffer de saída 'out'.
       Isso elimina o risco de truncamento intermediário e economiza RAM. */
    snprintk(out, out_sz,
             "%04d-%02d-%02d %02d:%02d:%02d.%03d BRT",
             tm_brt.tm_year + 1900,
             tm_brt.tm_mon + 1,
             tm_brt.tm_mday,
             tm_brt.tm_hour,
             tm_brt.tm_min,
             tm_brt.tm_sec,
             (int)msec);
}

/* ============================================================================
 * 3) Notify de STATUS (1524)
 * ========================================================================== */
static void notify_status(void)
{
    if (!current_conn || !lock_notify_enabled) {
        return;
    }

    int err = bt_gatt_notify_uuid(current_conn, &status_uuid.uuid, NULL,
                                  lock_status_buf, strlen(lock_status_buf));

    LOG_INF(">>> Notify STATUS (%s) err=%d", lock_status_buf, err);
}

static void lock_notify_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    notify_status();
}

/* ============================================================================
 * 4) Auto-lock + update_lock
 * ========================================================================== */
static void auto_lock_timer_handler(struct k_timer *dummy)
{
    ARG_UNUSED(dummy);
    LOG_INF("[AUTO-LOCK] Tempo esgotado. Trancando automaticamente...");
    update_lock(1);
}

static void update_lock(uint8_t locked)
{
    lock_state = locked ? 1 : 0;

    if (lock_gpio.port) {
        gpio_pin_set_dt(&lock_gpio, lock_state);
    }

    strcpy(lock_status_buf, lock_state ? "LOCKED" : "UNLOCKED");

    /* Se destravou -> inicia auto-lock. Se travou -> para */
    if (lock_state == 0) {
        k_timer_start(&auto_lock_timer, K_SECONDS(15), K_NO_WAIT);
        LOG_INF("Auto-Lock: Porta UNLOCKED. Travando em 15s.");
    } else {
        k_timer_stop(&auto_lock_timer);
    }

    /* Auditoria com timestamp (se tiver tempo válido) */
    int64_t now;
    if (timekeeper_now_epoch_ms(&now) == 0) {
        char ts[64];
        format_epoch_ms_brt(now, ts, sizeof(ts));
        LOG_INF(">>> [AUDITORIA] %s em %s", lock_status_buf, ts);
    } else {
        LOG_WRN(">>> [AUDITORIA] %s (relogio ainda invalido)", lock_status_buf);
    }

    k_work_submit(&lock_notify_work);
}

/* ============================================================================
 * 5) Callbacks GATT
 * ========================================================================== */
static ssize_t on_read_status(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    ARG_UNUSED(conn);
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             lock_status_buf, strlen(lock_status_buf));
}

static ssize_t on_write_ctrl(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (offset != 0 || len == 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    char cmd[16] = {0};
    memcpy(cmd, buf, MIN(len, sizeof(cmd) - 1));
    str_to_upper(cmd);

    LOG_INF("CTRL (1525) recebeu: '%s'", cmd);

    if (strstr(cmd, "TOGGLE")) {
        update_lock(!lock_state);
    } else if (strstr(cmd, "UNLOCK")) {
        update_lock(0);
    } else if (strstr(cmd, "LOCK")) {
        update_lock(1);
    } else {
        LOG_WRN("Comando desconhecido -> TOGGLE fallback");
        update_lock(!lock_state);
    }

    return len;
}

/***************/

static int64_t days_from_civil(int32_t y, uint32_t m, uint32_t d)
{
    y -= (m <= 2);
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const uint32_t yoe = (uint32_t)(y - era * 400);                // [0, 399]
    const uint32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1; // [0, 365]
    const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;     // [0, 146096]
    return (int64_t)era * 146097 + (int64_t)doe - 719468;           // 719468 = dias até 1970-01-01
}

static bool is_leap(int y)
{
    return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

static bool validate_ymdhms(int y, int mon, int day, int h, int min, int s)
{
    if (y < 2000 || y > 2099) return false;
    if (mon < 1 || mon > 12) return false;
    if (h < 0 || h > 23) return false;
    if (min < 0 || min > 59) return false;
    if (s < 0 || s > 59) return false;

    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int dim = mdays[mon - 1];
    if (mon == 2 && is_leap(y)) dim = 29;
    if (day < 1 || day > dim) return false;

    return true;
}

/* Converte data/hora em UTC para epoch_ms */
static int64_t ymdhms_to_epoch_ms_utc(int y, int mon, int day, int h, int min, int sec)
{
    int64_t days = days_from_civil(y, (uint32_t)mon, (uint32_t)day);
    int64_t t = days * 86400LL + (int64_t)h * 3600LL + (int64_t)min * 60LL + (int64_t)sec;
    return t * 1000LL;
}

/***************/

static ssize_t on_write_time_sync(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Agora esperamos 7 bytes no formato do app */
    if (len != 7) {
        LOG_WRN("TIME_SYNC: tamanho invalido (%u). Esperado 7 bytes (yy mm dd hh mm ss wd).", len);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *p = (const uint8_t *)buf;

    int year   = 2000 + p[0];
    int month  = p[1];
    int day    = p[2];
    int hour   = p[3];
    int minute = p[4];
    int second = p[5];
    int wday   = p[6]; /* Dart: 1..7 (Mon..Sun) - vamos só logar/ignorar */

    if (!validate_ymdhms(year, month, day, hour, minute, second)) {
        LOG_WRN("TIME_SYNC: data/hora invalida: %04d-%02d-%02d %02d:%02d:%02d (wday=%d)",
                year, month, day, hour, minute, second, wday);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* O app manda hora LOCAL (BRT). Seu timekeeper armazena UTC.
       Então convertemos BRT -> UTC somando 3 horas. */
    const int32_t brt_off_sec = -3 * 3600; /* BRT = UTC-3 */
    int64_t epoch_ms_local = ymdhms_to_epoch_ms_utc(year, month, day, hour, minute, second);
    int64_t epoch_ms_utc   = epoch_ms_local - ((int64_t)brt_off_sec * 1000LL);
    /* Como brt_off_sec é negativo, "subtrair negativo" = somar 3h */

    LOG_INF("TIME_SYNC(7B): local=%04d-%02d-%02d %02d:%02d:%02d (wday=%d) -> epoch_ms_utc=%lld",
            year, month, day, hour, minute, second, wday, (long long)epoch_ms_utc);

    int err = timekeeper_validate_and_sync_ms(epoch_ms_utc, 2000);
    if (err) {
        LOG_WRN("TIME_SYNC: sync falhou err=%d", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    k_work_submit(&lock_notify_work);
    return len;
}

static void on_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    lock_notify_enabled = (value & BT_GATT_CCC_NOTIFY) != 0;

    LOG_INF("Notificacoes STATUS: %s (raw=0x%04x)",
            lock_notify_enabled ? "ATIVAS" : "INATIVAS", value);

    if (lock_notify_enabled) {
        k_work_submit(&lock_notify_work);
    }
}

/* ============================================================================
 * 6) Serviço GATT (1523)
 * ========================================================================== */
BT_GATT_SERVICE_DEFINE(lock_svc,
    BT_GATT_PRIMARY_SERVICE(&svc_uuid),

    BT_GATT_CHARACTERISTIC(&status_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        on_read_status, NULL, NULL),

    BT_GATT_CCC(on_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&ctrl_uuid.uuid,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, on_write_ctrl, NULL),

    BT_GATT_CHARACTERISTIC(&time_sync_uuid.uuid,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, on_write_time_sync, NULL),
);

/* ============================================================================
 * 7) Advertising
 * ========================================================================== */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, UUID_SERVICE),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

/* ============================================================================
 * 8) Conexão BLE
 * ========================================================================== */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Falha ao conectar (err=%u)", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    LOG_INF("Conectado!");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    lock_notify_enabled = false;
    LOG_INF("Desconectado (reason=%u)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ============================================================================
 * 9) Bluetooth ready
 * ========================================================================== */
static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("bt_enable falhou: %d", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    struct bt_le_adv_param adv_param = *BT_LE_ADV_PARAM(
        BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
        BT_GAP_ADV_FAST_INT_MIN_2,
        BT_GAP_ADV_FAST_INT_MAX_2,
        NULL
    );

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    LOG_INF("Advertising ON (err=%d) - pronto para comandos!", err);
}

/* ============================================================================
 * 10) main()
 * ========================================================================== */
int main(void)
{
    k_work_init(&lock_notify_work, lock_notify_work_handler);

    int err = timekeeper_init();
    LOG_INF("timekeeper_init err=%d valid=%d", err, timekeeper_is_valid());

    if (run_led.port) {
        gpio_pin_configure_dt(&run_led, GPIO_OUTPUT_ACTIVE);
    }
    if (lock_gpio.port) {
        gpio_pin_configure_dt(&lock_gpio, GPIO_OUTPUT_INACTIVE);
    }

    bt_enable(bt_ready);

    while (1) {
        if (run_led.port) {
            gpio_pin_toggle_dt(&run_led);
        }

        int64_t now_ms;
        int e = timekeeper_now_epoch_ms(&now_ms);
        if (e == 0) {
            char ts[80];
            format_epoch_ms_brt(now_ms, ts, sizeof(ts));
            LOG_INF("Agora: %s", ts);
        } else {
            LOG_WRN("Relogio invalido (ainda sem TIME_SYNC). err=%d", e);
        }

        k_sleep(K_SECONDS(1));
    }

    return 0;
}
