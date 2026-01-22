/*
 * BLE Lock Service (Zephyr) - Versão Moto Segura (Sem PWM)
 * Inclui: Persistência, Sincronização de Tempo, Auditoria e Ignição Segura (P0.04)
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
#include <errno.h>

#include "timekeeper.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ============================================================================
 * Protótipos
 * ========================================================================== */
static void update_lock(uint8_t locked);
static void format_epoch_ms_brt(int64_t epoch_ms, char *out, size_t out_sz);
static void auto_lock_timer_handler(struct k_timer *dummy);
static void lock_notify_work_handler(struct k_work *work);
static void start_advertising(void);

/* ============================================================================
 * 1) UUIDs e Globais
 * ========================================================================== */
#define UUID_SERVICE         BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define UUID_LOCK_STATUS_VAL BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define UUID_LOCK_CTRL_VAL   BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define UUID_TIME_SYNC_VAL   BT_UUID_128_ENCODE(0x00001526, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

static struct bt_uuid_128 svc_uuid       = BT_UUID_INIT_128(UUID_SERVICE);
static struct bt_uuid_128 status_uuid    = BT_UUID_INIT_128(UUID_LOCK_STATUS_VAL);
static struct bt_uuid_128 ctrl_uuid      = BT_UUID_INIT_128(UUID_LOCK_CTRL_VAL);
static struct bt_uuid_128 time_sync_uuid = BT_UUID_INIT_128(UUID_TIME_SYNC_VAL);

static uint8_t lock_state = 1; 
static bool lock_notify_enabled = false;
static char lock_status_buf[32] = "LOCKED";
static struct bt_conn *current_conn = NULL;

static struct k_work lock_notify_work;
static struct k_work adv_restart_work;

static const struct gpio_dt_spec run_led   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec lock_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec ign_gpio  = GPIO_DT_SPEC_GET(DT_ALIAS(ignition), gpios);
static struct gpio_callback ign_cb_data;

K_TIMER_DEFINE(auto_lock_timer, auto_lock_timer_handler, NULL);

/* ============================================================================
 * 2) Persistência (Settings)
 * ========================================================================== */
#define SETTINGS_KEY "app/lock_state"

static int lock_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (strcmp(name, "lock_state") == 0) {
        if (len != sizeof(lock_state)) return -EINVAL;
        read_cb(cb_arg, &lock_state, sizeof(lock_state));
        LOG_INF("[SETTINGS] Estado carregado: %s", lock_state ? "LOCKED" : "UNLOCKED");
        
        if (lock_gpio.port) gpio_pin_set_dt(&lock_gpio, !lock_state);
        strcpy(lock_status_buf, lock_state ? "LOCKED" : "UNLOCKED");
        return 0;
    }
    return -ENOENT;
}

static struct settings_handler lock_conf = { .name = "app", .h_set = lock_settings_set };

/* ============================================================================
 * 3) Auxiliares e Tempo (Conversões originais preservadas)
 * ========================================================================== */
static void str_to_upper(char *s) {
    for (int i = 0; s[i]; i++) if (s[i] >= 'a' && s[i] <= 'z') s[i] -= 32;
}

static void format_epoch_ms_brt(int64_t epoch_ms, char *out, size_t out_sz) {
    if (!out || out_sz == 0) return;
    const int32_t tz_off_sec = -3 * 3600;
    int64_t adj_ms = epoch_ms + (int64_t)tz_off_sec * 1000;
    time_t sec = (time_t)(adj_ms / 1000);
    int32_t msec = (int32_t)(adj_ms % 1000);
    if (msec < 0) msec = -msec;
    struct tm tm_brt;
    gmtime_r(&sec, &tm_brt);
    snprintk(out, out_sz, "%04d-%02d-%02d %02d:%02d:%02d.%03d BRT",
             tm_brt.tm_year + 1900, tm_brt.tm_mon + 1, tm_brt.tm_mday,
             tm_brt.tm_hour, tm_brt.tm_min, tm_brt.tm_sec, (int)msec);
}

static int64_t days_from_civil(int32_t y, uint32_t m, uint32_t d) {
    y -= (m <= 2);
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const uint32_t yoe = (uint32_t)(y - era * 400);
    const uint32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static bool is_leap(int y) { return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0); }

static bool validate_ymdhms(int y, int mon, int day, int h, int min, int s) {
    if (y < 2000 || y > 2099 || mon < 1 || mon > 12 || h < 0 || h > 23 || min < 0 || min > 59 || s < 0 || s > 59) return false;
    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int dim = mdays[mon - 1];
    if (mon == 2 && is_leap(y)) dim = 29;
    return (day >= 1 && day <= dim);
}

static int64_t ymdhms_to_epoch_ms_utc(int y, int mon, int day, int h, int min, int sec) {
    int64_t days = days_from_civil(y, (uint32_t)mon, (uint32_t)day);
    return (days * 86400LL + (int64_t)h * 3600LL + (int64_t)min * 60LL + (int64_t)sec) * 1000LL;
}

/* ============================================================================
 * 4) Lógica de Ignição e Bloqueio (Segurança no P0.04)
 * ========================================================================== */
static void update_lock(uint8_t locked) {
    // Bloqueia travamento se a ignição estiver ativa (nível alto no P0.04)
    if (locked && gpio_pin_get_dt(&ign_gpio) > 0) {
        LOG_WRN("Travamento negado: Ignicao ligada!");
        return;
    }

    lock_state = locked ? 1 : 0;
    settings_save_one(SETTINGS_KEY, &lock_state, sizeof(lock_state));

    if (lock_gpio.port) gpio_pin_set_dt(&lock_gpio, !lock_state);
    strcpy(lock_status_buf, lock_state ? "LOCKED" : "UNLOCKED");

    if (lock_state == 0 && gpio_pin_get_dt(&ign_gpio) == 0) {
        k_timer_start(&auto_lock_timer, K_SECONDS(15), K_NO_WAIT);
    } else {
        k_timer_stop(&auto_lock_timer);
    }

    int64_t now;
    if (timekeeper_now_epoch_ms(&now) == 0) {
        char ts[64];
        format_epoch_ms_brt(now, ts, sizeof(ts));
        LOG_INF(">>> [AUDITORIA] %s em %s", lock_status_buf, ts);
    }
    k_work_submit(&lock_notify_work);
}

void ignition_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    // Só tranca se detectar a queda de tensão para 0V (Borda de Descida)
    if (gpio_pin_get_dt(&ign_gpio) == 0) {
        LOG_INF("Chave desligada. Bloqueando moto...");
        update_lock(1);
    }
}

static void auto_lock_timer_handler(struct k_timer *dummy) { update_lock(1); }

/* ============================================================================
 * 5) GATT Callbacks (Preservados Integralmente)
 * ========================================================================== */
static void lock_notify_work_handler(struct k_work *work) {
    if (current_conn && lock_notify_enabled) {
        bt_gatt_notify_uuid(current_conn, &status_uuid.uuid, NULL, lock_status_buf, strlen(lock_status_buf));
    }
}

static ssize_t on_read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, lock_status_buf, strlen(lock_status_buf));
}

static ssize_t on_write_ctrl(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset != 0 || len == 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    char cmd[16] = {0};
    memcpy(cmd, buf, MIN(len, sizeof(cmd) - 1));
    str_to_upper(cmd);
    // snprintk(lock_status_buf, sizeof(lock_status_buf), "RECEBIDO: %s", cmd);
    
    if(strstr(cmd, "LOCK")){
        if(gpio_pin_get_dt(&ign_gpio) > 0){
            strcpy(lock_status_buf, "RECEIVED"); 
        } else {
            strcpy(lock_status_buf, "LOCK");
            update_lock(1);
            
        }
        k_work_submit(&lock_notify_work);
    }
     
    k_work_submit(&lock_notify_work);
    if (strstr(cmd, "TOGGLE")) update_lock(!lock_state);
    else if (strstr(cmd, "UNLOCK")) update_lock(0);
    //else if (strstr(cmd, "LOCK")) update_lock(1);
    return len;
}

static ssize_t on_write_time_sync(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset != 0 || len != 7) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    const uint8_t *p = (const uint8_t *)buf;
    int year = 2000 + p[0], month = p[1], day = p[2], hour = p[3], minute = p[4], second = p[5];
    if (!validate_ymdhms(year, month, day, hour, minute, second)) return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    const int32_t brt_off_sec = -3 * 3600;
    int64_t epoch_ms_utc = ymdhms_to_epoch_ms_utc(year, month, day, hour, minute, second) - ((int64_t)brt_off_sec * 1000LL);
    timekeeper_validate_and_sync_ms(epoch_ms_utc, 2000);
    k_work_submit(&lock_notify_work);
    return len;
}

static void on_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    lock_notify_enabled = (value & BT_GATT_CCC_NOTIFY) != 0;
}

BT_GATT_SERVICE_DEFINE(lock_svc,
    BT_GATT_PRIMARY_SERVICE(&svc_uuid),
    BT_GATT_CHARACTERISTIC(&status_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, on_read_status, NULL, NULL),
    BT_GATT_CCC(on_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&ctrl_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, on_write_ctrl, NULL),
    BT_GATT_CHARACTERISTIC(&time_sync_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, on_write_time_sync, NULL),
);

/* ============================================================================
 * 6) BLE Advertising (Corrigido para build v3.2.1)
 * ========================================================================== */
static void start_advertising(void) {
    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, UUID_SERVICE),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
    };
    bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY, 
                    BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL), 
                    ad, ARRAY_SIZE(ad), NULL, 0);
}

static void adv_restart_work_handler(struct k_work *work) {
    bt_le_adv_stop();
    k_sleep(K_MSEC(50));
    start_advertising();
}

static void connected(struct bt_conn *conn, uint8_t err) { if (!err) current_conn = bt_conn_ref(conn); }
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; }
    lock_notify_enabled = false;
    k_work_submit(&adv_restart_work);
}
BT_CONN_CB_DEFINE(conn_callbacks) = { .connected = connected, .disconnected = disconnected };

static void bt_ready(int err) {
    if (err) return;
    settings_subsys_init();
    settings_register(&lock_conf);
    settings_load();
    start_advertising();
}

/* ============================================================================
 * 7) Main
 * ========================================================================== */
int main(void) {
    k_work_init(&adv_restart_work, adv_restart_work_handler);
    k_work_init(&lock_notify_work, lock_notify_work_handler);
    (void)timekeeper_init(); 

    if (run_led.port) gpio_pin_configure_dt(&run_led, GPIO_OUTPUT_ACTIVE);
    if (lock_gpio.port) {
        gpio_pin_configure_dt(&lock_gpio, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&lock_gpio, !lock_state);
    }
    if (ign_gpio.port) {
        gpio_pin_configure_dt(&ign_gpio, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&ign_gpio, GPIO_INT_EDGE_BOTH);
        gpio_init_callback(&ign_cb_data, ignition_handler, BIT(ign_gpio.pin));
        gpio_add_callback(ign_gpio.port, &ign_cb_data);
    }

    bt_enable(bt_ready);

    while (1) {
        if (run_led.port) gpio_pin_toggle_dt(&run_led);
        
        // Bloqueio extra do Timer se a moto for ligada
        if (gpio_pin_get_dt(&ign_gpio) > 0 && lock_state == 0) {
            k_timer_stop(&auto_lock_timer);
        }
        k_sleep(K_SECONDS(1));
    }
    return 0;
}