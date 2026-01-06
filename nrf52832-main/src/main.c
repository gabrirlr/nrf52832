/*
 * BLE Lock Service (Zephyr)
 *
 * Serviço 0x1523 (128-bit base UUID)
 *  - Char 0x1524: STATUS  (READ + NOTIFY)  -> App se inscreve (CCC) e recebe "LOCKED"/"UNLOCKED"
 *  - Char 0x1525: CTRL    (WRITE/WNR)      -> App envia comandos (ex: "LOCK", "UNLOCK", "TOGGLE")
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
#include <string.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ============================================================================
 * 1) UUIDs (128-bit)
 * ========================================================================== */
#define UUID_SERVICE           BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define UUID_LOCK_STATUS_VAL   BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123) /* STATUS: READ/NOTIFY */
#define UUID_LOCK_CTRL_VAL     BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123) /* CTRL:   WRITE */

static struct bt_uuid_128 svc_uuid    = BT_UUID_INIT_128(UUID_SERVICE);
static struct bt_uuid_128 status_uuid = BT_UUID_INIT_128(UUID_LOCK_STATUS_VAL);
static struct bt_uuid_128 ctrl_uuid   = BT_UUID_INIT_128(UUID_LOCK_CTRL_VAL);

/* ============================================================================
 * 2) Estado global do aplicativo
 * ========================================================================== */
static uint8_t lock_state = 0;                 /* 0=UNLOCKED, 1=LOCKED */
static bool    lock_notify_enabled = false;    /* habilitado via CCC */
static char    lock_status_buf[10] = "UNLOCKED";

static struct bt_conn *current_conn = NULL;    /* conexão ativa (1 max) */

/* Work item para notificar em contexto seguro (evita notificar “dentro” de callback) */
static struct k_work lock_notify_work;

/* ============================================================================
 * 3) Hardware (GPIO via Devicetree aliases)
 * ========================================================================== */
static const struct gpio_dt_spec run_led   = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec lock_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});

/* ============================================================================
 * 4) Funções auxiliares (strings/comandos)
 * ========================================================================== */

/* Converte string ASCII para uppercase (in-place) */
static void str_to_upper(char *s)
{
    for (int i = 0; s[i]; i++) {
        if (s[i] >= 'a' && s[i] <= 'z') {
            s[i] -= 32;
        }
    }
}

/* ============================================================================
 * 5) Notificação de STATUS (1524)
 * ========================================================================== */

/*
 * Envia notificação do STATUS (char 0x1524).
 * IMPORTANTE: o app do celular precisa ter escrito no CCC para habilitar NOTIFY.
 */
static void notify_status(void)
{
    if (!current_conn || !lock_notify_enabled) {
        return; /* sem conexão ou CCC não habilitado */
    }

    /* Passa NULL no attr: Zephyr localiza a characteristic pelo UUID */
    int err = bt_gatt_notify_uuid(current_conn, &status_uuid.uuid, NULL,
                                  lock_status_buf, strlen(lock_status_buf));

    LOG_INF(">>> Notify STATUS (%s) err=%d", lock_status_buf, err);
}

/* Handler do work item */
static void lock_notify_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    notify_status();
}

/* ============================================================================
 * 6) Controle do lock (atualiza GPIO + buffer + notifica)
 * ========================================================================== */

/*
 * Atualiza estado do lock e “publica” o status para o app (via notify).
 * locked=1 -> LOCKED
 * locked=0 -> UNLOCKED
 */
static void update_lock(uint8_t locked)
{
    lock_state = locked ? 1 : 0;

    /* Atualiza hardware (GPIO) se existir no overlay */
    if (lock_gpio.port) {
        gpio_pin_set_dt(&lock_gpio, lock_state);
    }

    /* Status exatamente como o app espera */
    strcpy(lock_status_buf, lock_state ? "LOCKED" : "UNLOCKED");

    LOG_INF("HARDWARE ALTERADO PARA: %s", lock_status_buf);

    /* Agenda notificação (em contexto da workqueue) */
    k_work_submit(&lock_notify_work);
}

/* ============================================================================
 * 7) Callbacks GATT
 * ========================================================================== */

/* READ do STATUS (1524): permite o app ler “LOCKED/UNLOCKED” */
static ssize_t on_read_status(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    ARG_UNUSED(conn);
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             lock_status_buf, strlen(lock_status_buf));
}

/*
 * WRITE do CTRL (1525): recebe comando do app.
 * Exemplos aceitos:
 *  - "LOCK"   -> trava
 *  - "UNLOCK" -> destrava
 *  - "TOGGLE" -> inverte
 *
 * Observação importante:
 *  - "UNLOCK" contém "LOCK", então a ordem do parse importa!
 */
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

    /* Parse robusto */
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

/*
 * CCC callback (habilita/desabilita NOTIFY do STATUS 1524)
 * value contém bits BT_GATT_CCC_NOTIFY / INDICATE
 */
static void on_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);

    lock_notify_enabled = (value & BT_GATT_CCC_NOTIFY) != 0;

    LOG_INF("Notificacoes STATUS: %s (raw=0x%04x)",
            lock_notify_enabled ? "ATIVAS" : "INATIVAS", value);

    /* Quando habilitar, já manda o estado atual */
    if (lock_notify_enabled) {
        k_work_submit(&lock_notify_work);
    }
}

/* ============================================================================
 * 8) Serviço GATT (1523)
 * ========================================================================== */
BT_GATT_SERVICE_DEFINE(lock_svc,
    BT_GATT_PRIMARY_SERVICE(&svc_uuid),

    /* 1524: STATUS (READ + NOTIFY) */
    BT_GATT_CHARACTERISTIC(&status_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        on_read_status, NULL, NULL),

    /* CCC do STATUS (onde o app habilita NOTIFY) */
    BT_GATT_CCC(on_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 1525: CTRL (WRITE/WNR) */
    BT_GATT_CHARACTERISTIC(&ctrl_uuid.uuid,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL, on_write_ctrl, NULL),
);

/* ============================================================================
 * 9) Advertising
 * ========================================================================== */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, UUID_SERVICE),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

/* ============================================================================
 * 10) Conexão BLE
 * ========================================================================== */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Falha ao conectar (err=%u)", err);
        return;
    }

    /* Guarda referência da conexão */
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

    lock_notify_enabled = false; /* CCC será refeito na próxima conexão */

    LOG_INF("Desconectado (reason=%u)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ============================================================================
 * 11) Bluetooth ready (bt_enable callback)
 * ========================================================================== */
static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("bt_enable falhou: %d", err);
        return;
    }

    /* Carrega settings (identity/keys) se habilitado */
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
 * 12) main()
 * ========================================================================== */
int main(void)
{
    /* Work item para notificar status */
    k_work_init(&lock_notify_work, lock_notify_work_handler);

    /* Configura GPIOs se existirem */
    if (run_led.port) {
        gpio_pin_configure_dt(&run_led, GPIO_OUTPUT_ACTIVE);
    }
    if (lock_gpio.port) {
        gpio_pin_configure_dt(&lock_gpio, GPIO_OUTPUT_INACTIVE);
    }

    /* Inicia Bluetooth (assíncrono: chama bt_ready ao concluir) */
    bt_enable(bt_ready);

    /* Loop principal: pisca LED de “run” */
    while (1) {
        if (run_led.port) {
            gpio_pin_toggle_dt(&run_led);
        }
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
