#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* --- 1. DEFINES DE UUID --- */
#define UUID_SERVICE       BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
/* O App está usando o 1525 para enviar comandos, então vamos focar nele */
#define UUID_LOCK_CTRL_VAL BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

static struct bt_uuid_128 svc_uuid  = BT_UUID_INIT_128(UUID_SERVICE);
static struct bt_uuid_128 lock_uuid = BT_UUID_INIT_128(UUID_LOCK_CTRL_VAL);

/* --- 2. GLOBAIS --- */
static uint8_t lock_state = 0; 
static uint8_t lock_notify_enabled;
static char lock_status_buf[8] = "UNLOCK";

static struct bt_conn *current_conn;
static struct k_work lock_notify_work;

/* Hardware */
static const struct gpio_dt_spec run_led  = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec lock_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0}); // GPIO 0.9

/* --- 3. LÓGICA DE TRANCA --- */

static void notify_status(void)
{
	if (current_conn && lock_notify_enabled) {
		bt_gatt_notify_uuid(current_conn, &lock_uuid.uuid, NULL, 
						   lock_status_buf, strlen(lock_status_buf));
		LOG_INF(">>> Resposta enviada ao App: %s", lock_status_buf);
	}
}

static void lock_notify_work_handler(struct k_work *work) { notify_status(); }

static void update_lock(uint8_t locked)
{
	lock_state = locked;
	if (lock_gpio.port) gpio_pin_set_dt(&lock_gpio, lock_state);
	
	strcpy(lock_status_buf, lock_state ? "LOCK" : "UNLOCK");
	LOG_INF("Hardware alterado: %s", lock_status_buf);
	
	k_work_submit(&lock_notify_work);
}

/* --- 4. GATT CALLBACKS --- */

static ssize_t on_write_ctrl(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	char cmd[16] = {0};
	uint16_t copy_len = MIN(len, sizeof(cmd) - 1);
	memcpy(cmd, buf, copy_len);

	// Log de depuração essencial
	LOG_INF("Comando recebido no UUID 1525: %s", cmd);

	// Converte para maiúsculo (Robustez)
	for (int i = 0; cmd[i]; i++) if(cmd[i]>='a' && cmd[i]<='z') cmd[i]-=32;

	if (strstr(cmd, "UNLOCK")) {
		update_lock(0);
	} else if (strstr(cmd, "LOCK")) {
		update_lock(1);
	} else {
		LOG_WRN("Texto ignorado. Use LOCK ou UNLOCK.");
	}

	return len;
}

static ssize_t on_read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, lock_status_buf, strlen(lock_status_buf));
}

static void on_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	lock_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notificacoes: %s", lock_notify_enabled ? "ATIVAS" : "INATIVAS");
}

/* --- 5. SERVIÇO GATT UNIFICADO --- */
BT_GATT_SERVICE_DEFINE(lock_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),
	/* Agora a característica 1525 faz TUDO: recebe comando e notifica status */
	BT_GATT_CHARACTERISTIC(&lock_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
		on_read_status, on_write_ctrl, NULL),
	BT_GATT_CCC(on_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* --- 6. BLE CORE --- */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, UUID_SERVICE),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

static void connected(struct bt_conn *conn, uint8_t err) {
	if (!err) current_conn = bt_conn_ref(conn);
	LOG_INF("Conectado!");
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; }
	LOG_INF("Desconectado.");
}

BT_CONN_CB_DEFINE(conn_callbacks) = { .connected = connected, .disconnected = disconnected };

static void bt_ready(int err) {
	if (err) return;
	if (IS_ENABLED(CONFIG_SETTINGS)) settings_load();
	struct bt_le_adv_param adv_param = *BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY, 
		BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
	bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	LOG_INF("Publicidade ON!");
}

int main(void)
{
	k_work_init(&lock_notify_work, lock_notify_work_handler);
	if (run_led.port)  gpio_pin_configure_dt(&run_led,  GPIO_OUTPUT_ACTIVE);
	if (lock_gpio.port) gpio_pin_configure_dt(&lock_gpio, GPIO_OUTPUT_INACTIVE);
	bt_enable(bt_ready);
	while (1) {
		if (run_led.port) gpio_pin_toggle_dt(&run_led);
		k_sleep(K_SECONDS(1));
	}
	return 0;
}