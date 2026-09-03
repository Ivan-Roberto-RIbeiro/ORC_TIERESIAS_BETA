#include "ble_telemetry.h"
#include "zbus_common.h"
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_REGISTER(bluetooth_mock, LOG_LEVEL_INF);

/* 1. Recria os canais Zbus para o main.c compilar sem erro de 'undefined reference' */
ZBUS_SUBSCRIBER_DEFINE(bt_cmd_sub, 4);

ZBUS_CHAN_DEFINE(bt_state_chan, struct bt_state_chan_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(.state = BT_STATE_OFF));
ZBUS_CHAN_DEFINE(bt_cmd_chan, struct bt_cmd_chan_msg, NULL, NULL, ZBUS_OBSERVERS(bt_cmd_sub), ZBUS_MSG_INIT(.cmd = BT_CMD_INIT));
ZBUS_CHAN_DEFINE(led_chan, struct led_chan_msg_t, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(.led = LED_3, .cmd = TURN_OFF));

/* 2. Mantém o Bluetooth desativado/dormindo enquanto focamos no teste do terminal */
static void bt_dummy_thread(void *arg1, void *arg2, void *arg3) {
    while(1) { 
        k_sleep(K_FOREVER); 
    }
}
K_THREAD_DEFINE(bt_tid, 1024, bt_dummy_thread, NULL, NULL, NULL, 5, 0, 0);

/* 3. Funções vazias para o ble_telemetry.h não reclamar */
void ble_connected_cb(void) {}
void ble_disconnected_cb(void) {}
int ble_init(void (*c)(void), void (*d)(void)) { return 0; }
int ble_start_advertising(void) { return 0; }
int ble_stop_advertising(void) { return 0; }