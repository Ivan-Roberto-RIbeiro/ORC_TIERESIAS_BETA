#ifndef ZBUS_COMMON_H
#define ZBUS_COMMON_H

#include <zephyr/zbus/zbus.h>
#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * 1. ESTADOS DA MÁQUINA (O que o Bluetooth está fazendo agora?)
 * ====================================================================== */
typedef enum {
    BT_STATE_OFF,
    BT_STATE_INITIALIZING,
    BT_STATE_INIT_ERROR,
    BT_STATE_NOT_CONNECTED,
    BT_STATE_ADVERTISING,
    BT_STATE_CONNECTING,
    BT_STATE_CONNECTED,
    BT_STATE_CONFIG,
    BT_STATE_STREAMING,     // Estado crítico onde gravamos os músculos
    BT_STATE_FOTA,
    BT_STATE_DISCONNECTING
} bt_state;

/* ======================================================================
 * 2. COMANDOS (Ordens enviadas para a máquina de estados)
 * ====================================================================== */
typedef enum {
    BT_CMD_INIT,
    BT_CMD_ADVERTISE,
    BT_CMD_DISCONNECT,
    BT_CMD_STREAMING,       // Gatilho para iniciar a leitura de 2000Hz
    BT_CMD_STOP_STREAMING   // Gatilho para parar e voltar a economizar bateria
} bt_cmd;

/* ======================================================================
 * 3. CONTROLE DE HARDWARE (LEDs)
 * ====================================================================== */
typedef enum {
    TURN_OFF,
    TURN_ON,
    BLINK
} led_cmd_t;

typedef enum {
    LED_1,
    LED_2,
    LED_3,
    LED_4
} led_id_t;

/* ======================================================================
 * 4. ESTRUTURAS DE MENSAGENS (O que viaja pelo Zbus)
 * ====================================================================== */
struct bt_state_chan_msg {
    bt_state state;
};

struct bt_cmd_chan_msg {
    bt_cmd cmd;
};

struct led_chan_msg_t {
    led_id_t led;
    led_cmd_t cmd;
};

/* ======================================================================
 * 5. DECLARAÇÃO DOS CANAIS (Para outros arquivos .c enxergarem)
 * ====================================================================== */
ZBUS_CHAN_DECLARE(bt_state_chan);
ZBUS_CHAN_DECLARE(bt_cmd_chan);
ZBUS_CHAN_DECLARE(led_chan);

#endif /* ZBUS_COMMON_H */