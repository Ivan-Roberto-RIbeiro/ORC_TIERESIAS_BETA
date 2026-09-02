#include "ble_telemetry.h"
#include "zbus_common.h"  /* <-- Essa linha traz os comandos de volta! */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(bluetooth, LOG_LEVEL_INF);

#define BT_THREAD_STACK_SIZE 4096
#define BT_THREAD_PRIORITY 4
#define BLUETOOTH_QUEUE_SIZE 4

#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* ======================================================================
 * 1. DEFINIÇÃO DOS UUIDS DE 128-BITS (Base: 5A10XXXX-2026-...)
 * ====================================================================== */
#define BT_UUID_ORC_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x5a100001, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)
#define BT_UUID_ORC_RX_VAL \
    BT_UUID_128_ENCODE(0x5a100002, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)
#define BT_UUID_ORC_TX_VAL \
    BT_UUID_128_ENCODE(0x5a100003, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)

#define BT_UUID_ORC_SERVICE BT_UUID_DECLARE_128(BT_UUID_ORC_SERVICE_VAL)
#define BT_UUID_ORC_RX      BT_UUID_DECLARE_128(BT_UUID_ORC_RX_VAL)
#define BT_UUID_ORC_TX      BT_UUID_DECLARE_128(BT_UUID_ORC_TX_VAL)

/* ======================================================================
 * 2. ESTRUTURA DO PACOTE DE DADOS (16 Bytes)
 * ====================================================================== */
struct __attribute__((packed)) orc_frame_t {
    uint16_t timestamp;
    uint16_t emg1;
    uint16_t emg2;
    uint16_t mic1;
    uint16_t mic2;
    int16_t  imu_x;
    int16_t  imu_y;
    int16_t  imu_z;
};

/* Variável global para rastrear se o aplicativo ativou as notificações */
static bool notify_tx_enabled = false;

/* ======================================================================
 * 3. CALLBACKS DO BLUETOOTH
 * ====================================================================== */
/* Função chamada quando o celular escreve na Característica de Controle (RX) */
static ssize_t orc_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    LOG_DBG("Comando recebido do celular!");
    
    if (len != 1U) {
        LOG_WRN("Tamanho de comando invalido.");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t comando = *((uint8_t *)buf);
    struct bt_cmd_chan_msg cmd_msg; // Cria a mensagem do Zbus
    
    if (comando == 0x01) {
        LOG_INF("Comando INICIAR TESTE recebido.");
        cmd_msg.cmd = BT_CMD_STREAMING; // Manda o comando para ir para streaming
        zbus_chan_pub(&bt_cmd_chan, &cmd_msg, K_MSEC(100));
    } else if (comando == 0x00) {
        LOG_INF("Comando PARAR TESTE recebido.");
        cmd_msg.cmd = BT_CMD_DISCONNECT; // Ou outro comando para parar/voltar a ficar ocioso
        zbus_chan_pub(&bt_cmd_chan, &cmd_msg, K_MSEC(100));
    }

    return len;
}

/* Função chamada quando o celular assina (Subscribe) a Telemetria (TX) */
static void orc_tx_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_tx_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notificacoes TX %s", notify_tx_enabled ? "ATIVADAS" : "DESATIVADAS");
}

/* ======================================================================
 * 4. CRIAÇÃO DA TABELA GATT
 * ====================================================================== */
BT_GATT_SERVICE_DEFINE(orc_tieresias_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ORC_SERVICE),

    /* Característica de Controle (Celular Escreve) */
    BT_GATT_CHARACTERISTIC(BT_UUID_ORC_RX,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, orc_rx_write, NULL),

    /* Característica de Telemetria (Placa Notifica) */
    BT_GATT_CHARACTERISTIC(BT_UUID_ORC_TX,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(orc_tx_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);


ZBUS_SUBSCRIBER_DEFINE(bt_cmd_sub, 4);

ZBUS_CHAN_DEFINE(
    bt_state_chan, struct bt_state_chan_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(.state = BT_STATE_OFF));

ZBUS_CHAN_DEFINE(
    bt_cmd_chan, struct bt_cmd_chan_msg, NULL, NULL, ZBUS_OBSERVERS(bt_cmd_sub), ZBUS_MSG_INIT(.cmd = BT_CMD_INIT));

/* Substitua o ZBUS_CHAN_DECLARE por esta definição completa: */
ZBUS_CHAN_DEFINE(
    led_chan, struct led_chan_msg_t, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(.led = LED_3, .cmd = TURN_OFF));

static bt_state current_state = BT_STATE_OFF;

static void set_bt_state(bt_state new_state)
{
  struct bt_state_chan_msg msg;

  if (current_state == new_state) {
    return;
  }

  current_state = new_state;
  msg.state = new_state;

  struct led_chan_msg_t led_msg;

  led_msg.led = LED_3;
  if (new_state == BT_STATE_ADVERTISING) {
    led_msg.cmd = BLINK;
  } else if (new_state == BT_STATE_CONNECTED) {
    led_msg.cmd = TURN_ON;
  } else {
    led_msg.cmd = TURN_OFF;
  }

  int err = zbus_chan_pub(&bt_state_chan, &msg, K_MSEC(100));
  if (err) {
    LOG_ERR("Failed to publish Bluetooth state change: %d", err);
  }

  err = zbus_chan_pub(&led_chan, &led_msg, K_MSEC(100));
  if (err) {
    LOG_ERR("Failed to send LED command: %d", err);
  }
}

void ble_connected_cb(void)
{
  set_bt_state(BT_STATE_CONNECTED);
}

void ble_disconnected_cb(void)
{
  set_bt_state(BT_STATE_ADVERTISING);
}

static void handle_state_off(bt_cmd cmd)
{
  if (cmd != BT_CMD_INIT) {
    LOG_WRN("Command %d not valid in OFF state", cmd);
    return;
  }

  LOG_DBG("Initializing Bluetooth");
  set_bt_state(BT_STATE_INITIALIZING);
  int ret = ble_init(ble_connected_cb, ble_disconnected_cb);
  if (ret != 0) {
    LOG_ERR("Failed to initialize Bluetooth: %d", ret);
    set_bt_state(BT_STATE_INIT_ERROR);
    return;
  }
  set_bt_state(BT_STATE_NOT_CONNECTED);
}

static void handle_state_initializing(bt_cmd cmd)
{
  LOG_WRN("Command %d ignored in INITIALIZING state", cmd);
}

static void handle_state_not_connected(bt_cmd cmd)
{
  if (cmd != BT_CMD_ADVERTISE) {
    LOG_WRN("Command %d not valid in NOT_CONNECTED state", cmd);
    return;
  }

  LOG_DBG("Starting advertising");
  int ret = ble_start_advertising();
  if (ret != 0) {
    LOG_ERR("Failed to start advertising: %d", ret);
    return;
  }
  set_bt_state(BT_STATE_ADVERTISING);
}

static void handle_state_advertising(bt_cmd cmd)
{
    /* 
     * VERSÃO SIMPLIFICADA PARA TESTES:
     * Não tenta parar a antena nem economizar bateria. 
     * Apenas ignora qualquer outro comando e mantém o Bluetooth ligado direto!
     */
    if (cmd != BT_CMD_ADVERTISE) {
        LOG_WRN("Comando %d ignorado. Mantendo o radio ligado (Advertising)!", cmd);
    }
}

static void handle_state_connecting(bt_cmd cmd)
{
  switch (cmd) {
  case BT_CMD_DISCONNECT:
    LOG_DBG("Cancelling connection attempt");
    set_bt_state(BT_STATE_DISCONNECTING);
    set_bt_state(BT_STATE_NOT_CONNECTED);
    break;
  default:
    LOG_WRN("Command %d not valid in CONNECTING state", cmd);
    break;
  }
  /* In a real implementation, connection success/failure would trigger state changes */
}

static void handle_state_connected(bt_cmd cmd)
{
  switch (cmd) {
  case BT_CMD_DISCONNECT:
    LOG_DBG("Disconnecting from connected state");
    set_bt_state(BT_STATE_DISCONNECTING);
    /* Add actual disconnection code here */
    set_bt_state(BT_STATE_NOT_CONNECTED);
    break;
  default:
    LOG_WRN("Command %d not valid in CONNECTED state", cmd);
    break;
  }
}

static void handle_state_config(bt_cmd cmd)
{
  /* Placeholder for CONFIG state handling */
  LOG_WRN("Command %d not implemented in CONFIG state", cmd);
}

static void handle_state_streaming(bt_cmd cmd)
{
  /* Placeholder for STREAMING state handling */
  LOG_WRN("Command %d not implemented in STREAMING state", cmd);
}

static void handle_state_fota(bt_cmd cmd)
{
  /* Placeholder for FOTA state handling */
  LOG_WRN("Command %d not implemented in FOTA state", cmd);
}

static void handle_state_disconnecting(bt_cmd cmd)
{
  /* Most commands would be ignored during disconnection */
  LOG_WRN("Command %d ignored in DISCONNECTING state", cmd);
}

static void bt_state_machine(bt_cmd cmd)
{
  /* State machine implementation - routes commands to appropriate handler */
  switch (current_state) {
  case BT_STATE_OFF:
    handle_state_off(cmd);
    break;
  case BT_STATE_INITIALIZING:
    handle_state_initializing(cmd);
    break;
  case BT_STATE_NOT_CONNECTED:
    handle_state_not_connected(cmd);
    break;
  case BT_STATE_ADVERTISING:
    handle_state_advertising(cmd);
    break;
  case BT_STATE_CONNECTING:
    handle_state_connecting(cmd);
    break;
  case BT_STATE_CONNECTED:
    handle_state_connected(cmd);
    break;
  case BT_STATE_CONFIG:
    handle_state_config(cmd);
    break;
  case BT_STATE_STREAMING:
    handle_state_streaming(cmd);
    break;
  case BT_STATE_FOTA:
    handle_state_fota(cmd);
    break;
  case BT_STATE_DISCONNECTING:
    handle_state_disconnecting(cmd);
    break;
  default:
    LOG_WRN("Unhandled Bluetooth state: %d", current_state);
    break;
  }
}

static void bt_thread(void* arg1, void* arg2, void* arg3)
{
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  LOG_DBG("Bluetooth thread started.");
  int ret = 0;
  struct bt_cmd_chan_msg msg;

  while (1) {
    const struct zbus_channel* chan;

    ret = zbus_sub_wait(&bt_cmd_sub, &chan, K_FOREVER);
    if (ret != 0) {
      LOG_ERR("Error waiting for Bluetooth command: %d", ret);
      continue;
    }

    ret = zbus_chan_read(chan, &msg, K_MSEC(500));
    if (ret != 0) {
      LOG_ERR("Error reading Bluetooth command: %d", ret);
      continue;
    }

    bt_state_machine(msg.cmd);
  }
}

/* ======================================================================
 * 5. IMPLEMENTAÇÃO FÍSICA DO RÁDIO (Nativo Zephyr)
 * ====================================================================== */

/* Callbacks de conexão padrão do Zephyr */
static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Falha na conexao (err %u)", err);
        return;
    }
    LOG_INF("Dispositivo conectado!");
    ble_connected_cb(); // Avisa o Zbus
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Dispositivo desconectado (razao: %d)", reason);
    ble_disconnected_cb(); // Avisa o Zbus
}

/* Registra os callbacks de conexão na memória do sistema */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

/* Função de inicialização real */
int ble_init(void (*connected_cb)(void), void (*disconnected_cb)(void)) {
    // Liga o rádio do nRF5340
    return bt_enable(NULL);
}

/* ======================================================================
 * Motor de Transmissão da Antena
 * ====================================================================== */
int ble_start_advertising(void) {
    
    /* Pacote Primário: Diz que a placa é conectável e tem o nosso serviço customizado */
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ORC_SERVICE_VAL),
    };

    /* Pacote Secundário (Scan Response): Entrega o nome para o celular */
    const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    struct bt_le_adv_param param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = BT_LE_ADV_OPT_CONN, 
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
        .peer = NULL,
    };

    /* Passamos os dois pacotes: 'ad' (anúncio) e 'sd' (resposta com o nome) */
    int err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    
    if (err) {
        LOG_ERR("Falha ao ligar antena (err %d).", err);
    }
    
    return err;
}

K_THREAD_DEFINE(bt_thread_id, BT_THREAD_STACK_SIZE, bt_thread, NULL, NULL, NULL, BT_THREAD_PRIORITY, 0, 0);


/* ======================================================================
 * Tabela GATT (O Coração do Bluetooth)
 * ====================================================================== */
/* 1. Definição dos UUIDs customizados (Com o ULL para evitar o erro de 32 bits) */
#define BT_UUID_ORC_SERVICE_VAL BT_UUID_128_ENCODE(0x5a100001, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)
#define BT_UUID_ORC_RX_VAL      BT_UUID_128_ENCODE(0x5a100002, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)
#define BT_UUID_ORC_TX_VAL      BT_UUID_128_ENCODE(0x5a100003, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)

static struct bt_uuid_128 orc_svc_uuid = BT_UUID_INIT_128(BT_UUID_ORC_SERVICE_VAL);
static struct bt_uuid_128 orc_rx_uuid  = BT_UUID_INIT_128(BT_UUID_ORC_RX_VAL);
static struct bt_uuid_128 orc_tx_uuid  = BT_UUID_INIT_128(BT_UUID_ORC_TX_VAL);

/* 2. Função que avisa no log quando você aperta "Subscribe" no celular */
static void orc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_INF("Celular alterou inscricao de dados: %d", value);
}

/* 3. A criação da tabela 'orc_svc' (O compilador vai achar a variável agora!) */
BT_GATT_SERVICE_DEFINE(orc_svc,
    BT_GATT_PRIMARY_SERVICE(&orc_svc_uuid),
    
    /* TX: Índice 1 (Declaração) e Índice 2 (Valor real onde injetamos os dados falsos) */
    BT_GATT_CHARACTERISTIC(&orc_tx_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(orc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    /* RX: Recebe comandos do celular (Para uso futuro) */
    BT_GATT_CHARACTERISTIC(&orc_rx_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, NULL, NULL)
);

/* ======================================================================
 * Sensor Fantasma (Mock Data Generator)
 * ====================================================================== */
static void mock_sensor_thread(void *arg1, void *arg2, void *arg3)
{
    /* Criamos um pacote temporário de 16 bytes simulando a nossa orc_frame_t */
    uint8_t mock_payload[16] = {0}; 
    uint16_t counter = 0;

    while (1) {
        /* Só consome processamento e transmite se o celular estiver conectado */
        if (current_state == BT_STATE_CONNECTED) {
            
            /* 1. Injetamos dados falsos (Um contador crescente e bytes fixos) */
            mock_payload[0] = (counter >> 8) & 0xFF; /* Contador (MSB) */
            mock_payload[1] = counter & 0xFF;        /* Contador (LSB) */
            mock_payload[2] = 0xAA;                  /* Simulação EMG */
            mock_payload[3] = 0xBB;                  /* Simulação IMU */
            mock_payload[15] = 0xFF;                 /* Fim do frame */
            
            /* 2. Dispara a notificação GATT para o celular */
            /* IMPORTANTE: O nome 'orc_svc' deve bater com o nome da sua tabela GATT */
            int err = bt_gatt_notify(NULL, &orc_svc.attrs[2], mock_payload, sizeof(mock_payload));
            
            if (err && err != -ENOTCONN) {
                LOG_ERR("Falha ao enviar mock data (err: %d)", err);
            }

            counter++;
        }
        
        /* Dorme por 10ms -> Gera uma taxa de transmissão de 100 pacotes por segundo (100Hz) */
        k_msleep(10);
    }
}

/* Inicia a thread automaticamente junto com o sistema, com prioridade 5 */
K_THREAD_DEFINE(mock_sensor_tid, 1024, mock_sensor_thread, NULL, NULL, NULL, 5, 0, 0);