#include "ble_telemetry.h"
#include "zbus_common.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "../sensor_maestro.h"

LOG_MODULE_REGISTER(bluetooth, LOG_LEVEL_INF);

/* ======================================================================
 * 1. ESTRUTURA DO PACOTE DE DADOS (O "Caminhão" de 240 Bytes)
 * ====================================================================== */
/* Frame individual de 16 bytes gerado a cada leitura (2.000 vezes por segundo) */
/*struct __attribute__((packed)) orc_frame_t {
    uint16_t timestamp;
    uint16_t emg1;
    uint16_t emg2;
    uint16_t mic1;
    uint16_t mic2;
    int16_t  imu_x;
    int16_t  imu_y;
    int16_t  imu_z;
};*/ 

/* O pacote que o Bluetooth vai enviar: Agrupa 15 frames (15 * 16 = 240 bytes) */
/*struct __attribute__((packed)) orc_batch_msg_t {
    struct orc_frame_t frames[15];
};*/ 

/* ======================================================================
 * 2. CONFIGURAÇÕES DO ZBUS E VARIÁVEIS GLOBAIS
 * ====================================================================== */
/* Ouvintes (Subscribers) para os canais do Zbus */
ZBUS_SUBSCRIBER_DEFINE(bt_cmd_sub, 4);
ZBUS_SUBSCRIBER_DEFINE(sensor_batch_sub, 4); /* Escuta o "caminhão" de dados cheio */

/* Canais de Comunicação Básicos do Sistema */
ZBUS_CHAN_DEFINE(bt_state_chan, struct bt_state_chan_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(.state = BT_STATE_OFF));
ZBUS_CHAN_DEFINE(bt_cmd_chan, struct bt_cmd_chan_msg, NULL, NULL, ZBUS_OBSERVERS(bt_cmd_sub), ZBUS_MSG_INIT(.cmd = BT_CMD_INIT));
ZBUS_CHAN_DEFINE(led_chan, struct led_chan_msg_t, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(.led = LED_3, .cmd = TURN_OFF));
/* Canal Mestre declarado externamente (definido no sensor_maestro.c) */
ZBUS_CHAN_DECLARE(sensor_batch_chan);


/* Canal Mestre de Dados: Por onde a Thread do Timer envia os 240 bytes para o Bluetooth */
//ZBUS_CHAN_DEFINE(sensor_batch_chan, struct orc_batch_msg_t, NULL, NULL, ZBUS_OBSERVERS(sensor_batch_sub), ZBUS_MSG_INIT(0));

/* Variáveis de controle de estado do rádio */
static bt_state current_state = BT_STATE_OFF;
static bool notify_tx_enabled = false; /* Diz se o celular apertou o botão "Subscribe" */
static struct bt_conn *current_conn = NULL; /* Guarda a conexão atual com o celular */

/* ======================================================================
 * 3. IDENTIDADE DO BLUETOOTH (UUIDs E TABELA GATT)
 * ====================================================================== */
/* Endereços de 128-bits exclusivos do seu projeto ORC Tieresias */
#define BT_UUID_ORC_SERVICE_VAL BT_UUID_128_ENCODE(0x5a100001, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)
#define BT_UUID_ORC_RX_VAL      BT_UUID_128_ENCODE(0x5a100002, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)
#define BT_UUID_ORC_TX_VAL      BT_UUID_128_ENCODE(0x5a100003, 0x2026, 0x0000, 0x0000, 0x000000000000ULL)

static struct bt_uuid_128 orc_svc_uuid = BT_UUID_INIT_128(BT_UUID_ORC_SERVICE_VAL);
static struct bt_uuid_128 orc_rx_uuid  = BT_UUID_INIT_128(BT_UUID_ORC_RX_VAL);
static struct bt_uuid_128 orc_tx_uuid  = BT_UUID_INIT_128(BT_UUID_ORC_TX_VAL);

/* Callback: Disparado quando o celular envia um comando para a placa (RX) */
static ssize_t orc_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    LOG_INF("Comando recebido do celular! Tamanho: %d bytes", len);
    /* Aqui você pode ler o 'buf' para saber se o celular mandou parar ou iniciar o teste */
    return len;
}

/* Callback: Disparado quando o celular ativa/desativa o recebimento de dados (TX) */
static void orc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    notify_tx_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Celular alterou inscricao de dados: %s", notify_tx_enabled ? "LIGADO" : "DESLIGADO");
}

/* A Tabela GATT Limpa: Define o Serviço e as duas Portas (TX e RX) sem duplicidade */
BT_GATT_SERVICE_DEFINE(orc_svc,
    BT_GATT_PRIMARY_SERVICE(&orc_svc_uuid),
    
    /* Porta TX: Envia os 240 bytes (NOTIFY) */
    BT_GATT_CHARACTERISTIC(&orc_tx_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(orc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    /* Porta RX: Recebe comandos (WRITE) */
    BT_GATT_CHARACTERISTIC(&orc_rx_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, orc_rx_write, NULL)
);

/* ======================================================================
 * 4. GESTÃO DE CONEXÃO E NEGOCIAÇÃO DE MTU
 * ====================================================================== */
static struct bt_gatt_exchange_params mtu_exchange_params;

/* Callback: Chamado automaticamente quando o celular responde ao pedido de MTU */
static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params) {
    if (err) {
        LOG_WRN("Erro ao negociar MTU: %d", err);
    } else {
        LOG_INF("Negociacao de MTU concluida! MTU Atual: %d bytes", bt_gatt_get_mtu(conn));
    }
}

/* Callback: Disparado no exato milissegundo que um celular conecta */
static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Falha na conexao (err %u)", err);
        return;
    }
    LOG_INF("Celular conectado!");
    current_conn = bt_conn_ref(conn);
    current_state = BT_STATE_CONNECTED;
    
    /* Dispara o pedido de aumento de MTU para caber os nossos 240 bytes */
    mtu_exchange_params.func = mtu_exchange_cb;
    int mtu_err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (mtu_err) {
        LOG_WRN("Nao foi possivel pedir aumento de MTU agora (err %d)", mtu_err);
    }
}

/* Callback: Disparado quando o celular se afasta ou desconecta */
static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Celular desconectado (razao: %d)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    current_state = BT_STATE_ADVERTISING;
    notify_tx_enabled = false;
    
    /* Religa a antena automaticamente para o celular poder achar a placa de novo */
    ble_start_advertising(); 
}

/* Registra os callbacks no núcleo do Zephyr */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

/* Liga a antena (Advertising) para a placa ficar "visível" no ar */
int ble_start_advertising(void) {
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ORC_SERVICE_VAL),
    };
    const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };
    
    /* Configura os intervalos rápidos de anúncio */
    struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
    
    int err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err && err != -EALREADY) LOG_ERR("Falha ao ligar antena (err %d).", err);
    return err;
}

/* Função de inicialização chamada no Boot */
int ble_init(void (*connected_cb)(void), void (*disconnected_cb)(void)) {
    int err = bt_enable(NULL);
    if (err) return err;
    return ble_start_advertising();
}

/* ======================================================================
 * 5. THREAD DE TRANSMISSÃO (Despachando o "Caminhão" de Dados)
 * ====================================================================== */
static void ble_tx_thread(void *arg1, void *arg2, void *arg3)
{
    const struct zbus_channel *chan;
    // Substitui a struct por um buffer plano do exato tamanho do lote (240 bytes)
    uint8_t batch_data[15 * 16]; 

    LOG_INF("Iniciando a Thread de Transmissao BLE...");
    
    zbus_chan_add_obs(&sensor_batch_chan, &sensor_batch_sub, K_MSEC(200));
    ble_init(NULL, NULL);

    while (1) {
        /* BATERIA OTIMIZADA: A thread dorme 100% do tempo. 
         * Ela só acorda quando o Zbus avisa que o pacote de 240 bytes está pronto. */
        if (zbus_sub_wait(&sensor_batch_sub, &chan, K_FOREVER) == 0) {
            
            if (chan == &sensor_batch_chan) {
                /* Extrai os 240 bytes do canal Zbus */
                zbus_chan_read(&sensor_batch_chan, &batch_data, K_NO_WAIT);
                
                /* Segurança: Só transmite se estiver pareado e o App autorizar */
                if (current_state == BT_STATE_CONNECTED && notify_tx_enabled) {
                    
                    /* Dispara o pacote gigantesco inteiro de uma vez só */
                    int err = bt_gatt_notify(current_conn, &orc_svc.attrs[2], 
                                             &batch_data, sizeof(batch_data));
                    
                    /* Proteção contra inundação do terminal se o buffer RAM lotar */
                    if (err && err != -ENOTCONN && err != -ENOMEM) {
                        LOG_WRN("Falha ao enviar pacote BLE: %d", err);
                    }
                }
            }
        }
    }
}

/* Prioridade 5: Roda depois das rotinas de DSP e do Timer mestre */
K_THREAD_DEFINE(ble_tx_tid, 2048, ble_tx_thread, NULL, NULL, NULL, 5, 0, 0);