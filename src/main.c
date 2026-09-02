#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

/* Inclui o nosso cardápio de comandos do Zbus */
#include "zbus_common.h" 

/* Registra o módulo de log para o main */
LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("    Iniciando o orc-tieresias...        ");
    LOG_INF("========================================");

    /* 1. Manda LIGAR a antena (Tira do OFF e vai para NOT_CONNECTED) */
    struct bt_cmd_chan_msg cmd_msg = {
        .cmd = BT_CMD_INIT
    };
    int err = zbus_chan_pub(&bt_cmd_chan, &cmd_msg, K_MSEC(100));
    
    if (err) {
        LOG_ERR("Falha ao acordar a thread do Bluetooth! (err: %d)", err);
    } else {
        LOG_INF("Comando BT_CMD_INIT enviado com sucesso.");
    }

    /* Dá 500 milissegundos para o Zephyr iniciar o hardware do rádio com segurança */
    k_msleep(500);

    /* 2. Manda ANUNCIAR (Começa a gritar o nome no ar) */
    cmd_msg.cmd = BT_CMD_ADVERTISE;
    err = zbus_chan_pub(&bt_cmd_chan, &cmd_msg, K_MSEC(100));

    if (err) {
        LOG_ERR("Falha ao iniciar advertising! (err: %d)", err);
    } else {
        LOG_INF("Comando BT_CMD_ADVERTISE enviado com sucesso.");
    }

    /* 3. Loop infinito do Main (Dorme para economizar bateria) */
    while (1) {
        k_sleep(K_SECONDS(10)); 
    }

    return 0;
}