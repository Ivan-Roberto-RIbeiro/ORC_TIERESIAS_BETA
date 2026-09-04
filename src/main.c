#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("    Iniciando o orc-tieresias...        ");
    LOG_INF("========================================");

    int contador = 0;
    
    while (1) {
        LOG_INF("Main viva! Ping: %d", contador);
        k_msleep(1000); /* Pisca a cada 1 segundo */
    }

    return 0;
}