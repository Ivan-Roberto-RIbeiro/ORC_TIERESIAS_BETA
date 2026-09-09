#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("Iniciando o Sistema orc-tieresias...");

    // O ble_telemetry_init() ou ble_init() já cuida do rádio.
    // O sensor_maestro inicializa sozinho via K_THREAD_DEFINE.

    LOG_INF("Sistema rodando. Aguardando conexao Python...");
    
    while (1) {
        k_sleep(K_FOREVER);
    }
    
    return 0;
}