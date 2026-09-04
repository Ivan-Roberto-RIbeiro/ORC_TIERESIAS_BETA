#include "sensor_maestro.h"
#include "../imu_sensor/head_tracking.h" /* Importa a estrutura de ângulos (verifique se o caminho está correto) */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_maestro, LOG_LEVEL_INF);

/* Canal de saída (Bluetooth) */
ZBUS_CHAN_DEFINE(sensor_batch_chan, struct orc_batch_msg_t, NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* Ouvinte para pegar os dados do Madgwick que estão vindo do head_tracking.c */
ZBUS_SUBSCRIBER_DEFINE(maestro_orient_sub, 4);
ZBUS_CHAN_DECLARE(orientation_chan);

K_SEM_DEFINE(timer_sem, 0, 1);

static void maestro_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&timer_sem);
}
K_TIMER_DEFINE(maestro_timer, maestro_timer_handler, NULL);

static void maestro_thread(void *arg1, void *arg2, void *arg3) {
    struct orc_batch_msg_t batch;
    int frame_count = 0;
    uint16_t time_counter = 0;
    
    /* Variável para guardar o último ângulo calculado conhecido */
    struct orientation_chan_msg orient_data = {0}; 
    const struct zbus_channel *chan;

    LOG_INF("Iniciando Maestro a 2000 Hz...");
    
    /* Conecta o Maestro para escutar o Madgwick */
    zbus_chan_add_obs(&orientation_chan, &maestro_orient_sub, K_MSEC(200));
    
    k_timer_start(&maestro_timer, K_USEC(500), K_USEC(500));

    while (1) {
        k_sem_take(&timer_sem, K_FOREVER);
        
        /* Tenta ler ângulos novos no Zbus. 
         * K_NO_WAIT garante que, se o Madgwick não calculou nada novo, 
         * o Maestro não atrasa a amostragem de 2000 Hz! */
       /* Lê DIRETAMENTE do canal o último valor salvo na memória, sem esperar 
             * notificação de Subscriber. Isso garante que a amostragem de 2000Hz 
             * sempre tenha um valor válido para transmitir. */
            zbus_chan_read(&orientation_chan, &orient_data, K_NO_WAIT);
        
        batch.frames[frame_count].timestamp = time_counter++;
        batch.frames[frame_count].emg1 = 0;
        batch.frames[frame_count].emg2 = 0;
        batch.frames[frame_count].mic1 = 0;
        batch.frames[frame_count].mic2 = 0;
        
        /* Injetamos o Pitch e Roll REAIS nos slots do IMU! 
         * Como o espaço (int16_t) não aceita float, multiplicamos por 10 
         * para preservar 1 casa decimal (ex: 45.7 graus viaja como 457) */
        batch.frames[frame_count].imu_x = (int16_t)(orient_data.pitch * 10);
        batch.frames[frame_count].imu_y = (int16_t)(orient_data.roll * 10);
        batch.frames[frame_count].imu_z = 0;

        frame_count++;

        /* Caminhão encheu? Despacha via Bluetooth! */
        if (frame_count >= 15) {
            zbus_chan_pub(&sensor_batch_chan, &batch, K_NO_WAIT);
            frame_count = 0;
        }
    }
}

K_THREAD_DEFINE(maestro_tid, 2048, maestro_thread, NULL, NULL, NULL, 2, 0, 0);