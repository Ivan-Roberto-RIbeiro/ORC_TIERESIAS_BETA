#include "sensor_maestro.h"
#include "../imu_sensor/head_tracking.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(sensor_maestro, LOG_LEVEL_INF);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ==========================================
// CANAIS ZBUS (Tipagem compatível com o BLE)
// ==========================================
#define FRAMES_PER_PACKET 15
#define FRAME_SIZE 16
typedef uint8_t sensor_packet_type[FRAMES_PER_PACKET * FRAME_SIZE]; // Exatos 240 bytes

ZBUS_CHAN_DEFINE(sensor_batch_chan,
                 sensor_packet_type,
                 NULL, NULL,
                 ZBUS_OBSERVERS(),
                 ZBUS_MSG_INIT(0));

/* Ouvinte para pegar os dados do Madgwick */
ZBUS_SUBSCRIBER_DEFINE(maestro_orient_sub, 4);
ZBUS_CHAN_DECLARE(orientation_chan);

K_SEM_DEFINE(timer_sem, 0, 1);

static void maestro_timer_handler(struct k_timer *timer_id) {
    k_sem_give(&timer_sem);
}
K_TIMER_DEFINE(maestro_timer, maestro_timer_handler, NULL);

/* Função simples e direta para gerar o valor do microfone (300 Hz) */
static uint16_t gerar_seno_mic(uint32_t t) {
    float rad = 2.0f * (float)M_PI * 850.0f * ((float)t / 2000.0f);
    return (uint16_t)((sinf(rad) + 2.0f) * 2000.0f);
}

static void maestro_thread(void *arg1, void *arg2, void *arg3) {
    struct orc_batch_msg_t batch; 
    int frame_count = 0;
    uint16_t time_counter = 0;
    
    struct orientation_chan_msg orient_data = {0}; 

    LOG_INF("Iniciando Maestro a 2000 Hz com Sincronizacao Magic Byte...");
    
    zbus_chan_add_obs(&orientation_chan, &maestro_orient_sub, K_MSEC(200));
    k_timer_start(&maestro_timer, K_USEC(500), K_USEC(500));

    while (1) {
        k_sem_take(&timer_sem, K_FOREVER);
        
        // Lê o último ângulo conhecido sem atrasar o ciclo de 500us
        zbus_chan_read(&orientation_chan, &orient_data, K_NO_WAIT);
        
        uint32_t t = time_counter++;

        // MAGIC BYTE: O timestamp envia sempre 0x55AA para o Python nunca perder o alinhamento
        batch.frames[frame_count].timestamp = 0x55AA;
        
        batch.frames[frame_count].emg1 = 0;
        batch.frames[frame_count].emg2 = 0;
        
        // MIC 1 COM VALOR FIXO: Deve formar uma linha reta exata em 15000 no Python
        batch.frames[frame_count].mic2 = 15000;
        
        // MIC 2 COM A SENÓIDE: Deve formar uma onda limpa
        batch.frames[frame_count].mic1 = gerar_seno_mic(t);
        
        // Ângulos reais do IMU
        batch.frames[frame_count].imu_x = (int16_t)(orient_data.pitch * 10);
        batch.frames[frame_count].imu_y = (int16_t)(orient_data.roll * 10);
        batch.frames[frame_count].imu_z = 0;

        frame_count++;

        /* Caminhão encheu (15 frames = 240 bytes)? Despacha via Zbus! */
        if (frame_count >= 15) {
            zbus_chan_pub(&sensor_batch_chan, &batch, K_NO_WAIT);
            frame_count = 0;
        }
    }
}

K_THREAD_DEFINE(maestro_tid, 2048, maestro_thread, NULL, NULL, NULL, 2, 0, 0);