#ifndef SENSOR_MAESTRO_H
#define SENSOR_MAESTRO_H

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

/* Frame individual (16 bytes) */
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

/* O caminhão de dados (15 frames = 240 bytes) */
struct __attribute__((packed)) orc_batch_msg_t {
    struct orc_frame_t frames[15];
};

/* Declaramos o canal Zbus mestre para o resto do sistema enxergar */
ZBUS_CHAN_DECLARE(sensor_batch_chan);

#endif /* SENSOR_MAESTRO_H */