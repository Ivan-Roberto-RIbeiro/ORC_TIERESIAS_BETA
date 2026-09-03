#ifndef HEAD_TRACKING_H
#define HEAD_TRACKING_H
#include <zephyr/zbus/zbus.h>

#include <zephyr/kernel.h>

/* Estrutura de dados que sai do imu_sensor.c e vai para o head_tracking.c */
struct imu_data_chan_msg {
    float ax, ay, az;
    float gx, gy, gz;
};

/* Estrutura de dados que sai do head_tracking.c e vai para o BLE */
struct orientation_chan_msg {
    int16_t pitch_scaled;
    int16_t roll_scaled;
};

ZBUS_CHAN_DECLARE(imu_data_chan);
ZBUS_CHAN_DECLARE(orientation_chan);

#endif  