#include "head_tracking.h"
#include "zbus_common.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <math.h>

LOG_MODULE_REGISTER(head_tracking, LOG_LEVEL_INF);

/* === ZBUS === */
ZBUS_SUBSCRIBER_DEFINE(ht_sub, 4);

/* O canal que recebe os dados do imu_sensor.c */
ZBUS_CHAN_DECLARE(imu_data_chan);

/* O canal de saída (Pitch e Roll) que vai para o BLE */
ZBUS_CHAN_DEFINE(orientation_chan, 
                 struct orientation_chan_msg, 
                 NULL, NULL, 
                 ZBUS_OBSERVERS_EMPTY,
                 ZBUS_MSG_INIT(0));

/* quaternion */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* Ajuste o Beta */
#define beta 0.03f 

/* === Madgwick update (gyro + accel) === */
static void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/* === Thread === */
static void head_tracking_thread(void)
{
    LOG_INF("Head tracking thread started (Pitch/Roll Mode)");

    static int64_t last_ts = 0;
    const struct zbus_channel *chan;
    struct imu_data_chan_msg imu_msg;

    int ret = zbus_chan_add_obs(&imu_data_chan, &ht_sub, K_MSEC(100));
    if (ret != 0) {
        LOG_ERR("Failed to subscribe to imu_data_chan: %d", ret);
    }

    while (1) {
        ret = zbus_sub_wait(&ht_sub, &chan, K_FOREVER);
        if (ret != 0) {
            continue;
        }

        if (chan == &imu_data_chan) {
            ret = zbus_chan_read(chan, &imu_msg, K_MSEC(10));
            if (ret != 0) {
                continue;
            }

            float ax = imu_msg.ax;
            float ay = imu_msg.ay;
            float az = imu_msg.az;

            float gx = imu_msg.gx;
            float gy = imu_msg.gy;
            float gz = imu_msg.gz;

            int64_t now = k_uptime_get();
            float dt = (last_ts == 0) ? 0.01f : (now - last_ts) / 1000.0f;
            last_ts = now;

            /* Atualiza a malha 3D com o Madgwick */
            madgwick_update(gx, gy, gz, ax, ay, az, dt);

            /* CONVERSÃO QUATERNION -> EULER (Pitch e Roll) */
            float pitch_rad = asinf(2.0f * (q0 * q2 - q3 * q1));
            float roll_rad = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

            float pitch_deg = pitch_rad * (180.0f / 3.14159265359f);
            float roll_deg = roll_rad * (180.0f / 3.14159265359f);

            /* Empacota para envio no Zbus (x100 para preservar 2 casas decimais) */
            struct orientation_chan_msg msg = {
                .pitch_scaled = (int16_t)(pitch_deg * 100.0f),
                .roll_scaled  = (int16_t)(roll_deg * 100.0f)
            };

            ret = zbus_chan_pub(&orientation_chan, &msg, K_NO_WAIT);
            if (ret != 0) {
                LOG_ERR("Failed to publish orientation: %d", ret);
            }
        }
    }
}

K_THREAD_DEFINE(ht_thread_id, 2048, head_tracking_thread, NULL, NULL, NULL, 3, 0, 0);