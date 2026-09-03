#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "head_tracking.h"
#include "zbus_common.h"

LOG_MODULE_REGISTER(bmi270_driver, LOG_LEVEL_INF);

const struct device *const bmi270_dev = DEVICE_DT_GET_ANY(bosch_bmi270);

ZBUS_CHAN_DEFINE(imu_data_chan,            
                  struct imu_data_chan_msg, 
                  NULL,                     
                  NULL,                     
                  ZBUS_OBSERVERS_EMPTY,     
                  ZBUS_MSG_INIT(.ax = 0.0f, .ay = 0.0f, .az = 0.0f, 
                                .gx = 0.0f, .gy = 0.0f, .gz = 0.0f)
);

static void imu_poll_thread(void *arg1, void *arg2, void *arg3)
{
    k_msleep(3000); 

    /* Tratamento de Erro: Verifica se o hardware respondeu no boot */
    if (!device_is_ready(bmi270_dev)) {
        LOG_ERR("FALHA: Dispositivo BMI270 nao esta pronto. Verifique o I2C.");
        return; 
    }

    /* Configuração de parâmetros */
    struct sensor_value odr_attr = {.val1 = 100, .val2 = 0};
    if (sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_WRN("AVISO: Falha ao configurar a frequencia do acelerometro.");
    }
    if (sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_WRN("AVISO: Falha ao configurar a frequencia do giroscopio.");
    }

    struct sensor_value accel[3], gyro[3];
    struct imu_data_chan_msg msg;

    LOG_INF("SUCESSO: Iniciando loop de aquisicao a 100Hz...");

    while (1) {
        /* 
         * Tratamento de Erro e Bateria: Se a leitura I2C falhar (fio solto, etc),
         * a thread entra em um sleep profundo (5s) antes de tentar novamente.
         * Isso evita que o processador fique em loop infinito lendo erros e drene a bateria.
         */
        if (sensor_sample_fetch(bmi270_dev) < 0) {
            LOG_ERR("FALHA I2C: Leitura corrompida. Dormindo para poupar bateria...");
            k_msleep(5000); 
            continue;
        }
        
        if (sensor_channel_get(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, accel) == 0 &&
            sensor_channel_get(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, gyro) == 0) {
            
            msg.ax = sensor_value_to_double(&accel[0]);
            msg.ay = sensor_value_to_double(&accel[1]);
            msg.az = sensor_value_to_double(&accel[2]);
            msg.gx = sensor_value_to_double(&gyro[0]);
            msg.gy = sensor_value_to_double(&gyro[1]);
            msg.gz = sensor_value_to_double(&gyro[2]);
            
            zbus_chan_pub(&imu_data_chan, &msg, K_NO_WAIT);
        }

        /* O yield cede o processador para outras tarefas, otimizando o consumo. */
        k_msleep(10); 
    }
}

/* Thread com prioridade elevada (4) para manter o dt do filtro estável */
K_THREAD_DEFINE(imu_poll_tid, 2048, imu_poll_thread, NULL, NULL, NULL, 4, 0, 0);