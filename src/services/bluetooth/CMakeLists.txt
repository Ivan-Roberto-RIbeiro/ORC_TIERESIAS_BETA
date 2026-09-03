zephyr_library_sources(
    ble_telemetry.c
    ../imu_sensor/imu_sensor.c
    ../imu_sensor/head_tracking.c
)

zephyr_library_include_directories(
    .
    ../imu_sensor
)