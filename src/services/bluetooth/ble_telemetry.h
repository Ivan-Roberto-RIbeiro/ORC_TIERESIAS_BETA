#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

/* Assinaturas das funções para o arquivo .c se reconhecer */
void ble_connected_cb(void);
void ble_disconnected_cb(void);
int ble_init(void (*connected_cb)(void), void (*disconnected_cb)(void));
int ble_start_advertising(void);
int ble_stop_advertising(void);

#endif /* BLE_TELEMETRY_H */