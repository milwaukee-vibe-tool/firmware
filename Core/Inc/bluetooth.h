#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "stm32f4xx_hal.h"

#define BLUETOOTH_TX_BUFFER_SIZE 2048
#define BLUETOOTH_RX_BUFFER_SIZE 2048

typedef struct {
	UART_HandleTypeDef *uart;
} BluetoothConfig;

typedef enum {
	BLUETOOTH_NOT_READY,
	BLUETOOTH_WAITING_FOR_HEADER,
	BLUETOOTH_WAITING_FOR_PARTIAL_HEADER,
	BLUETOOTH_WAITING_FOR_PAYLOAD,
} BluetoothState;

typedef struct {
	UART_HandleTypeDef *uart;
	BluetoothState state;
	uint8_t tx_buffer[BLUETOOTH_TX_BUFFER_SIZE];
	uint8_t rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
} BluetoothController;

HAL_StatusTypeDef bluetooth_init(BluetoothConfig *config, BluetoothController *controller);
HAL_StatusTypeDef bluetooth_run(BluetoothController *controller);
void bluetooth_uart_rx(BluetoothController *controller);
void bluetooth_uart_tx(BluetoothController *controller);

#endif
