#include "bluetooth.h"

#define COMMAND_CHANGE_NAME "AT+GAPDEVNAME=Milwaukee Vibe Tool\n"
#define COMMAND_CHANGE_MODE "+++\n"

static void start_uart_reception(BluetoothController *controller);

HAL_StatusTypeDef bluetooth_init(BluetoothConfig *config, BluetoothController *controller)
{
	controller->uart= config->uart;

	HAL_UART_Transmit_DMA(controller->uart, (uint8_t*)COMMAND_CHANGE_NAME, sizeof(COMMAND_CHANGE_NAME)-1);
	HAL_UART_Transmit_DMA(controller->uart, (uint8_t*)COMMAND_CHANGE_MODE, sizeof(COMMAND_CHANGE_MODE)-1);

	return HAL_OK;
}

HAL_StatusTypeDef bluetooth_run(BluetoothController *controller)
{
	return HAL_OK;
}

void bluetooth_uart_rx(BluetoothController *controller)
{

}

void bluetooth_uart_tx(BluetoothController *controller)
{

}
