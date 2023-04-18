#include "bluetooth.h"

#include <stdbool.h>

#define COMMAND_CHANGE_NAME "+++\nAT+GAPDEVNAME=Milwaukee Vibe Monitoring yeet\n+++\n"

#define HEADER_OFFSET_HEADER_START 0
#define HEADER_OFFSET_PAYLOAD_LENGTH 2
#define HEADER_OFFSET_CHECKSUM 4
#define HEADER_SIZE 6

#define GET_UINT16(b, i) ((uint16_t)((b[i+0] << 0) | (b[i+1] << 1)))

#define HEADER_START 0x0045

static void await_header(BluetoothController *controller);
static void await_payload(BluetoothController *controller);

static bool verify_header(BluetoothController *controller);
static bool verify_checksum(BluetoothController *controller);

static void formulate_response(BluetoothController *controller);

HAL_StatusTypeDef bluetooth_init(BluetoothConfig *config, BluetoothController *controller)
{
	controller->uart= config->uart;
	controller->state = BLUETOOTH_NOT_READY;

	HAL_UART_Transmit(controller->uart, (uint8_t*)COMMAND_CHANGE_NAME, sizeof(COMMAND_CHANGE_NAME)-1, -1);

	await_header(controller);

	return HAL_OK;
}

HAL_StatusTypeDef bluetooth_run(BluetoothController *controller)
{
	return HAL_OK;
}

void bluetooth_uart_rx(BluetoothController *controller)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	await_header(controller);
//	for (int i = 0; i < 10; i++) {
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
//		HAL_Delay(1000);
//	}
//	switch (controller->state) {
//	case BLUETOOTH_WAITING_FOR_HEADER:
//		if (!verify_header(controller))
//			await_header(controller);
//		await_payload(controller);
//		break;
//	case BLUETOOTH_WAITING_FOR_PAYLOAD:
//		if (!verify_checksum(controller))
//			break;
//		formulate_response(controller);
//		await_header(controller);
//		break;
//	default:
//		break;
//	}
}

void bluetooth_uart_tx(BluetoothController *controller)
{
}

static void await_header(BluetoothController *controller)
{
	controller->state = BLUETOOTH_WAITING_FOR_HEADER;
	HAL_UART_Receive_DMA(controller->uart, controller->read_buffer, HEADER_SIZE);
}

static void await_payload(BluetoothController *controller)
{
	controller->state = BLUETOOTH_WAITING_FOR_PAYLOAD;
	HAL_UART_Receive_DMA(controller->uart, &controller->read_buffer[HEADER_SIZE], GET_UINT16(controller->read_buffer, HEADER_OFFSET_PAYLOAD_LENGTH));
}

static bool verify_header(BluetoothController *controller)
{
	return GET_UINT16(controller->read_buffer, HEADER_OFFSET_HEADER_START) == HEADER_START;
}

static bool verify_checksum(BluetoothController *controller)
{
	// todo
	return true;
}

static void formulate_response(BluetoothController *controller)
{
	for (int i = 0; i < 20; i++) {
	  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	  	  HAL_Delay(200);
	}
}

