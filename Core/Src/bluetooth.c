#include "bluetooth.h"

#include <stdbool.h>
#include <string.h>

#include "protos/request.pb-c.h"

#define COMMAND_CHANGE_NAME "+++\nAT+GAPDEVNAME=Milwaukee Vibe Tool\n+++\n"

#define HEADER_OFFSET_HEADER_START 0
#define HEADER_OFFSET_PAYLOAD_LENGTH 2
#define HEADER_OFFSET_CHECKSUM 4
#define HEADER_SIZE 6

#define GET_UINT16(b, i) ((uint16_t)((b[i+0] << 0) | (b[i+1] << 8)))

#define HEADER_START 0x0045

static void await_header(BluetoothController *controller);
static void await_partial_header(BluetoothController *controller);
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
//	await_header(controller);
//	for (int i = 0; i < 10; i++) {
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
//		HAL_Delay(1000);
//	}
	switch (controller->state) {
	case BLUETOOTH_WAITING_FOR_HEADER:
	case BLUETOOTH_WAITING_FOR_PARTIAL_HEADER:
		if (!verify_header(controller)) {
			await_partial_header(controller);
			break;
		}
		await_payload(controller);
		break;
	case BLUETOOTH_WAITING_FOR_PAYLOAD:
		if (!verify_checksum(controller)) {
			await_header(controller);
			break;
		}
		formulate_response(controller);
		await_header(controller);
		break;
	default:
		break;
	}
}

void bluetooth_uart_tx(BluetoothController *controller)
{
}

static void await_header(BluetoothController *controller)
{
	controller->state = BLUETOOTH_WAITING_FOR_HEADER;
	HAL_UART_Receive_DMA(controller->uart, controller->rx_buffer, HEADER_SIZE);
}

static void await_partial_header(BluetoothController *controller)
{
	controller->state = BLUETOOTH_WAITING_FOR_PARTIAL_HEADER;
	memmove(&controller->rx_buffer[0], &controller->rx_buffer[1], HEADER_SIZE-1);
	HAL_UART_Receive_DMA(controller->uart, &controller->rx_buffer[HEADER_SIZE-1], 1);
}

static void await_payload(BluetoothController *controller)
{
	controller->state = BLUETOOTH_WAITING_FOR_PAYLOAD;
	HAL_UART_Receive_DMA(controller->uart, &controller->rx_buffer[HEADER_SIZE], GET_UINT16(controller->rx_buffer, HEADER_OFFSET_PAYLOAD_LENGTH));
}

static bool verify_header(BluetoothController *controller)
{
	bool header_start_correct = GET_UINT16(controller->rx_buffer, HEADER_OFFSET_HEADER_START) == HEADER_START;
	bool payload_length_okay = GET_UINT16(controller->rx_buffer, HEADER_OFFSET_PAYLOAD_LENGTH) < BLUETOOTH_RX_BUFFER_SIZE;
	return header_start_correct && payload_length_okay;
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

