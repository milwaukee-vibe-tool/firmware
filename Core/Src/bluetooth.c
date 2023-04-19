#include "bluetooth.h"

#include <stdbool.h>
#include <string.h>

#define COMMAND_CHANGE_NAME "+++\nAT+GAPDEVNAME=Milwaukee Vibe Tool\n+++\n"

#define HEADER_OFFSET_HEADER_START 0
#define HEADER_OFFSET_PAYLOAD_LENGTH 2
#define HEADER_OFFSET_CHECKSUM 4
#define HEADER_SIZE 6

#define GET_UINT16(b, i) ((uint16_t)((b[i+0] << 0) | (b[i+1] << 8)))

#define HEADER_START 0x0045

// todo: replace with protobuf
typedef enum {
	REQUEST_GET_VALUES,
	REQUEST_TURN_ON_LED,
	REQUEST_TURN_OFF_LED,
	REQUEST_TOGGLE_LED,
} REQUESTS;

#define RESPONSE_ACK "ACK"

// todo: replace with real values
static const uint8_t mock_values[] = {
		0x64,0x69,0x6e,0x73,0x78,0x7c,0x81,0x86,
		0x8a,0x8f,0x93,0x97,0x9c,0xa0,0xa3,0xa7,
		0xab,0xae,0xb1,0xb4,0xb7,0xba,0xbc,0xbe,
		0xc0,0xc2,0xc4,0xc5,0xc6,0xc7,0xc8,0xc8,
		0xc8,0xc8,0xc8,0xc7,0xc6,0xc5,0xc4,0xc2,
		0xc0,0xbe,0xbc,0xba,0xb7,0xb4,0xb1,0xae,
		0xab,0xa7,0xa3,0xa0,0x9c,0x97,0x93,0x8f,
		0x8a,0x86,0x81,0x7c,0x78,0x73,0x6e,0x69,
		0x64,0x5f,0x5a,0x55,0x50,0x4c,0x47,0x42,
		0x3e,0x39,0x35,0x31,0x2c,0x28,0x25,0x21,
		0x1d,0x1a,0x17,0x14,0x11,0xe,0xc,0xa,
		0x8,0x6,0x4,0x3,0x2,0x1,0x0,0x0,
		0x0,0x0,0x0,0x1,0x2,0x3,0x4,0x6,
		0x8,0xa,0xc,0xe,0x11,0x14,0x17,0x1a,
		0x1d,0x21,0x25,0x28,0x2c,0x31,0x35,0x39,
		0x3e,0x42,0x47,0x4c,0x50,0x55,0x5a,0x5f,
};

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
	controller->tx_buffer[HEADER_OFFSET_HEADER_START] = HEADER_START;

	switch (controller->rx_buffer[HEADER_SIZE]) {
	case REQUEST_GET_VALUES:
		controller->tx_buffer[HEADER_OFFSET_PAYLOAD_LENGTH] = sizeof(mock_values);
		memcpy(&controller->tx_buffer[HEADER_SIZE], mock_values, sizeof(mock_values));
		break;
	case REQUEST_TURN_ON_LED:
		controller->tx_buffer[HEADER_OFFSET_PAYLOAD_LENGTH] = sizeof(RESPONSE_ACK);
		memcpy(&controller->tx_buffer[HEADER_SIZE], RESPONSE_ACK, sizeof(RESPONSE_ACK));
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
		break;
	case REQUEST_TURN_OFF_LED:
		controller->tx_buffer[HEADER_OFFSET_PAYLOAD_LENGTH] = sizeof(RESPONSE_ACK);
		memcpy(&controller->tx_buffer[HEADER_SIZE], RESPONSE_ACK, sizeof(RESPONSE_ACK));
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
		break;
	case REQUEST_TOGGLE_LED:
		controller->tx_buffer[HEADER_OFFSET_PAYLOAD_LENGTH] = sizeof(RESPONSE_ACK);
		memcpy(&controller->tx_buffer[HEADER_SIZE], RESPONSE_ACK, sizeof(RESPONSE_ACK));
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		break;
	}

	HAL_UART_Transmit(controller->uart, controller->tx_buffer, HEADER_SIZE + GET_UINT16(controller->tx_buffer, HEADER_OFFSET_PAYLOAD_LENGTH), -1);
}

