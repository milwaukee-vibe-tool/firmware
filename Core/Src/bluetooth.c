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
static const float mock_values[32] = {
		4.000000,
		4.780361,
		5.530734,
		6.222281,
		6.828427,
		7.325878,
		7.695518,
		7.923141,
		8.000000,
		7.923141,
		7.695518,
		7.325878,
		6.828427,
		6.222281,
		5.530734,
		4.780361,
		4.000000,
		3.219639,
		2.469266,
		1.777719,
		1.171573,
		0.674122,
		0.304482,
		0.076859,
		0.000000,
		0.076859,
		0.304482,
		0.674122,
		1.171573,
		1.777719,
		2.469266,
		3.219639,
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

