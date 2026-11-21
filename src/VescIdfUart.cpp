#if defined(ESP_PLATFORM)

#include "VescIdfUart.h"

#include <esp_log.h>

static const char * TAG = "VescIdfUart";

VescIdfUart::VescIdfUart(uart_port_t port, int txPin, int rxPin, int baud, int rxBufferSize)
	: port(port), txPin(txPin), rxPin(rxPin), baud(baud), rxBufferSize(rxBufferSize) {
}

esp_err_t VescIdfUart::begin() {
	uart_config_t config = {};
	config.baud_rate = baud;
	config.data_bits = UART_DATA_8_BITS;
	config.parity = UART_PARITY_DISABLE;
	config.stop_bits = UART_STOP_BITS_1;
	config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	config.source_clk = UART_SCLK_DEFAULT;

	esp_err_t err = uart_driver_install(port, rxBufferSize, 0, 0, NULL, 0);
	if (err == ESP_OK) err = uart_param_config(port, &config);
	if (err == ESP_OK) err = uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	if (err == ESP_OK) {
		ESP_LOGI(TAG, "UART%d started at %d baud (tx=%d rx=%d)", (int) port, baud, txPin, rxPin);
	} else {
		ESP_LOGE(TAG, "UART%d setup failed: %s", (int) port, esp_err_to_name(err));
	}
	return err;
}

int VescIdfUart::write(const uint8_t * data, size_t len) {
	return uart_write_bytes(port, data, len);
}

int VescIdfUart::read(uint8_t * data, size_t len, TickType_t timeout) {
	// Wait for the first byte, then take whatever else is already buffered without waiting.
	int got = uart_read_bytes(port, data, 1, timeout);
	if (got <= 0 || len == 1) {
		return got;
	}
	int more = uart_read_bytes(port, data + 1, len - 1, 0);
	return more < 0 ? got : got + more;
}

void VescIdfUart::flushInput() {
	uart_flush_input(port);
}

#endif // ESP_PLATFORM
