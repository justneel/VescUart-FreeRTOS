#ifndef _VESC_IDF_UART_h
#define _VESC_IDF_UART_h

#if defined(ESP_PLATFORM)

#include <driver/uart.h>
#include <esp_err.h>

#include "VescTransport.h"

/**
 * VescTransport over an ESP-IDF UART. Reads block in the UART driver with a FreeRTOS timeout, so
 * a task waiting for a VESC reply sleeps instead of spinning.
 */
class VescIdfUart : public VescTransport
{
	public:
		/**
		 * @param      port          - UART to use, e.g. UART_NUM_1
		 * @param      txPin, rxPin  - GPIO numbers
		 * @param      baud          - Baud rate set in the VESC's UART app (VESC Tool defaults to 115200)
		 * @param      rxBufferSize  - Size of the driver's receive ring buffer, more than 128 bytes
		 */
		VescIdfUart(uart_port_t port, int txPin, int rxPin, int baud, int rxBufferSize = 1024);

		/**
		 * @brief      Installs and configures the UART driver. Call it once before using the VESC.
		 */
		esp_err_t begin();

		int write(const uint8_t * data, size_t len) override;
		int read(uint8_t * data, size_t len, TickType_t timeout) override;
		void flushInput() override;

	private:
		uart_port_t port;
		int txPin;
		int rxPin;
		int baud;
		int rxBufferSize;
};

#endif // ESP_PLATFORM

#endif
