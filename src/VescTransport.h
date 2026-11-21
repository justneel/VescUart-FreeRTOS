#ifndef _VESC_TRANSPORT_h
#define _VESC_TRANSPORT_h

#include <stddef.h>
#include <stdint.h>

#include "VescFreeRTOS.h"

/**
 * The byte link to a VESC. VescUart only talks to the VESC through this interface, so any
 * FreeRTOS port can use the library by implementing it for its own UART driver.
 * VescIdfUart (VescIdfUart.h) is the ready-made one for ESP-IDF.
 */
class VescTransport
{
	public:
		virtual ~VescTransport() {}

		/**
		 * @brief      Writes bytes to the VESC
		 * @return     Number of bytes written, or a negative value on error
		 */
		virtual int write(const uint8_t * data, size_t len) = 0;

		/**
		 * @brief      Reads up to len bytes, blocking the calling task for at most timeout ticks.
		 *             It may return as soon as any bytes are available.
		 * @return     Number of bytes read (0 on timeout), or a negative value on error
		 */
		virtual int read(uint8_t * data, size_t len, TickType_t timeout) = 0;

		/**
		 * @brief      Discards any received bytes that have not been read yet. VescUart calls it
		 *             before each request that expects a reply, so a late reply to an earlier
		 *             request can't be mistaken for the new one.
		 */
		virtual void flushInput() {}
};

#endif
