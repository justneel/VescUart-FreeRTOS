#include <stdint.h>
#include <string.h>
#include "VescUart.h"

// Logging goes to ESP_LOG on ESP-IDF. On other FreeRTOS ports, define VESC_LOGD/I/W(fmt, ...)
// to send it somewhere; by default it is compiled out.
#if defined(ESP_PLATFORM)
	#include <esp_log.h>
	static const char * TAG = "VescUart";
	#define VESC_LOGD(...) ESP_LOGD(TAG, __VA_ARGS__)
	#define VESC_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
	#define VESC_LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#else
	#ifndef VESC_LOGD
		#define VESC_LOGD(...) ((void) 0)
	#endif
	#ifndef VESC_LOGI
		#define VESC_LOGI(...) ((void) 0)
	#endif
	#ifndef VESC_LOGW
		#define VESC_LOGW(...) ((void) 0)
	#endif
#endif

// Holds the VescUart mutex for one scope. It is recursive, so locked methods can call each other.
class Lock {
	public:
		explicit Lock(SemaphoreHandle_t mutex) : mutex(mutex) { xSemaphoreTakeRecursive(mutex, portMAX_DELAY); }
		~Lock() { xSemaphoreGiveRecursive(mutex); }
	private:
		SemaphoreHandle_t mutex;
};

VescUart::VescUart(uint32_t timeout_ms) : _TIMEOUT(timeout_ms) {
	nunchuck.valueX         = 127;
	nunchuck.valueY         = 127;
	nunchuck.lowerButton  	= false;
	nunchuck.upperButton  	= false;
	memset(&data, 0, sizeof(data));
	memset(&fw_version, 0, sizeof(fw_version));
#if configSUPPORT_STATIC_ALLOCATION
	mutex = xSemaphoreCreateRecursiveMutexStatic(&mutexBuffer);
#else
	mutex = xSemaphoreCreateRecursiveMutex();
	configASSERT(mutex != NULL);
#endif
}

VescUart::~VescUart() {
	vSemaphoreDelete(mutex);
}

void VescUart::setTransport(VescTransport* port)
{
	Lock lock(mutex);
	serialPort = port;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	// Makes no sense to run this function if no serialPort is defined.
	if (serialPort == NULL)
		return -1;

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	
	TimeOut_t timeOut;
	TickType_t remaining = pdMS_TO_TICKS(_TIMEOUT);
	vTaskSetTimeOutState(&timeOut);

	while (messageRead == false && xTaskCheckForTimeOut(&timeOut, &remaining) == pdFALSE) {

		// Blocks this task until a byte arrives or the time left runs out.
		uint8_t byte;
		if (serialPort->read(&byte, 1, remaining) == 1) {

			// Skip bytes until the start of a short packet.
			if (counter == 0 && byte != 2) {
				if (byte == 3) {
					VESC_LOGW("Message is larger than 256 bytes - not supported");
				} else {
					VESC_LOGD("Unvalid start bit %u", byte);
				}
				continue;
			}

			messageReceived[counter++] = byte;

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
					break;

					default:
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] != 3) {
				VESC_LOGD("Invalid end byte %u", messageReceived[endMessage - 1]);
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				VESC_LOGD("End of message reached!");
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false) {
		VESC_LOGD("Timeout");
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked) {
		// Message was read
		return lenPayload; 
	}
	else {
		// No Message Read
		return 0;
	}
}


bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	VESC_LOGD("SRC received: %u", crcMessage);

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	VESC_LOGD("SRC calc: %u", crcPayload);
	
	if (crcPayload == crcMessage) {
		VESC_LOGD("Received payload of %u bytes, packet id %u", message[1], payload[0]);
		return true;
	}else{
		return false;
	}
}


int VescUart::packSendPayload(uint8_t * payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];
	
	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lenPay);
	count += lenPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	// messageSend[count] = NULL;
	
	VESC_LOGD("Package to send: packet id %u, %d bytes", payload[0], count);

	// Sending package
	if( serialPort != NULL )
		serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}


bool VescUart::processReadPacket(uint8_t * message) {

	COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			fw_version.major = message[index++];
			fw_version.minor = message[index++];
			return true;
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			data.tempMosfet 		= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_fet_filtered()
			data.tempMotor 			= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_motor_filtered()
			data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_motor_current()
			data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_input_current()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_id()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_iq()
			data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &index); 	// 2 bytes - mc_interface_get_duty_cycle_now()
			data.rpm 				= buffer_get_float32(message, 1.0, &index);		// 4 bytes - mc_interface_get_rpm()
			data.inpVoltage 		= buffer_get_float16(message, 10.0, &index);		// 2 bytes - GET_INPUT_VOLTAGE()
			data.ampHours 			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours(false)
			data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours_charged(false)
			data.wattHours			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours(false)
			data.wattHoursCharged	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours_charged(false)
			data.tachometer 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_value(false)
			data.tachometerAbs 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_abs_value(false)
			data.error 				= (mc_fault_code)message[index++];								// 1 byte  - mc_interface_get_fault()
			data.pidPos				= buffer_get_float32(message, 1000000.0, &index);	// 4 bytes - mc_interface_get_pid_pos_now()
			data.id					= message[index++];								// 1 byte  - app_get_configuration()->controller_id	

			return true;

		break;

		/* case COMM_GET_VALUES_SELECTIVE:

			uint32_t mask = 0xFFFFFFFF; */

		default:
			return false;
		break;
	}
}

bool VescUart::getFWversion(void){
	return getFWversion(0);
}

bool VescUart::getFWversion(uint8_t canId){
	Lock lock(mutex);
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_FW_VERSION };

	if (serialPort != NULL)
		serialPort->flushInput(); // Drop a late reply to an earlier request
	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);
	if (messageLength > 2 && message[0] == COMM_FW_VERSION) { 
		return processReadPacket(message); 
	}
	return false;
}

bool VescUart::getVescValues(void) {
	return getVescValues(0);
}

bool VescUart::getVescValues(uint8_t canId) {
	Lock lock(mutex);
	VESC_LOGD("Command: COMM_GET_VALUES %u", canId);

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_GET_VALUES };

	if (serialPort != NULL)
		serialPort->flushInput(); // Drop a late reply to an earlier request
	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	if (messageLength > 55 && message[0] == COMM_GET_VALUES) {
		return processReadPacket(message); 
	}
	return false;
}

bool VescUart::getVescValues(dataPackage& out, uint8_t canId) {
	Lock lock(mutex);
	if (!getVescValues(canId)) {
		return false;
	}
	out = data;
	return true;
}
void VescUart::setNunchuckValues() {
	return setNunchuckValues(0);
}

void VescUart::setNunchuckValues(uint8_t canId) {

	Lock lock(mutex);
	VESC_LOGD("Command: COMM_SET_CHUCK_DATA %u", canId);
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 11 : 13);
	uint8_t payload[payloadSize];

	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CHUCK_DATA };
	payload[index++] = nunchuck.valueX;
	payload[index++] = nunchuck.valueY;
	buffer_append_bool(payload, nunchuck.lowerButton, &index);
	buffer_append_bool(payload, nunchuck.upperButton, &index);
	
	// Acceleration Data. Not used, Int16 (2 byte)
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;

	VESC_LOGD("Nunchuck Values: x=%d y=%d LBTN=%d UBTN=%d", nunchuck.valueX, nunchuck.valueY, nunchuck.lowerButton, nunchuck.upperButton);

	packSendPayload(payload, payloadSize);
}

void VescUart::setCurrent(float current) {
	return setCurrent(current, 0);
}

void VescUart::setCurrent(float current, uint8_t canId) {
	Lock lock(mutex);
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CURRENT };
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
	return setBrakeCurrent(brakeCurrent, 0);
}

void VescUart::setBrakeCurrent(float brakeCurrent, uint8_t canId) {
	Lock lock(mutex);
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_CURRENT_BRAKE };
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::setRPM(float rpm) {
	return setRPM(rpm, 0);
}

void VescUart::setRPM(float rpm, uint8_t canId) {
	Lock lock(mutex);
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setDuty(float duty) {
	return setDuty(duty, 0);
}

void VescUart::setDuty(float duty, uint8_t canId) {
	Lock lock(mutex);
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_DUTY };
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::sendKeepalive(void) {
	return sendKeepalive(0);
}

void VescUart::sendKeepalive(uint8_t canId) {
	Lock lock(mutex);
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_ALIVE };
	packSendPayload(payload, payloadSize);
}

void VescUart::printVescValues() {
	Lock lock(mutex);
	VESC_LOGI("avgMotorCurrent: %.2f", data.avgMotorCurrent);
	VESC_LOGI("avgInputCurrent: %.2f", data.avgInputCurrent);
	VESC_LOGI("dutyCycleNow: %.2f", data.dutyCycleNow);
	VESC_LOGI("rpm: %.2f", data.rpm);
	VESC_LOGI("inputVoltage: %.2f", data.inpVoltage);
	VESC_LOGI("ampHours: %.2f", data.ampHours);
	VESC_LOGI("ampHoursCharged: %.2f", data.ampHoursCharged);
	VESC_LOGI("wattHours: %.2f", data.wattHours);
	VESC_LOGI("wattHoursCharged: %.2f", data.wattHoursCharged);
	VESC_LOGI("tachometer: %ld", data.tachometer);
	VESC_LOGI("tachometerAbs: %ld", data.tachometerAbs);
	VESC_LOGI("tempMosfet: %.2f", data.tempMosfet);
	VESC_LOGI("tempMotor: %.2f", data.tempMotor);
	VESC_LOGI("error: %d", (int) data.error);
}
