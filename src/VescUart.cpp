#include <stdint.h>
#include <string.h>
#include <new>
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

// Reads exactly len bytes, or fewer if the time runs out. Returns the number read.
static int readExact(VescTransport * port, uint8_t * data, int len, TimeOut_t * timeOut, TickType_t * remaining) {
	int count = 0;
	while (count < len && xTaskCheckForTimeOut(timeOut, remaining) == pdFALSE) {
		int got = port->read(data + count, len - count, *remaining);
		if (got < 0) {
			break;
		}
		count += got;
	}
	return count;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived, size_t maxPayload, uint32_t timeoutMs) {

	// Short packets: 2, length (1 byte), payload, CRC (2 bytes), 3.
	// Long packets:  3, length (2 bytes), payload, CRC (2 bytes), 3.

	// Makes no sense to run this function if no serialPort is defined.
	if (serialPort == NULL)
		return -1;

	TimeOut_t timeOut;
	TickType_t remaining = pdMS_TO_TICKS(timeoutMs != 0 ? timeoutMs : _TIMEOUT);
	vTaskSetTimeOutState(&timeOut);

	// Skip bytes until a start byte.
	uint8_t start = 0;
	while (true) {
		if (readExact(serialPort, &start, 1, &timeOut, &remaining) != 1) {
			VESC_LOGD("Timeout");
			return 0;
		}
		if (start == 2 || start == 3) {
			break;
		}
		VESC_LOGD("Unvalid start bit %u", start);
	}

	uint8_t lengthBytes[2];
	const int lengthSize = start == 2 ? 1 : 2;
	if (readExact(serialPort, lengthBytes, lengthSize, &timeOut, &remaining) != lengthSize) {
		VESC_LOGD("Timeout");
		return 0;
	}
	const size_t lenPayload = lengthSize == 1 ? lengthBytes[0] : ((size_t) lengthBytes[0] << 8) | lengthBytes[1];
	if (lenPayload == 0 || lenPayload > maxPayload) {
		VESC_LOGW("Payload of %u bytes does not fit the %u byte buffer", (unsigned) lenPayload, (unsigned) maxPayload);
		return 0;
	}

	uint8_t footer[3];
	if (readExact(serialPort, payloadReceived, lenPayload, &timeOut, &remaining) != (int) lenPayload ||
		readExact(serialPort, footer, 3, &timeOut, &remaining) != 3) {
		VESC_LOGD("Timeout");
		return 0;
	}
	if (footer[2] != 3) {
		VESC_LOGD("Invalid end byte %u", footer[2]);
		return 0;
	}

	const uint16_t crcMessage = ((uint16_t) footer[0] << 8) | footer[1];
	const uint16_t crcPayload = crc16(payloadReceived, lenPayload);
	if (crcMessage != crcPayload) {
		VESC_LOGD("CRC mismatch: received %u, calculated %u", crcMessage, crcPayload);
		return 0;
	}

	VESC_LOGD("Received payload of %u bytes, packet id %u", (unsigned) lenPayload, payloadReceived[0]);
	return lenPayload;
}


int VescUart::packSendPayload(const uint8_t * payload, int lenPay) {

	if (lenPay <= 0 || lenPay > 0xFFFF) {
		VESC_LOGW("Payload of %d bytes can't be sent", lenPay);
		return 0;
	}

	uint16_t crcPayload = crc16(payload, lenPay);

	uint8_t header[3];
	int headerLength = 0;
	if (lenPay < 256) {
		header[headerLength++] = 2;
		header[headerLength++] = lenPay;
	} else {
		header[headerLength++] = 3;
		header[headerLength++] = (uint8_t)(lenPay >> 8);
		header[headerLength++] = (uint8_t)(lenPay & 0xFF);
	}

	const uint8_t footer[3] = { (uint8_t)(crcPayload >> 8), (uint8_t)(crcPayload & 0xFF), 3 };

	VESC_LOGD("Package to send: packet id %u, %d bytes", payload[0], headerLength + lenPay + 3);

	if (serialPort != NULL) {
		serialPort->write(header, headerLength);
		serialPort->write(payload, lenPay);
		serialPort->write(footer, sizeof(footer));
	}

	return headerLength + lenPay + 3;
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

// Packet ids from the VESC firmware's datatypes.h.
static const uint8_t PACKET_SET_MCCONF = 13;
static const uint8_t PACKET_GET_MCCONF = 14;
static const uint8_t PACKET_SET_APPCONF = 16;
static const uint8_t PACKET_GET_APPCONF = 17;
static const uint8_t PACKET_SET_APPCONF_NO_STORE = 149;

// Reading a configuration is quick; saving one erases a flash sector, which can take a second or two.
static const uint32_t CONFIG_READ_TIMEOUT_MS = 1000;
static const uint32_t CONFIG_WRITE_TIMEOUT_MS = 4000;

static const char * kindName(VescConfKind kind) {
	return kind == VescConfKind::MCCONF ? "motor config" : "app config";
}

const char * VescUart::applyResultName(ApplyResult result) {
	switch (result) {
		case ApplyResult::UNCHANGED: return "unchanged";
		case ApplyResult::WRITTEN: return "written";
		case ApplyResult::NO_REPLY: return "no reply";
		case ApplyResult::UNKNOWN_FIRMWARE: return "unknown firmware";
		case ApplyResult::UNKNOWN_SETTING: return "unknown setting";
		case ApplyResult::BAD_VALUE: return "bad value";
		case ApplyResult::WRITE_FAILED: return "write failed";
		case ApplyResult::VERIFY_FAILED: return "verify failed";
	}
	return "?";
}

bool VescUart::getConfig(VescConfKind kind, VescConfig& out, uint8_t canId) {
	Lock lock(mutex);
	const uint8_t packetId = kind == VescConfKind::MCCONF ? PACKET_GET_MCCONF : PACKET_GET_APPCONF;

	uint8_t request[3];
	int length = 0;
	if (canId != 0) {
		request[length++] = COMM_FORWARD_CAN;
		request[length++] = canId;
	}
	request[length++] = packetId;

	uint8_t * reply = new (std::nothrow) uint8_t[VescConfig::MAX_SIZE + 1];
	if (reply == nullptr) {
		return false;
	}

	if (serialPort != NULL)
		serialPort->flushInput(); // Drop a late reply to an earlier request
	packSendPayload(request, length);
	const int replyLength = receiveUartMessage(reply, VescConfig::MAX_SIZE + 1, CONFIG_READ_TIMEOUT_MS);

	bool loaded = false;
	if (replyLength > 1 && reply[0] == packetId) {
		loaded = out.load(kind, reply + 1, replyLength - 1);
		if (!loaded) {
			VESC_LOGW("Unknown %s layout: signature %lu, %d bytes", kindName(kind),
				(unsigned long) out.signature(), replyLength - 1);
		}
	}
	delete[] reply;
	return loaded;
}

bool VescUart::setConfig(const VescConfig& conf, bool store, uint8_t canId) {
	if (!conf.isLoaded()) {
		return false;
	}
	Lock lock(mutex);

	uint8_t packetId;
	if (conf.kind() == VescConfKind::MCCONF) {
		packetId = PACKET_SET_MCCONF;
	} else {
		packetId = store ? PACKET_SET_APPCONF : PACKET_SET_APPCONF_NO_STORE;
	}

	const size_t length = conf.size() + 3;
	uint8_t * request = new (std::nothrow) uint8_t[length];
	if (request == nullptr) {
		return false;
	}
	int index = 0;
	if (canId != 0) {
		request[index++] = COMM_FORWARD_CAN;
		request[index++] = canId;
	}
	request[index++] = packetId;
	memcpy(request + index, conf.data(), conf.size());
	index += conf.size();

	if (serialPort != NULL)
		serialPort->flushInput();
	packSendPayload(request, index);
	delete[] request;

	uint8_t reply[16];
	const int replyLength = receiveUartMessage(reply, sizeof(reply), CONFIG_WRITE_TIMEOUT_MS);
	return replyLength >= 1 && reply[0] == packetId;
}

VescUart::ApplyResult VescUart::applySettings(VescConfKind kind, const VescSetting* settings, size_t count, uint8_t canId) {
	Lock lock(mutex);

	// About 1 KB each, so keep them off the calling task's stack.
	VescConfig * conf = new (std::nothrow) VescConfig();
	VescConfig * check = new (std::nothrow) VescConfig();
	ApplyResult result = ApplyResult::NO_REPLY;
	bool anyChanged = false;

	if (conf == nullptr || check == nullptr) {
		goto done;
	}

	if (!getConfig(kind, *conf, canId)) {
		result = conf->signature() != 0 ? ApplyResult::UNKNOWN_FIRMWARE : ApplyResult::NO_REPLY;
		goto done;
	}
	VESC_LOGI("Read %s, firmware %s", kindName(kind), conf->layout()->firmware);

	for (size_t i = 0; i < count; i++) {
		double before = 0;
		if (!conf->get(settings[i].name, before)) {
			VESC_LOGW("%s has no setting %s", kindName(kind), settings[i].name);
			result = ApplyResult::UNKNOWN_SETTING;
			goto done;
		}
		bool changed = false;
		if (!conf->set(settings[i].name, settings[i].value, &changed)) {
			VESC_LOGW("%s = %g does not fit its field", settings[i].name, settings[i].value);
			result = ApplyResult::BAD_VALUE;
			goto done;
		}
		if (changed) {
			double after = 0;
			conf->get(settings[i].name, after);
			VESC_LOGI("%s: %g -> %g", settings[i].name, before, after);
			anyChanged = true;
		}
	}

	if (!anyChanged) {
		result = ApplyResult::UNCHANGED;
		goto done;
	}

	if (!setConfig(*conf, true, canId)) {
		VESC_LOGW("The VESC did not acknowledge the %s write", kindName(kind));
		result = ApplyResult::WRITE_FAILED;
		goto done;
	}

	// The firmware can clamp values to the hardware's limits, so read back what it kept.
	result = ApplyResult::WRITTEN;
	if (!getConfig(kind, *check, canId)) {
		result = ApplyResult::VERIFY_FAILED;
		goto done;
	}
	for (size_t i = 0; i < count; i++) {
		double kept = 0;
		bool differs = false;
		check->get(settings[i].name, kept);
		check->set(settings[i].name, settings[i].value, &differs);
		if (differs) {
			VESC_LOGW("%s: asked for %g, the VESC kept %g", settings[i].name, settings[i].value, kept);
			result = ApplyResult::VERIFY_FAILED;
		}
	}

done:
	delete conf;
	delete check;
	return result;
}
