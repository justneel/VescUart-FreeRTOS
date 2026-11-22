#ifndef _VESCUART_h
#define _VESCUART_h

#include "VescFreeRTOS.h"
#include "VescConfig.h"
#include "VescTransport.h"
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"

class VescUart
{

	public:
	/** Struct to store the telemetry data returned by the VESC */
	struct dataPackage {
       float avgMotorCurrent;
        float avgInputCurrent;
        float dutyCycleNow;
        float rpm;
        float inpVoltage;
        float ampHours;
        float ampHoursCharged;
        float wattHours;
        float wattHoursCharged;
        long tachometer;
        long tachometerAbs;
        float tempMosfet;
        float tempMotor;
        float pidPos;
        uint8_t id;
        mc_fault_code error; 
	};

	/** Struct to hold the nunchuck values to send over UART */
	struct nunchuckPackage {
		int	valueX;
		int	valueY;
		bool upperButton; // valUpperButton
		bool lowerButton; // valLowerButton
	};

    struct FWversionPackage {
        uint8_t major;
        uint8_t minor;
    };

	private:
	//Timeout - specifies how long the function will wait for the vesc to respond
	const uint32_t _TIMEOUT;

	public:
		/**
		 * @brief      Class constructor
		 */
		VescUart(uint32_t timeout_ms = 100);
		~VescUart();

		// Not copyable: a copy would share, and later delete, this object's mutex.
		VescUart(const VescUart&) = delete;
		VescUart& operator=(const VescUart&) = delete;

		/** Variabel to hold measurements returned from VESC */
		dataPackage data; 

		/** Variabel to hold nunchuck values */
		nunchuckPackage nunchuck; 

       /** Variable to hold firmware version */
        FWversionPackage fw_version; 

        /**
         * @brief      Set the transport (e.g. VescIdfUart) for uart communication
         * @param      port  - Must outlive this object
         */
        void setTransport(VescTransport* port);

        /**
         * @brief      Populate the firmware version variables
         *
         * @return     True if successfull otherwise false
         */
        bool getFWversion(void);

        /**
         * @brief      Populate the firmware version variables
         *
         * @param      canId  - The CAN ID of the VESC
         * @return     True if successfull otherwise false
         */
        bool getFWversion(uint8_t canId);

        /**
         * @brief      Sends a command to VESC and stores the returned data
         *
         * @return     True if successfull otherwise false
         */
        bool getVescValues(void);

        /**
         * @brief      Sends a command to VESC and stores the returned data
         * @param      canId  - The CAN ID of the VESC
         *
         * @return     True if successfull otherwise false
         */
        bool getVescValues(uint8_t canId);

        /**
         * @brief      Like getVescValues(canId), but also copies data to out while holding the
         *             lock, so another task can't change it in between
         */
        bool getVescValues(dataPackage& out, uint8_t canId = 0);

        /**
         * @brief      Sends values for joystick and buttons to the nunchuck app
         */
        void setNunchuckValues(void);
        /**
         * @brief      Sends values for joystick and buttons to the nunchuck app
         * @param      canId  - The CAN ID of the VESC
         */
        void setNunchuckValues(uint8_t canId);

        /**
         * @brief      Set the current to drive the motor
         * @param      current  - The current to apply
         */
        void setCurrent(float current);

        /**
         * @brief      Set the current to drive the motor
         * @param      current  - The current to apply
         * @param      canId  - The CAN ID of the VESC
         */
        void setCurrent(float current, uint8_t canId);

        /**
         * @brief      Set the current to brake the motor
         * @param      brakeCurrent  - The current to apply
         */
        void setBrakeCurrent(float brakeCurrent);

        /**
         * @brief      Set the current to brake the motor
         * @param      brakeCurrent  - The current to apply
         * @param      canId  - The CAN ID of the VESC
         */
        void setBrakeCurrent(float brakeCurrent, uint8_t canId);


        /**
         * @brief      Set the rpm of the motor
         * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
         */
        void setRPM(float rpm);		

        /**
         * @brief      Set the rpm of the motor
         * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
         * @param      canId  - The CAN ID of the VESC
         */
        void setRPM(float rpm, uint8_t canId);

        /**
         * @brief      Set the duty of the motor
         * @param      duty  - The desired duty (0.0-1.0)
         */
        void setDuty(float duty);

        /**
         * @brief      Set the duty of the motor
         * @param      duty  - The desired duty (0.0-1.0)
         * @param      canId  - The CAN ID of the VESC
         */
        void setDuty(float duty, uint8_t canId);

        /**
         * @brief      Send a keepalive message
         */
        void sendKeepalive(void);

        /**
         * @brief      Send a keepalive message
         * @param      canId  - The CAN ID of the VESC
         */
        void sendKeepalive(uint8_t canId);

        /** What applySettings() did. */
        enum class ApplyResult {
            UNCHANGED,        // Every setting already had its value; nothing was written
            WRITTEN,          // Changed settings were written, and reading back showed them
            NO_REPLY,         // The VESC did not send its configuration
            UNKNOWN_FIRMWARE, // The configuration's signature is not in VescConfLayouts.cpp
            UNKNOWN_SETTING,  // A setting name is not in this firmware's configuration
            BAD_VALUE,        // A value does not fit its field's encoding
            WRITE_FAILED,     // The VESC did not acknowledge the write
            VERIFY_FAILED,    // Reading back after the write did not show the new values
        };

        static const char * applyResultName(ApplyResult result);

        /**
         * @brief      Reads the motor (MCCONF) or app (APPCONF) configuration
         * @param      out  - Loaded with the VESC's blob; if the signature is unknown, isLoaded() is false
         *                    and out.signature() tells which one it was
         * @return     True if the VESC sent a configuration with a known layout
         */
        bool getConfig(VescConfKind kind, VescConfig& out, uint8_t canId = 0);

        /**
         * @brief      Writes a whole configuration, as VESC Tool's write buttons do, and waits for the
         *             VESC to acknowledge it. Start from a blob read with getConfig() and change fields
         *             with VescConfig::set(), so every other setting stays as it was.
         * @param      store  - Save to the VESC's flash. The motor configuration is always saved; for the
         *                      app configuration, false applies it only until the VESC restarts.
         * @return     True if the VESC acknowledged it
         */
        bool setConfig(const VescConfig& conf, bool store = true, uint8_t canId = 0);

        /**
         * @brief      Makes the VESC's configuration hold the given values. Reads the configuration,
         *             changes the named fields, and writes it (saving to flash) only if any of them
         *             differ, then reads it back to check. Each changed field is logged. Nothing is
         *             written if any name is unknown or any value does not fit.
         *             Write while the motor is stopped: the VESC pauses while it saves to flash.
         */
        ApplyResult applySettings(VescConfKind kind, const VescSetting* settings, size_t count, uint8_t canId = 0);

        /**
         * @brief      Scales the motor current limits (l_current_min / l_current_max) for now, without
         *             saving to flash, as VESC Tool's profiles do (COMM_SET_MCCONF_TEMP). The VESC goes
         *             back to its saved scales when it restarts. The ERPM, duty and power limits are
         *             read from the VESC and sent back unchanged. Safe while the motor runs.
         *             getConfig() reports the scaled values until the restart, so a later
         *             applySettings() write would save them.
         * @param      minScale  - Braking current scale, 0 to 1
         * @param      maxScale  - Drive current scale, 0 to 1
         * @return     True if the VESC acknowledged it
         */
        bool setTempCurrentScale(float minScale, float maxScale, uint8_t canId = 0);

        /**
         * @brief      Help Function to log struct dataPackage for Debug
         */
        void printVescValues(void);

	private: 

		/** Variabel to hold the reference to the transport to use for UART */
		VescTransport* serialPort = NULL;

		/** Serialises access from different tasks, so a reply goes to the task that asked for it */
		SemaphoreHandle_t mutex;
#if configSUPPORT_STATIC_ALLOCATION
		StaticSemaphore_t mutexBuffer;
#endif

		/**
		 * @brief      Packs the payload and sends it over Serial
		 *
		 * @param      payload  - The payload as a unit8_t Array with length of int lenPayload
		 * @param      lenPay   - Length of payload
		 * @return     The number of bytes send
		 */
		int packSendPayload(const uint8_t * payload, int lenPay);

		/**
		 * @brief      Receives one packet, short or long, and checks its CRC
		 *
		 * @param      payloadReceived  - Where the payload goes
		 * @param      maxPayload       - Size of payloadReceived; longer packets are dropped
		 * @param      timeoutMs        - How long to wait; 0 uses the constructor's timeout
		 * @return     The number of payload bytes, or 0 if none arrived intact
		 */
		int receiveUartMessage(uint8_t * payloadReceived, size_t maxPayload = 256, uint32_t timeoutMs = 0);

		/**
		 * @brief      Extracts the data from the received payload
		 *
		 * @param      message  - The payload to extract data from
		 * @return     True if the process was a success
		 */
		bool processReadPacket(uint8_t * message);

};

#endif
