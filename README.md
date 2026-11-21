# VescUart-FreeRTOS

FreeRTOS / ESP-IDF fork of [SolidGeek/VescUart](https://github.com/SolidGeek/VescUart), the Arduino library for interfacing with a VESC over UART. This library is based upon the works of many. The library is tested on a Teensy 4, and is updated for VESC firmware FW5+. The library is not nessecary backwards compatible with older versions of VESC firmware, so please update to the newest firmware available to your VESC.

The library supports only a small amount of features available to the VESC. You are welcome to make a pull request if you integrate new functionality and I will do my best to merge. 

## CAN BUS support

CAN BUS is now supported, such that you can communcate with multiple VESCs over a single UART port. All methods can be called with a CAN ID, and the main VESC will forward the command to the desired CAN Id. 

You can't use a CAN bus ID of 0 for this library, as this is used to refer to the local device; start numbering at 1.

## Changes from upstream

- No Arduino: the library talks to the UART through a small `VescTransport` interface (`write`, `read` with a FreeRTOS timeout, `flushInput`). `VescIdfUart` implements it for ESP-IDF; on other FreeRTOS ports, implement it for your UART driver.
- `setSerialPort(Stream*)` is now `setTransport(VescTransport*)`. `setDebugPort()` is gone: logging goes through ESP_LOG (tag `VescUart`), or through `VESC_LOGD`/`VESC_LOGI`/`VESC_LOGW` macros you define on other ports.
- Waiting for a reply blocks the task in the UART driver instead of busy-polling `available()`.
- Each `VescUart` has its own mutex, so several tasks can use it. `getVescValues(dataPackage& out)` returns a copy taken under that lock.
- The receiver skips bytes until a start byte, drops stale input before each request, and checks that the reply is the packet type it asked for.

## Usage

Add `https://github.com/justneel/VescUart-FreeRTOS.git` to `lib_deps` (PlatformIO, `framework = espidf`), or clone it into your ESP-IDF project's `components/` folder.

```cpp
#include <VescIdfUart.h>
#include <VescUart.h>

VescIdfUart uart(UART_NUM_1, /* tx */ 17, /* rx */ 18, /* baud */ 115200);
VescUart UART;

extern "C" void app_main() {
  ESP_ERROR_CHECK(uart.begin());
  UART.setTransport(&uart);
}
```

Getting VESC telemetry:

```cpp
VescUart::dataPackage values;
if ( UART.getVescValues(values) ) {
  ESP_LOGI("app", "rpm=%.0f voltage=%.1f ampHours=%.3f tachAbs=%ld",
           values.rpm, values.inpVoltage, values.ampHours, values.tachometerAbs);
}
```

You can find example usage and more information in the examples directory.  
  
