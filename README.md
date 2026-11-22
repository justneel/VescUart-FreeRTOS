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
- Long packets (over 255 bytes) are sent and received, which reading and writing configurations needs.
- Reads and writes the VESC's motor and app configurations (see below).

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

## Writing VESC settings

`applySettings()` makes a VESC hold the values you give it, so a controller can set up its VESCs at startup instead of each one being configured in VESC Tool:

```cpp
const VescSetting motor[] = {
  {"l_current_max", 10.0},
  {"l_battery_cut_start", 33.0},
};
VescUart::ApplyResult result = UART.applySettings(VescConfKind::MCCONF, motor, 2);
ESP_LOGI("app", "settings: %s", VescUart::applyResultName(result));
```

It reads the configuration, changes the named fields, and writes it back only if any differ (saved to the VESC's flash, as VESC Tool's write buttons do), then reads it back to check. Every other byte stays as the VESC sent it, so motor detection results and anything you set in VESC Tool are kept. Each changed field is logged. Write while the motor is stopped.

- Names are the firmware's `mc_configuration` / `app_configuration` fields, which are also the names in VESC Tool's parameter XML. Values are in the firmware's units, which are not always VESC Tool's display units (for example, `foc_motor_r` is ohms, not milliohms).
- The configuration's layout changes with each firmware release. `src/VescConfLayouts.cpp` holds the layouts of releases 5.01 to 7.00, generated from the firmware's `confgenerator.c` by `tools/gen_conf_layouts.py`. Add a release to `REFS` there and run it again to support it. A VESC on a release that isn't listed gives `UNKNOWN_FIRMWARE`, and nothing is written.
- `getConfig()`, `VescConfig::get()` / `set()` and `setConfig()` are there for reading or writing fields yourself.
- Changing `app_to_use` or `app_uart_baudrate` can cut the UART link, which then needs VESC Tool over USB to restore.

