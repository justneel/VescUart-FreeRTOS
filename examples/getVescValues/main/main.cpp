// Reads telemetry from a VESC on UART1 once a second and keeps it alive from a second task.
// Set the pins and baud rate to match your wiring and the VESC's UART app settings.

#include <VescIdfUart.h>
#include <VescUart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "example";

static VescIdfUart uart(UART_NUM_1, /* tx */ 17, /* rx */ 18, /* baud */ 115200);
static VescUart vesc(/* timeout_ms */ 100);

static void keepAliveTask(void *) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        // VescUart locks internally, so calling it from several tasks is safe.
        vesc.sendKeepalive();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() {
    ESP_ERROR_CHECK(uart.begin());
    vesc.setTransport(&uart);

    xTaskCreate(keepAliveTask, "vescKeepAlive", 2048, nullptr, 5, nullptr);

    for (;;) {
        VescUart::dataPackage values;
        // Blocks this task (without spinning) until the reply arrives or 100 ms pass.
        if (vesc.getVescValues(values)) {
            ESP_LOGI(TAG, "rpm=%.0f voltage=%.1f V motor current=%.2f A tach=%ld",
                     values.rpm, values.inpVoltage, values.avgMotorCurrent, values.tachometer);
        } else {
            ESP_LOGW(TAG, "No reply from VESC");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
