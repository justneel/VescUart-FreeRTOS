#ifndef _VESC_FREERTOS_h
#define _VESC_FREERTOS_h

// ESP-IDF puts the FreeRTOS headers under freertos/; most other ports put them on the include path directly.
#if defined(ESP_PLATFORM) || __has_include(<freertos/FreeRTOS.h>)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#else
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#endif

#endif
