#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
typedef struct { int locked; } StaticSemaphore_t;
#define configTICK_RATE_HZ 200U
#define portMAX_DELAY UINT32_MAX
#define pdTRUE 1
#endif
