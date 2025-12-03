/*
 * myprintf.c
 * Modified for FreeRTOS Thread Safety
 */

#include "main.h"
#include "myprintf.h"
#include "cmsis_os.h"
#include <stdio.h>  
#include <stdarg.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern osMutexId_t printfMutexHandle;

PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void UART_Printf(const char *fmt, ...) {
    static char buffer[128]; 
    va_list args;

    if (printfMutexHandle != NULL) {
        osMutexAcquire(printfMutexHandle, osWaitForever);
    }

    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 100);
    }

    if (printfMutexHandle != NULL) {
        osMutexRelease(printfMutexHandle);
    }
}