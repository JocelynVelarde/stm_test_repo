#ifndef HC_05_H
#define HC_05_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include "Constants.h"

void startHCrx(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void sendHC(const char *format, ...);
const char* HC05_GetData(void);

#endif /* HC_05_H */