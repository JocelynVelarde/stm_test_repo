/*
 * myprintf.h
 */

#ifndef INC_MYPRINTF_H_
#define INC_MYPRINTF_H_

#include <stdio.h>

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

void UART_Printf(const char *fmt, ...);
#define printf(...) UART_Printf(__VA_ARGS__)

#endif /* INC_MYPRINTF_H_ */