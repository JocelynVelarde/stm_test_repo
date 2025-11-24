#ifndef ESC_H
#define ESC_H
#include "main.h"

void setEscSpeed_us(uint16_t pulse_us); // 1000–2000 us
void stopCarEsc(void);                  // ~1500 us neutral
void hardStopCarEsc(void);            
void movePercent(int8_t percent);
void setEscNeutral(void);

#endif