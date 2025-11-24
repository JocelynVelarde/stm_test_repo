#ifndef ESC_H
#define ESC_H
#include "main.h"

void setEscSpeed_us(uint16_t pulse_us); // 1000–2000 us
void stopCarEsc(void);                  // ~1500 us neutral
uint16_t esc_apply_dir(uint16_t us);
void Esc_SetInvert(uint8_t invert);

#endif