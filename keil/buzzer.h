#ifndef __BUZZER_H
#define __BUZZER_H

#include "ti_msp_dl_config.h"
#include "stdint.h"

/* 蜂鸣器：PB13，有源低电平触发 */
#define BUZZER_PORT    (GPIOB)
#define BUZZER_PIN     (DL_GPIO_PIN_13)

#define BUZZER_ON()    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN)
#define BUZZER_OFF()   DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN)

void buzzer_beep(uint16_t duration_ms);
void buzzer_startup_sound(void);

#endif
