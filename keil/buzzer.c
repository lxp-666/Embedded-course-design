#include "buzzer.h"
#include "time.h"

void buzzer_beep(uint16_t duration_ms)
{
    BUZZER_ON();
    delay1_ms(duration_ms);
    BUZZER_OFF();
}

void buzzer_startup_sound(void)
{
    int i;
    for (i = 0; i < 3; i++) {
        BUZZER_ON();
        delay_ms(80);
        BUZZER_OFF();
        delay_ms(60);
    }

    delay_ms(120);

    BUZZER_ON();
    delay_ms(300);
    BUZZER_OFF();
}
