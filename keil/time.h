#ifndef __TIME_H
#define __TIME_H		


#define delay1_ms(X)		delay_cycles( (CPUCLK_FREQ/1000) * (X) );

void delay_ms(unsigned long ms);
void delay_us(unsigned long __us);


#endif




