#include "key.h"

uint8_t Get_KEY(void)
 {
	 static uint8_t key_last=0,key_now=0;
	 static uint8_t key_long_flag=0;
	 key_last=key_now;
	 
	 if( DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_PIN_UP_PIN) == 0)			key_now=1;
	 else if( DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_PIN_LEFT_PIN) == 0)	key_now=2;
	 else if( DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_PIN_RIGHT_PIN) == 0)	key_now=3;
	 else if( DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_PIN_DOWN_PIN) == 0)	key_now=4;
	 else key_now=0;
	 
	 if(key_last!=0&&key_last==key_now)
	 {
		 if(key_long_flag<5)key_long_flag++;
	 }
	
	 if(key_last!=0&&key_now==0)
	 {
		 if(key_long_flag==5)
		 {
			 key_long_flag=0;
			 return key_last+10;
		 }
		 key_long_flag=0;
		 return key_last;
	 }
	 return 0;
 }