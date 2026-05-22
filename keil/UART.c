#include "ti_msp_dl_config.h"
#include <stdio.h>
// 定义接收缓冲区和相关变量
#define UART_RX_BUFFER_SIZE 10
volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint8_t Uart_Receive_num = 0;
volatile uint8_t Uart_Receive_delaytimers = 0;
volatile uint8_t uart_data;


void uart_init(void)
{
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void uart0_send_char(char ch)//串口发送单个字符
{
   
    while( DL_UART_isBusy(UART_0_INST) == true ); //当串口0忙的时候等待，不忙的时候再发送传进来的字符
  
    DL_UART_Main_transmitData(UART_0_INST, ch);  //发送单个字符
}



void uart0_send_string(char* str)//串口发送字符串
{
   
    while(*str!=0&&str!=0) //当前字符串地址不在结尾 并且 字符串首地址不为空
    {
        uart0_send_char(*str++);//发送字符串首地址中的字符，并且在发送完成之后首地址自增
    }
}


void clear_uart_data(void)//清除串口存储数据数组和指针延时
{
	unsigned char i;
	for(i=0;i<10;i++)uart_rx_buffer[i] = 0;
	Uart_Receive_num = 0;
	Uart_Receive_delaytimers = 0;
}



void UART_0_INST_IRQHandler(void)// 串口的中断服务函数
{
    uint32_t pendingInterrupt = DL_UART_getPendingInterrupt(UART_0_INST);
    switch(pendingInterrupt)
    {
        case DL_UART_IIDX_RX: // 如果是接收中断
            
            // 读取接收到的数据
            uart_data = DL_UART_Main_receiveData(UART_0_INST);
//		uart0_send_char(uart_data);//调试时使用  简单回显实现
		    uart_rx_buffer[Uart_Receive_num] = uart_data;
		    Uart_Receive_num++;
            if(Uart_Receive_num>10)
			{
				clear_uart_data();
			}
			else
			{
				Uart_Receive_delaytimers = 2;
			}
            break;
    }
}

void UART0_dispose(void)
{
	if(Uart_Receive_delaytimers != 0)
	{
		Uart_Receive_delaytimers--;
	}
	if(Uart_Receive_delaytimers == 0)
	{
		if(Uart_Receive_num >0)
		{
			if(uart_rx_buffer[0] == '1')DL_GPIO_togglePins(GPIO_LED1_PORT,GPIO_LED1_PIN_22_PIN);
			clear_uart_data();
		}
		else
		{
			clear_uart_data();
		}
	}
}

int fputc(int ch, FILE *stream)
{
        while( DL_UART_isBusy(UART_0_INST) == true );

        DL_UART_Main_transmitData(UART_0_INST, ch);

        return ch;
}


#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
        int handle;
};
#endif
FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
        x = x;
}
#endif