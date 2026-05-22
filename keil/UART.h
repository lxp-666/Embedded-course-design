// UART.h
#ifndef UART_H
#define UART_H
#include "ti_msp_dl_config.h"

#define UART_RX_BUFFER_SIZE 10

// 函数声明
void uart1_init(void);//串口初始化
void uart1_send_char(char ch);//串口发送单个字符
void uart1_send_string(char* str);//串口发送字符串
void uart_init(void);//串口初始化
void uart0_send_char(char ch);//串口发送单个字符
void uart0_send_string(char* str);//串口发送字符串
void clear_uart_data(void);//清除串口存储数据数组
void UART_0_INST_IRQHandler(void);// 串口的中断服务函数
void UART0_dispose(void);//串口0处理函数

#endif // UART_H