#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
void UARTini();
void UARTcog();
void gui(uint8_t c);
void guichu(char *text);
char nhanchu(void);
void xulymangdulieu(uint32_t *diachimang,int xoa);
void Read_Uart(void);
void Send_Char( char *text);
char USART3_Write (char ch);
void Get_velo(void);
#endif
