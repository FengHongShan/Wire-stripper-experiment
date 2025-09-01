#ifndef _BSP_USART_H_
#define _BSP_USART_H_
#include <stdio.h>
#include "usart.h"
#include "struct_typedef.h"
#define USART_REC_LEN  			10  	//定义最大接收字节数 200
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
#define RXBUFFERSIZE   1 //缓存大小
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer
extern u8 ch;
extern UART_HandleTypeDef huart1;
void Usart_SendString(uint8_t *str);
void ShowMessage(void);
#endif/**/

