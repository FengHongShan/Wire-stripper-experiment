#ifndef _BSP_USART_H_
#define _BSP_USART_H_
#include <stdio.h>
#include "usart.h"
#include "struct_typedef.h"
#define USART_REC_LEN  			10  	//�����������ֽ��� 200
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
#define RXBUFFERSIZE   1 //�����С
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
extern u8 ch;
extern UART_HandleTypeDef huart1;
void Usart_SendString(uint8_t *str);
void ShowMessage(void);
#endif/**/

