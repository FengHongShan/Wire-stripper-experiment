#include "sensor.h"

void Sensor_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	

  GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;  
  GPIO_InitStruct.Pin=GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_RESET);
	
	
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;  
  GPIO_InitStruct.Pin=GPIO_PIN_8;
  GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
	
}



