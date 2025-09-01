/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "CAN_receive.h"
#include "bsp_usart.h"
#include "sensor.h"
#include "single_nerve_pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float PID_speed[3]={6,0.01,0.1};
float PID_position[3]={ 0.3f, 0, 3.0f};
float PID_speed_F[3] = {5.0,0.01,0.1};
//#define one_circle 19.203*8192
pid_type_def pid_speed[3];/*pid*/
pid_type_def pid_position[3];
NERVEPID  npid_speed[3];/*single_nerve_pid*/
const motor_measure_t * motor_data0;
const motor_measure_t * motor_data1;
const motor_measure_t * motor_data2;
float set_speed_0 =0 ;  //0号电机角度值
float set_speed_1 =0 ;	//1号电机速度值
float set_speed_2 =0 ;	//2号电机速度值
float set_angle_0 =0 ; 	//0号电机角度值
float set_angle_1 =0 ; 	//1号电机角度值
float set_angle_2 =0 ; 	//2号电机角度值
float set_speed =0;			//2号电机角度值
float m = 400;//旋转电机加减速用
uint32_t i=0;//位置环pid计算周期
uint16_t p=0;
uint8_t speed_flag_0=0;//0号电机速度标志位
uint8_t angle_flag_0=0;//0号电机角度标志位
uint8_t speed_flag_1=0;//1号电机速度标志位
uint8_t angle_flag_1=0;//1号电机角度标志位
uint8_t speed_flag_2=0;//2号电机速度标志位
uint8_t angle_flag_2=0;//2号电机角度标志位
uint8_t Sensor_flag=0;//传感器标志位
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==htim1.Instance)
	{
	if(speed_flag_2==1)//速度环
		{
			//2号电机
//      set_speed_2=0;
      PID_calc(&pid_speed[2], motor_data2->speed_rpm, set_speed_2);//进给电机
			if(Sensor_flag==1)
			{
			 if(ReadSensor!=GPIO_PIN_RESET)//如果没检测到线芯
			 {
 
			  set_speed_2 =-500;
			  PID_calc(&pid_speed[2], motor_data2->speed_rpm, set_speed_2);//进给电机
        
			 }
			 else
			 {
         speed_flag_2=2;
////         Sensor_flag=0;
//			   set_speed_2 =0; 
//			  PID_calc(&pid_speed[2], motor_data2->speed_rpm, set_speed_2);//进给电机
			 }
     }

   }


    
    if(speed_flag_2==2)//传感器检测一次进行关闭
		{
//       Sensor_flag=0;
       set_speed_2=0;
       PID_calc(&pid_speed[2], motor_data2->speed_rpm, set_speed_2);//进给电机  
    }

/*---------------------------------------------------------------------------------------------------*/
		if(angle_flag_2==1)//位置环
		{
			if((i++%3)==0)
			{
				  PID_calc(&pid_position[2], motor_data2->total_angle, set_angle_2);
			}
			if(pid_position[2].out>5000)
			{
				pid_position[2].out = 5000;
			}
			else if(pid_position[2].out<-5000)
			{
				pid_position[2].out = -5000;
			}
			PID_calc(&pid_speed[2], motor_data2->speed_rpm, pid_position[2].out);

		
		}
	}
  /************************************************************************************/
	
	if(htim->Instance==htim2.Instance)
	{
		if(speed_flag_0==1)//速度环
		{
			    //   NERVEPID_cal(&npid_speed[0], set_speed_0,motor_data0->speed_rpm);/*单神经元pid*/
			 PID_calc(&pid_speed[0], motor_data0->speed_rpm, set_speed_0);//主电机/*传统pid
		}
  /*------------------------------------------------------------------------------*/
		if(angle_flag_0==1)//位置环
		{
			if((i++%3)==0)
			{
				  PID_calc(&pid_position[0], motor_data0->total_angle, set_angle_0);
			}
			if(pid_position[0].out>5000)
			{
				pid_position[0].out = 5000;
			}
			else if(pid_position[0].out<-5000)
			{
				pid_position[0].out = -5000;
			}
			PID_calc(&pid_speed[0], motor_data0->speed_rpm, pid_position[0].out);
			}
		
	}
	
/*************************************************************************************/
	if(htim->Instance==htim3.Instance)
	{
		if(speed_flag_1==1)//速度环
		{
			//1号电机
		 PID_calc(&pid_speed[1], motor_data1->speed_rpm, set_speed_1);//夹紧电机
		}
/*------------------------------------------------------------------------------------*/
		if(angle_flag_1==1)//位置环
		{
			if((i++%3)==0)
			{
				  PID_calc(&pid_position[1], motor_data1->total_angle, set_angle_1);
			}
			if(pid_position[1].out>5000)
			{
				pid_position[1].out = 5000;
			}
			else if(pid_position[1].out<-5000)
			{
				pid_position[1].out = -5000;
			}
			PID_calc(&pid_speed[1], motor_data1->speed_rpm, pid_position[1].out);
		
		}
	}
/************************************************************************************/
	if(htim->Instance==htim4.Instance)
	{
	
    
    
//    
//    if((motor_data2->total_angle)==0)
//    {
//      angle_flag_2=0;
//     
//    }
      
    
    
}
				CAN_cmd_chassis(pid_speed[0].out,pid_speed[1].out,pid_speed[2].out,0);	//电流发送函数npid_speed[0].result
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int i=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	Sensor_GPIO_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
	MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);//基本定时器开启中断
	HAL_TIM_Base_Start_IT(&htim2);//基本定时器开启中断
	HAL_TIM_Base_Start_IT(&htim3);//基本定时器开启中断
	HAL_TIM_Base_Start_IT(&htim4);//基本定时器开启中断
	//PID初始化
	for(i=1;i<3;i++)
	{
		PID_init(&pid_speed[i], PID_POSITION, PID_speed, 16000, 2000);
		PID_init(&pid_position[i], PID_POSITION, PID_position, 2000, 600);
	}
  	PID_init(&pid_speed[0],FUZZY_PID_POSITION, PID_speed_F, 16000, 2000);/*模糊pid初始化*/
  	//NERVEPIDInitialization(&npid_speed[0],0, 16000);/*单神经元pid初始化*/
	//1572917.73, 2000//
	//获取电机反馈值
	//定义0号电机为主线抱紧电机
	//定义1号电机为刀具升降电机
	//定义2号电机为旋转剥皮电机
  motor_data0=get_chassis_motor_measure_point(0);
	motor_data1=get_chassis_motor_measure_point(1);
	motor_data2=get_chassis_motor_measure_point(2);
	//printf("初始化结束。\r\n");
	//ShowMessage();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		u8 c = ch;
		switch(c)
		{
			case 1:
				if(set_speed_0<2500)
				{
					set_speed_0 = set_speed_0 + m;
				}
				speed_flag_0=1;
				printf("旋转切皮(加速)");
				ch=100;
				break;
			case 2:
				set_speed_0 = 0; 
				speed_flag_0=1;
				set_speed_1 = 0; 
			 	speed_flag_1=1;
				set_speed_2= 0; 
			 	speed_flag_2=1;
        Sensor_flag=0;
				ch=100;
			break;
			case 3:
//		      set_angle_2 =-5*19.203*8191 ;
   		    angle_flag_2=0;
         
          speed_flag_2=1;
				  Sensor_flag=1;		          
				  ch=100;
		   	break;
			case 4:
        Sensor_flag=0;
	      set_speed_2 =600;
				speed_flag_2=1;
				ch=100;
				break;
			case 5:
        angle_flag_1=0;
				set_speed_1= 300; 
				speed_flag_1=1;
				printf("主线抱紧机构夹紧!");
				ch=100;
				break;

			case 6:
        angle_flag_1=0;
				set_speed_1= -1500; 
				speed_flag_1=1;
				printf("主线抱紧机构松开!");
				ch=100;
				break;
			case 7:
			  if(set_speed_0>500)
				{
					set_speed_0 = set_speed_0 - m;
				}
				speed_flag_0=1;
		
				printf("旋转切皮(减速)\n");
				ch=100;
				break;
		
			case 8:
				
				set_speed_0 =0 ; 
				speed_flag_0 = 1;
				set_angle_1=0;
				angle_flag_1=1;
        set_speed_2=0;
				set_angle_2=0;
			 	angle_flag_2=1;
			//在旋转切皮机构归位时，不能让刀具升降机构再次开启（速度环）
//				printf("复位");
//				HAL_Delay(3000);
				ch=100;
				break;
			
			case 9:
				 
				speed_flag_0 = 0;
		  	angle_flag_0=0;
				speed_flag_1 = 0;
			 	angle_flag_1=0;
				speed_flag_2 = 0;
		  	angle_flag_2=0;
				ch=100;
				break;
			
			case 10:
				set_speed_2 =1000 ; 
				speed_flag_2 = 1;
				printf("主线抱紧机构停止!");
				ch=100;
				break;
			
			default:
				break;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
