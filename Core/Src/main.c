/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED1_TOGGLE					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin)
#define IrDa_DATA_IN()	  	HAL_GPIO_ReadPin(IRDA_GPIO_Port, IRDA_Pin)
#define IRDA_ID 						0
#define IRDA_EXTI_LINE			IRDA_Pin

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t 	frame_data = 0;    /* 一帧数据缓存 */
uint8_t  	frame_cnt = 0;
uint8_t  	frame_flag = 0;    /* 一帧数据接收完成标志 */
uint8_t 	isr_cnt;  				 /* 用于计算进了多少次中断 */
uint8_t 	fac_us=0;
uint16_t 	fac_ms=0;
static __IO uint32_t TimingDelay;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void IRDA_EXTI_IRQHANDLER_FUN(void);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
uint8_t Get_Pulse_Time(void);
uint8_t IrDa_Process(void);
void Delay_us(uint32_t nus);
void TimingDelay_Decrement(void);
void SysTick_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t key_val;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  SysTick_Init();

  NVIC_SetPriority(SysTick_IRQn, 0);

	printf("\r\n Hello \r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if( frame_flag == 1 ) /* 一帧红外数据接收完成 */
		{
			key_val = IrDa_Process();
//			printf("\r\n key_val = %d \r\n", key_val);
//			printf("\r\n button pressed: frame_cnt = %d \r\n", frame_cnt);
//			printf("\r\n IT triggered: isr_cnt = %d \r\n", isr_cnt);

			/* 不同的遥控器面板对应不同的键值，需要实际测量 */
			switch( key_val )
			{
				case 0:
				LED1_TOGGLE;
					printf("\r\n key_val = %d \r\n", key_val);
					printf("\r\n Error \r\n");
					HAL_GPIO_WritePin(IRDA_TRIG_GPIO_Port, 	IRDA_TRIG_Pin, 	GPIO_PIN_SET);
					HAL_GPIO_WritePin(IRDA2_TRIG_GPIO_Port, IRDA2_TRIG_Pin, GPIO_PIN_SET);
					Delay_us(5);
					HAL_GPIO_WritePin(IRDA_TRIG_GPIO_Port, 	IRDA_TRIG_Pin, 	GPIO_PIN_RESET);
					HAL_GPIO_WritePin(IRDA2_TRIG_GPIO_Port, IRDA2_TRIG_Pin, GPIO_PIN_RESET);
				break;

				case 162:
				LED1_TOGGLE;
					printf("\r\n key_val = %d \r\n", key_val);
					printf("\r\n POWER \r\n");
					HAL_GPIO_WritePin(IRDA_TRIG_GPIO_Port, 	IRDA_TRIG_Pin, 	GPIO_PIN_SET);
					Delay_us(5);
					HAL_GPIO_WritePin(IRDA_TRIG_GPIO_Port, 	IRDA_TRIG_Pin, 	GPIO_PIN_RESET);
				break;

//				case 226:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n MENU \r\n");
//				break;
//
//				case 34:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n TEST \r\n");
//				break;
//
//				case 2:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n + \r\n");
//				break;
//
//				case 194:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n RETURN \r\n");
//				break;
//
//				case 224:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n |<< \r\n");
//				break;
//
				case 168:
					LED1_TOGGLE;
					printf("\r\n key_val = %d \r\n", key_val);
					printf("\r\n > \r\n");
					HAL_GPIO_WritePin(IRDA2_TRIG_GPIO_Port, IRDA2_TRIG_Pin, GPIO_PIN_SET);
					Delay_us(5);
					HAL_GPIO_WritePin(IRDA2_TRIG_GPIO_Port, IRDA2_TRIG_Pin, GPIO_PIN_RESET);
				break;

//				case 144:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n >>| \r\n");
//				break;
//
//				case 104:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 0 \r\n");
//				break;
//
//				case 152:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n - \r\n");
//				break;
//
//				case 176:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n C \r\n");
//				break;
//
//				case 48:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 1 \r\n");
//				break;
//
//				case 24:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 2 \r\n");
//				break;
//
//				case 122:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 3 \r\n");
//				break;
//
//				case 16:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 4 \r\n");
//				break;
//
//				case 56:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 5 \r\n");
//				break;
//
//				case 90:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 6 \r\n");
//				break;
//
//					case 66:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 7 \r\n");
//				break;
//
//				case 74:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 8 \r\n");
//				break;
//
//				case 82:
//					LED1_TOGGLE;
//					printf("\r\n key_val = %d \r\n", key_val);
//					printf("\r\n 9 \r\n");
//				break;

				default:
					HAL_GPIO_WritePin(IRDA_TRIG_GPIO_Port, 	IRDA_TRIG_Pin, 	GPIO_PIN_RESET);
					HAL_GPIO_WritePin(IRDA2_TRIG_GPIO_Port, IRDA2_TRIG_Pin, GPIO_PIN_RESET);
				break;
			}
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IRDA_TRIG_Pin|IRDA2_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IRDA_TRIG_Pin IRDA2_TRIG_Pin */
  GPIO_InitStruct.Pin = IRDA_TRIG_Pin|IRDA2_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IRDA_Pin IRDA2_Pin */
  GPIO_InitStruct.Pin = IRDA_Pin|IRDA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void IRDA_EXTI_IRQHANDLER_FUN(void)
{
	uint8_t pulse_time = 0;
  uint8_t leader_code_flag = 0; /* 引导码标志位，当引导码出现时，表示一帧数据开始 */
  uint8_t irda_data = 0;        /* 数据暂存位 */

  if(__HAL_GPIO_EXTI_GET_IT(IRDA_EXTI_LINE) != GPIO_PIN_RESET) /* 确保是否产生了EXTI Line中断 */
	{
    while(1)
    {
      if( IrDa_DATA_IN() == GPIO_PIN_SET )        /* 只测量高电平的时间 */
      {
        pulse_time = Get_Pulse_Time();

        /* >=5ms 不是有用信号 当出现干扰或者连发码时，也会break跳出while(1)循环 */
        if( pulse_time >= 250 )
        {
          break; /* 跳出while(1)循环 */
        }

        if(pulse_time >= 200 && pulse_time < 250)         /* 获得前导位 4ms~4.5ms */
        {
          leader_code_flag = 1;
        }
        else if(pulse_time >= 10 && pulse_time < 50)      /* 0.56ms: 0.2ms~1ms */
        {
          irda_data = 0;
        }
        else if(pulse_time >= 50 && pulse_time < 100)     /* 1.68ms：1ms~2ms */
        {
          irda_data =1 ;
        }
        else if(pulse_time >= 100 && pulse_time <= 200 ) /* 2.1ms：2ms~4ms */
        {/* 连发码，在第二次中断出现 */
          frame_flag = 1;               /* 一帧数据接收完成 */
          frame_cnt++;                  /* 按键次数加1 */
          isr_cnt ++;                   /* 进中断一次加1 */
          break;                        /* 跳出while(1)循环 */
        }

        if( leader_code_flag == 1 )
        {/* 在第一次中断中完成 */
          frame_data <<= 1;
          frame_data += irda_data;
          frame_cnt = 0;
          isr_cnt = 1;
        }
      }
    }
    /* 1. Please note you need to comment the interrupt clear flag
     * (__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);) under function
     *  "void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)"
     *  at "stm32f1xx_hal_gpio.c" after re-configuration of .ioc file
     * 2. Please also note you need to comment "HAL_IncTick();" under
     * function "void SysTick_Handler(void)" at "stm32f1xx_it.c"
     * after re-configuration of .ioc file
     */
    __HAL_GPIO_EXTI_CLEAR_IT(IRDA_EXTI_LINE);
	}
}

void IRDA2_EXTI_IRQHANDLER_FUN(void)
{
	uint8_t pulse_time = 0;
  uint8_t leader_code_flag = 0; /* 引导码标志位，当引导码出现时，表示一帧数据开始 */
  uint8_t irda_data = 0;        /* 数据暂存位 */

  if(__HAL_GPIO_EXTI_GET_IT(IRDA_EXTI_LINE) != GPIO_PIN_RESET) /* 确保是否产生了EXTI Line中断 */
	{
    while(1)
    {
      if( IrDa_DATA_IN() == GPIO_PIN_SET )        /* 只测量高电平的时间 */
      {
        pulse_time = Get_Pulse_Time();

        /* >=5ms 不是有用信号 当出现干扰或者连发码时，也会break跳出while(1)循环 */
        if( pulse_time >= 250 )
        {
          break; /* 跳出while(1)循环 */
        }

        if(pulse_time >= 200 && pulse_time < 250)         /* 获得前导位 4ms~4.5ms */
        {
          leader_code_flag = 1;
        }
        else if(pulse_time >= 10 && pulse_time < 50)      /* 0.56ms: 0.2ms~1ms */
        {
          irda_data = 0;
        }
        else if(pulse_time >= 50 && pulse_time < 100)     /* 1.68ms：1ms~2ms */
        {
          irda_data =1 ;
        }
        else if(pulse_time >= 100 && pulse_time <= 200 ) /* 2.1ms：2ms~4ms */
        {/* 连发码，在第二次中断出现 */
          frame_flag = 1;               /* 一帧数据接收完成 */
          frame_cnt++;                  /* 按键次数加1 */
          isr_cnt ++;                   /* 进中断一次加1 */
          break;                        /* 跳出while(1)循环 */
        }

        if( leader_code_flag == 1 )
        {/* 在第一次中断中完成 */
          frame_data <<= 1;
          frame_data += irda_data;
          frame_cnt = 0;
          isr_cnt = 1;
        }
      }
    }
    /* 1. Please note you need to comment the interrupt clear flag
     * (__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);) under function
     *  "void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)"
     *  at "stm32f1xx_hal_gpio.c" after re-configuration of .ioc file
     * 2. Please also note you need to comment "HAL_IncTick();" under
     * function "void SysTick_Handler(void)" at "stm32f1xx_it.c"
     * after re-configuration of .ioc file
     */
    __HAL_GPIO_EXTI_CLEAR_IT(IRDA_EXTI_LINE);
	}
}


/* Get the time for HIGH level */
uint8_t Get_Pulse_Time(void)
{
  uint8_t time = 0;
  while( IrDa_DATA_IN() )
  {
    time ++;
    Delay_us(2);     	// Delay 20us
    if(time == 250)		// Waiting time over 5ms
      return time;
  }
  return time;
}

/*
 * 帧数据有4个字节，第一个字节是遥控的ID，第二个字节是第一个字节的反码
 * 第三个数据是遥控的真正的键值，第四个字节是第三个字节的反码
 */
uint8_t IrDa_Process(void)
{
  uint8_t first_byte, sec_byte, tir_byte, fou_byte;

  first_byte = frame_data >> 24;
  sec_byte = (frame_data>>16) & 0xff;
  tir_byte = frame_data >> 8;
  fou_byte = frame_data;

  /* 记得清标志位 */
  frame_flag = 0;

  if( (first_byte==(uint8_t)~sec_byte) && (first_byte==IRDA_ID) )
  {
    if( tir_byte == (uint8_t)~fou_byte )
      return tir_byte;
  }

  return 0;   /* 错误返回 */
}

/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0库版本
	if (SysTick_Config(SystemCoreClock / 100000))	// ST3.5.0库版本
	{
		/* Capture error */
		while (1);
	}
		// 关闭滴答定时器
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief   us延时程序,10us为一个单位
  * @param
  *		@arg nTime: Delay_us( 1 ) 则实现的延时为 1 * 10us = 10us
  * @retval  无
  */
void Delay_us(__IO uint32_t nTime)
{
	TimingDelay = nTime;

	// 使能滴答定时器
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

/**
  * @brief  获取节拍程序
  * @param  无
  * @retval 无
  * @attention  在 SysTick 中断函数 SysTick_Handler()调用
  */
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	IRDA_EXTI_IRQHANDLER_FUN();
//	IRDA2_EXTI_IRQHANDLER_FUN();
}


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch. FILE *f)
#endif /* __GNUC__ */

//重定向printf函数
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);//输出指向串口USART1
	return ch;
}

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
