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



#include "main.h"
#include "i2c-lcd.h"

#include "stdio.h"
#include "delay_timer.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


float a,b;
unsigned short int uiTimeLed = 0,uiTimeReadData = 0;
int TIME_READ_DATA = 100;



uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;
float setTemp = 0, SET_OK = 0;


void Set_Pin_Output ()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void Set_Pin_Input ()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void DHT11_Start (void){
		Set_Pin_Output();
		HAL_GPIO_WritePin(DHT11_GPIO_Port , DHT11_Pin ,GPIO_PIN_RESET);
		DELAY_TIM_Ms(&htim3, 20);
		HAL_GPIO_WritePin(DHT11_GPIO_Port , DHT11_Pin ,GPIO_PIN_SET);
		DELAY_TIM_Us(&htim3,40);
	  Set_Pin_Input();
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	DELAY_TIM_Us(&htim3,40);
	if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port , DHT11_Pin)))
	{
		DELAY_TIM_Us(&htim3,80);
		if ((HAL_GPIO_ReadPin (DHT11_GPIO_Port , DHT11_Pin))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port , DHT11_Pin)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port , DHT11_Pin)));   // wait for the pin to go high
		DELAY_TIM_Us(&htim3,40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port , DHT11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port , DHT11_Pin)));  // wait for the pin to go low
	}
	return i;
}

#define DEBOUNCE_DELAY      200
static uint32_t lastDebounceTime_UP   = 0;
static uint32_t lastDebounceTime_DOWN = 0;
static uint32_t lastDebounceTime_BACK = 0;
static uint32_t lastDebounceTime_OK   = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currentTick = HAL_GetTick();
		 static uint8_t relayState = 0 ,relayState_1 = 0 ,relayState_2 = 0, relayState_3 = 0;
			
    if (GPIO_Pin == UP_Pin)
    {
        if ((currentTick - lastDebounceTime_UP) >= DEBOUNCE_DELAY)
        {
            lastDebounceTime_UP = currentTick;
             relayState = !relayState;
            if (relayState)
            {
               setTemp +=1; 
            }
        }
    }
    else if (GPIO_Pin == DOWN_Pin)
    {
        if ((currentTick - lastDebounceTime_DOWN) >= DEBOUNCE_DELAY)
        {
            lastDebounceTime_DOWN = currentTick;
         					 relayState_1 = !relayState_1;
            if (relayState_1)
            {
                setTemp-=1;
            }
        }
    }
    else if (GPIO_Pin == BACK_Pin)
    {
        if ((currentTick - lastDebounceTime_BACK) >= DEBOUNCE_DELAY)
        {
            lastDebounceTime_BACK = currentTick;
					 if (relayState_3)
            {
                HAL_GPIO_WritePin(GPIOA, RELAY_1_Pin|RELAY_2_Pin, GPIO_PIN_SET);
            }
						
						else{
							 HAL_GPIO_WritePin(GPIOA, RELAY_1_Pin|RELAY_2_Pin, GPIO_PIN_RESET);
						}
        }
    }
    else if (GPIO_Pin == OK_Pin)
    {
        if ((currentTick - lastDebounceTime_OK) >= DEBOUNCE_DELAY)
        {
            lastDebounceTime_OK = currentTick;
            relayState_2 = !relayState_2;
            if (relayState_2)
            {
                SET_OK = setTemp;
            }
        }
    }
}


void SysTick_Handler(void){
  HAL_IncTick();
	if(uiTimeLed != 0){
		uiTimeLed--;
	}
	if(uiTimeReadData != 0){
		uiTimeReadData--;
	}
}


char temp [100], hump[100];
int main(void)
{

  HAL_Init();
  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	DELAY_TIM_Init(&htim3);
	HAL_GPIO_WritePin(GPIOA, DHT11_Pin|LED_Pin, GPIO_PIN_SET);
	lcd_init();	
	lcd_clear();
	sprintf(temp, "Nhiet do: %0.1f C", Temperature);
	sprintf(hump, "Do am: %0.1f%%", Humidity);		
	lcd_put_cur(0, 0);
  lcd_send_string(temp);
  lcd_put_cur(1, 0);
  lcd_send_string(hump);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
if(uiTimeLed == 0)
		{
			uiTimeLed = 1000;
		}
		else if(uiTimeLed <= 150)
		{
				HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); 
	  } 
		else { 
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET); 
			} 
		if(uiTimeReadData == 0){ 
				DHT11_Start(); 
				Presence = DHT11_Check_Response();
				Rh_byte1 = DHT11_Read (); 
				Rh_byte2 = DHT11_Read (); 
				Temp_byte1 = DHT11_Read (); 
				Temp_byte2 = DHT11_Read (); 
				SUM = DHT11_Read(); 
				TEMP = Temp_byte1; 
				RH = Rh_byte1; 
				Temperature = (float) TEMP;
				Humidity = (float) RH; 
				
			sprintf(temp, "TEMP:%0.1fC-SET:%f", Temperature, setTemp); 
			sprintf(hump, "HUMP: %0.1f%%", Humidity); 
			lcd_put_cur(0, 0); 
			lcd_send_string(temp); 
			lcd_put_cur(1, 0); 
			lcd_send_string(hump); 
			if(Temperature < SET_OK){
				 HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, GPIO_PIN_RESET);
			}
			else{
				 HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, GPIO_PIN_SET);
			}
			uiTimeReadData =TIME_READ_DATA;
			}
    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT11_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY_1_Pin|RELAY_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DHT11_Pin LED_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UP_Pin DOWN_Pin BACK_Pin OK_Pin */
  GPIO_InitStruct.Pin = UP_Pin|DOWN_Pin|BACK_Pin|OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_3_Pin */
  GPIO_InitStruct.Pin = RELAY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RELAY_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_1_Pin RELAY_2_Pin */
  GPIO_InitStruct.Pin = RELAY_1_Pin|RELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
