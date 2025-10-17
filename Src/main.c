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
#include <string.h>
#include <stdio.h>

#include "lcd.h"
#include "th_sensor.h"
#include "keypad.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	TEMPERATURE,
	CALCULATOR,
	MARQUEE,
	SETTINGS
} Mode;

typedef enum
{
	START,
	FIRST_NUMBER,
	OPERAND,
	SECOND_NUMBER,
} Calculator_state;

typedef void (*mode_function)(void);
typedef Mode (*mode_key_response)(void);
typedef struct { Mode mode; mode_function action; mode_key_response key_response; } fsmTransition;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIS_LOGIN "filippe2"

const uint8_t NUMBER_OF_STATES = 4;

const uint32_t TH_PERIOD = 2000;
const uint32_t MARQUEE_PERIOD = 100;

const char CALC_OPERAND[] = { FALSE_KEY, '+', '-', '*', '/' };

const int FALSE_CONTRAST = 10;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Mode mode = TEMPERATURE;

Calculator_state calc_state = START;
int operand_idx = 0;
int calc_first_number = 0;
int calc_second_number = -1;

uint8_t marquee_idx = 0;
int show_marquee = 1;

uint8_t settings_request = 0;

uint8_t contrast_level = 9;
uint8_t new_contrast_level = FALSE_CONTRAST;

int button_press = 0;
int contrast_change = 0;
int new_contrast_display = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void handle_temp_and_humidity(void);
void handle_calculator(void);
void handle_marquee(void);
void handle_settings(void);
Mode th_response(void);
Mode calc_response(void);
Mode marquee_response(void);
Mode settings_response(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
fsmTransition fsm[] = {
		{ TEMPERATURE, handle_temp_and_humidity, th_response },
		{ CALCULATOR, handle_calculator, calc_response },
		{ MARQUEE, handle_marquee, marquee_response },
		{ SETTINGS, handle_settings, settings_response },
};

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, ptr, len, 100);
  return len;
}

void handle_temp_and_humidity(void)
{
	static int measurement_done = 0;
	static uint32_t measurement_start;

	if(!measurement_done)
	{
		measurement_done = 1;
		measurement_start = HAL_GetTick();

		int t_degC;
		int t_degF;
		int rh_pRH;
		if(th_sensor_measure(&t_degC, &t_degF, &rh_pRH) == 0)
		{
			LCD_WriteCommand(LCD_CLEAR);
			LCD_setCursor(0, 1);
			LCD_print("Temp: ");
			LCD_Int(t_degC);
			LCD_print("C ");
			LCD_Int(t_degF);
			LCD_print("F");
			LCD_setCursor(1, 1);
			LCD_print("Humidity: ");
			LCD_Int(rh_pRH);
			LCD_print("%");
		}
	}
	else
	{
		if(HAL_GetTick() - measurement_start > TH_PERIOD)
		{
			measurement_done = 0;
		}
	}

}

void handle_calculator(void)
{
	switch(calc_state)
	{
	case START:
		LCD_WriteCommand(LCD_CLEAR);
		break;
	case OPERAND:
		LCD_setCursor(1, 0);
		LCD_WriteData(CALC_OPERAND[operand_idx]);
		break;
	case FIRST_NUMBER:
	case SECOND_NUMBER:
		break;
	}
}

void handle_marquee(void)
{
	static int marquee_step_done = 0;
	static uint32_t marquee_step_start;
	if(!marquee_step_done)
	{
		marquee_step_done = 1;
		marquee_step_start = HAL_GetTick();
		char* sis_login = SIS_LOGIN;
		int length = strlen(sis_login);
		if(show_marquee)
			LCD_WriteData(sis_login[marquee_idx]);
		if(++marquee_idx >= length)
		{
			show_marquee = 0;
			marquee_idx = 0;
		}
	}
	else
	{
		if(HAL_GetTick() - marquee_step_start > MARQUEE_PERIOD)
		{
			marquee_step_done = 0;
		}
	}
}

void handle_settings(void)
{
	if(button_press || new_contrast_display || contrast_change)
	{
		if(new_contrast_display)
		{
			new_contrast_display = 0;
			LCD_setCursor(1, strlen("New Contrast "));
			LCD_Int(new_contrast_level);
		}
		else
		{
			button_press = 0;
			contrast_change = 0;
			LCD_WriteCommand(LCD_CLEAR);
			LCD_setCursor(0, 0);
			LCD_print("Contrast L: ");
			LCD_Int(contrast_level);
			LCD_setCursor(1, 0);
			LCD_print("New Contrast ");
			LCD_WriteData(' ');
		}
	}
}

Mode th_response(void)
{
	read_row(4); // '*', '0', '#'
	switch(keypadFSM_task(4))
	{
		case '#':
			HAL_IWDG_Refresh(&hiwdg);
			LCD_WriteCommand(LCD_CLEAR);
			return CALCULATOR;
		case '*':
			HAL_IWDG_Refresh(&hiwdg);
			LCD_WriteCommand(LCD_CLEAR);
			return MARQUEE;
	}
	return TEMPERATURE;
}

Mode calc_response(void)
{
	char key = FALSE_KEY;
	for(uint8_t row = 1; row < KEYPAD_ROWS+1; row++)
	{
		read_row(row);
		key = keypadFSM_task(row);
		if(key != FALSE_KEY)
		{
			switch(calc_state)
			{
			case START:
				if(key != '*' && key != '#')
				{
					HAL_IWDG_Refresh(&hiwdg);
					calc_first_number = (key - '0');
					LCD_WriteData(key);
				}
				calc_state = FIRST_NUMBER;
				break;
			case FIRST_NUMBER:
				if(key == '*')
				{
					HAL_IWDG_Refresh(&hiwdg);
					operand_idx = 1;
					calc_state = OPERAND;
				}
				else if(key != '#')
				{
					HAL_IWDG_Refresh(&hiwdg);
					calc_first_number = 10*calc_first_number + (key - '0');
					LCD_WriteData(key);
				}
				break;
			case OPERAND:
				if(key == '*')
				{
					HAL_IWDG_Refresh(&hiwdg);
					operand_idx++;
					operand_idx = (operand_idx - 1) % 4 + 1;
				}
				else if(key != '#')
				{
					HAL_IWDG_Refresh(&hiwdg);
					calc_second_number = (key - '0');
					LCD_setCursor(1, 2);
					LCD_WriteData(key);
					calc_state = SECOND_NUMBER;
				}
				break;
			case SECOND_NUMBER:
				if(key == '#')
				{
					HAL_IWDG_Refresh(&hiwdg);
					switch(operand_idx)
					{
					case 1:
						calc_first_number += calc_second_number;
						break;
					case 2:
						calc_first_number -= calc_second_number;
						break;
					case 3:
						calc_first_number *= calc_second_number;
						break;
					case 4:
						calc_first_number /= calc_second_number;
						break;
					}
					LCD_WriteCommand(LCD_CLEAR);
					LCD_Int(calc_first_number);
					calc_state = OPERAND;
				}
				else if(key != '*')
				{
					HAL_IWDG_Refresh(&hiwdg);
					calc_second_number = 10*calc_second_number + (key - '0');
					LCD_WriteData(key);
				}
				break;
			}
		}
	}
	return CALCULATOR;
}

Mode marquee_response(void)
{
	read_row(4);
	switch(keypadFSM_task(4))
	{
		case '*':
			HAL_IWDG_Refresh(&hiwdg);
			LCD_WriteCommand(LCD_CLEAR);
			show_marquee = 1;
			marquee_idx = 0;
			return MARQUEE;
		case '#':
			HAL_IWDG_Refresh(&hiwdg);
			calc_state = START;
			return CALCULATOR;
	}
	return MARQUEE;
}

Mode settings_response(void)
{
	char key = FALSE_KEY;
	for(uint8_t row = 1; row < KEYPAD_ROWS+1; row++)
	{
		read_row(row);
		key = keypadFSM_task(row);
		if(key != FALSE_KEY)
		{
			if(key == '*')
			{
				new_contrast_level = FALSE_CONTRAST;
				return TEMPERATURE;
			}
			else if(key == '#')
			{
				if(new_contrast_level != FALSE_CONTRAST)
				{
					contrast_change = 1;
					contrast_level = new_contrast_level;
					new_contrast_level = FALSE_CONTRAST;
					/*
					 * write the new value to backup registers
					 */
					HAL_PWR_EnableBkUpAccess();
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, contrast_level);
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, BKUP_MAGIC_NUMBER);
					HAL_PWR_DisableBkUpAccess();
					LCD_setContrastLevel(contrast_level);
				}
			}
			else
			{
				HAL_IWDG_Refresh(&hiwdg);
				new_contrast_display = 1;
				new_contrast_level = key - '0';
			}
			break;
		}
	}
	return SETTINGS;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_0:
			HAL_IWDG_Refresh(&hiwdg);
			button_press = 1;
			settings_request = 1;
			break;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_DAC_Start(&hdac, DAC1_CHANNEL_2);
  LCD_Init();
  LCD_setBackupContrastLevel();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for(uint8_t i = 0; i<NUMBER_OF_STATES; i++)
	  {
		  if(mode == fsm[i].mode)
		  {
			  fsm[i].action();
			  mode = fsm[i].key_response();

			  if(settings_request)
			  {
				  HAL_IWDG_Refresh(&hiwdg);
				  settings_request = 0;
				  mode = SETTINGS;
			  }
			  break;
		  }
	  }
	  HAL_Delay(10);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 3750;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|Audio_RST_Pin
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE4 PE5 PE6
                           PE15 */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           PD1 PD2 PD3 Audio_RST_Pin
                           PD6 */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|Audio_RST_Pin
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
