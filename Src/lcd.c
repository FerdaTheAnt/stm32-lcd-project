#include "lcd.h"

#include "main.h"
#include <string.h>
#include <math.h>

const GPIO_TypeDef* LCD_RS_PORT = GPIOE;
const uint16_t LCD_RS_PIN = GPIO_PIN_4;
const GPIO_TypeDef* LCD_RW_PORT = GPIOE;
const uint16_t LCD_RW_PIN = GPIO_PIN_5;
const GPIO_TypeDef* LCD_E_PORT = GPIOE;
const uint16_t LCD_E_PIN = GPIO_PIN_6;

const GPIO_TypeDef* LCD_DATA_PORTS[] = { GPIOD, GPIOD, GPIOD, GPIOD };
const uint16_t LCD_DATA_PINS[] = { GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_6 };

const uint8_t DATA_PINS_NUMBER = 4;

const uint8_t ROWS[] = { 0x00, 0x40 };

const uint8_t NUM_CONTRAST_LEVELS = 9;
const uint8_t DEFAULT_CONTRAST = 9;
const int BKUP_MAGIC_NUMBER = 0xABCD;
const int DAC_MIN_CONTRAST = 4095;
const int DAC_MAX_CONTRAST = 0;

extern DAC_HandleTypeDef hdac;
extern RTC_HandleTypeDef hrtc;

extern uint8_t contrast_level;

void LCD_WriteNibble(char nibble)
{
	for(uint8_t i = 0; i < DATA_PINS_NUMBER; i++)
	{
		HAL_GPIO_WritePin(LCD_DATA_PORTS[i], LCD_DATA_PINS[i], ((nibble >> i) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	/*
	 * enable pulse
	 */
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
}

void LCD_WriteCommand(char command)
{
	HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
	LCD_WriteNibble(command >> 4);
	LCD_WriteNibble(command & 0x0F);
}

void LCD_WriteData(char data)
{
	HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);
	LCD_WriteNibble(data >> 4);
	LCD_WriteNibble(data & 0x0F);
}

void LCD_Init(void)
{
	HAL_Delay(40);
	HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, GPIO_PIN_RESET);
	LCD_WriteNibble(0x03);
	HAL_Delay(5);
	LCD_WriteNibble(0x03);
	HAL_Delay(1);
	LCD_WriteNibble(0x03);
	LCD_WriteNibble(0x02);
	LCD_WriteCommand(0x28);
	LCD_WriteCommand(0x08);
	LCD_WriteCommand(0x01);
	LCD_WriteCommand(0x06);
	LCD_WriteCommand(0x0C);
}

void LCD_setCursor(uint8_t row, uint8_t col)
{
	if(row > LCD_ROWS - 1)
	{
		row = LCD_ROWS - 1;
	}
	if(col > LCD_COLS - 1)
	{
		col = LCD_COLS - 1;
	}
	LCD_WriteCommand(0x80 + ROWS[row] + col);
}

void LCD_Int(int number)
{
	char buffer[11];
	sprintf(buffer, "%d", number);
	LCD_print(buffer);
}

void LCD_print(char* str)
{
	uint8_t length = strlen(str);
	for(uint8_t i = 0; i < length; i++)
	{
		LCD_WriteData(str[i]);
	}
}

void LCD_setContrastLevel(uint8_t level)
{
	int contrast_range = abs(DAC_MAX_CONTRAST - DAC_MIN_CONTRAST);
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, abs(DAC_MIN_CONTRAST - level*contrast_range/NUM_CONTRAST_LEVELS));
}

void LCD_setBackupContrastLevel()
{
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2) == BKUP_MAGIC_NUMBER)
	{
		contrast_level = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	}
	else
	{
		contrast_level = DEFAULT_CONTRAST;
	}
	LCD_setContrastLevel(contrast_level);
}


