#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

#define LCD_ROWS 2
#define LCD_COLS 16

#define LCD_CLEAR 0x01

extern const GPIO_TypeDef* LCD_RS_PORT;
extern const uint16_t LCD_RS_PIN;
extern const GPIO_TypeDef* LCD_RW_PORT;
extern const uint16_t LCD_RW_PIN;
extern const GPIO_TypeDef* LCD_E_PORT;
extern const uint16_t LCD_E_PIN;

extern const GPIO_TypeDef* LCD_DATA_PORTS[];
extern const uint16_t LCD_DATA_PINS[];

extern const uint8_t DATA_PINS_NUMBER;

extern const uint8_t ROWS[];

extern const uint8_t NUM_CONTRAST_LEVELS;
extern const uint8_t DEFAULT_CONTRAST;
extern const int BKUP_MAGIC_NUMBER;
extern const int DAC_MIN_CONTRAST;
extern const int DAC_MAX_CONTRAST;

void LCD_WriteNibble(char nibble);
void LCD_WriteCommand(char command);
void LCD_WriteData(char data);
void LCD_Init(void);
void LCD_Int(int number);
void LCD_print(char* str);
void LCD_setCursor(uint8_t row, uint8_t col);
void LCD_setContrastLevel(uint8_t level);
void LCD_setBackupContrastLevel();

#endif /* INC_LCD_H_ */
