#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include "main.h"

#define FALSE_KEY 'F'
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 3

extern const GPIO_TypeDef* R_PORTS[];
extern const uint16_t R_PINS[];

extern const GPIO_TypeDef* C_PORTS[];
extern const uint16_t C_PINS[];

extern const char KEYMAP[];

char keypadFSM_task(uint8_t row);

void read_row(uint8_t row);
char get_key(uint8_t row);

#endif /* INC_KEYPAD_H_ */
