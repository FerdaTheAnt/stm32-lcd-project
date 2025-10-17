#include "keypad.h"

#include "main.h"

typedef enum {
    KEYPAD_STATE_IDLE,
    KEYPAD_STATE_DEBOUNCE,
    KEYPAD_STATE_PRESSED,
    KEYPAD_STATE_RELEASED
} Keypad_state;

const GPIO_TypeDef* R_PORTS[] = { GPIOB, GPIOB, GPIOB, GPIOE };
const uint16_t R_PINS[] = { GPIO_PIN_15, GPIO_PIN_13, GPIO_PIN_11, GPIO_PIN_15 };

const GPIO_TypeDef* C_PORTS[] = { GPIOE, GPIOE, GPIOE };
const uint16_t C_PINS[] = { GPIO_PIN_13, GPIO_PIN_11, GPIO_PIN_9 };

const char KEYMAP[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#' };

static Keypad_state keypad_state = KEYPAD_STATE_IDLE;
static char current_key = FALSE_KEY;
static int current_row = -1;
static uint32_t last_tick = 0;
const uint32_t debounce_time = 30; // ms

char keypadFSM_task(uint8_t row)
{
	if(row > KEYPAD_ROWS || row < 1)
	{
		return FALSE_KEY;
	}
    char key = get_key(row);
    uint32_t now = HAL_GetTick();

    switch (keypad_state)
    {
    case KEYPAD_STATE_IDLE:
        if (key != FALSE_KEY)
        {
            current_key = key;
            current_row = row;
            last_tick = now;
            keypad_state = KEYPAD_STATE_DEBOUNCE;
        }
        break;

    case KEYPAD_STATE_DEBOUNCE:
        if (key == current_key && (now - last_tick) > debounce_time)
        {
            keypad_state = KEYPAD_STATE_PRESSED;
        }
        else if (row == current_row && key == FALSE_KEY)
        {
            keypad_state = KEYPAD_STATE_IDLE;
        }
        break;

    case KEYPAD_STATE_PRESSED:
        if (row == current_row && key == FALSE_KEY)
        {
            last_tick = now;
            keypad_state = KEYPAD_STATE_RELEASED;
        }
        break;

    case KEYPAD_STATE_RELEASED:
        if (row == current_row && (now - last_tick) > debounce_time)
        {
        	key = current_key;
            current_key = FALSE_KEY;
            current_row = -1;
            keypad_state = KEYPAD_STATE_IDLE;
            return key;
        }
        break;
    }
    return FALSE_KEY;
}

void read_row(uint8_t row)
{
	for(uint8_t r = 0; r < KEYPAD_ROWS; r++)
	{
		HAL_GPIO_WritePin(R_PORTS[r], R_PINS[r], GPIO_PIN_SET);
	}
	if(row <= KEYPAD_ROWS && row > 0)
	{
		HAL_GPIO_WritePin(R_PORTS[row-1], R_PINS[row-1], GPIO_PIN_RESET);
	}
}

char get_key(uint8_t row)
{
	if(row > KEYPAD_ROWS || row < 1)
	{
		return FALSE_KEY;
	}
	for(uint8_t col = 0; col < KEYPAD_COLS; col++)
	{
		if(HAL_GPIO_ReadPin(C_PORTS[col], C_PINS[col]) == GPIO_PIN_RESET)
		{
			return KEYMAP[KEYPAD_COLS*(row-1) + col];
		}
	}
	return FALSE_KEY;
}
