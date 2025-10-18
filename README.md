# stm32-lcd-project
Mini project for MFF Embedded and Real-time Systems course utilizing an STM32 MCU, a 16x2 LCD, a 4x3 membrane keypad and a temperature and humidity sensor.

For this project an STM32F407G-DISC1 development board was used. Example of pin connections that I used.

## LED Display

* VSS -> GND
* VDD -> 5V
* VEE -> PA5 (DAC_OUT2)
* RS -> PE4 (GPIO_Output)
* RW -> PE5 (GPIO_Output)
* E -> PE6 (GPIO_Output)

* D4 -> PD1 (GPIO_Output)
* D5 -> PD2 (GPIO_Output)
* D6 -> PD3 (GPIO_Output)
* D7 -> PD6 (GPIO_Output)

* A (LED+) -> 5V
* K (LED-) -> GND

## TH Sensor

* VCC -> VDD
* GND -> GND
* SCL -> PB7 (I2C_SCL)
* SDA -> PB8 (I2C_SDA)

## Keyboard

* Row pins -- PB15, PB13, PB11, PE15 (GPIO_Output)
* Column pins -- PE13, PE11, PE9 (GPIO_Input)

Independent watchdog set to reset the board 15 seconds.

STM32CubeIDE was used to generate MCU code, my added parts are in the repository along with the modified main.c

The program operates in 4 modes. Mode 1 is the default one.

## Mode 1

* LCD displays temperature (in degrees C and F) and humidity readings.
* Pushing `#` on the keypad enters Mode 2 -- calculator
* Pushing `*` enters Mode 3 -- marquee
* Pushing the blue push button enter Mode 4 -- settings
* the measurements are taken and displayed every 2 seconds

## Mode 2

* Calculator mode
* the whole keypad is active and a LCD is blank
* User can enter the first number
* If the user pushes `*`, an operand is displayed, multiple pushes of `*` cycle through other operands
* If numbers are pushed on the keypad, then a second number is displayed
* Pushing `#`, the LCD displays the result on the first row and it is possible to change the operand
using `*` again and a new second term.
* Blue push button enters settings
* Waiting for 15 seconds resets the board and enters mode 1

## Mode 3

* Some text is displayed with 100 ms between each letter
* To start again, one can push `*`
* Pushing `#`, calculator mode is activated
* blue push button enters settings

## Mode 4

* Settings mode
* only possible setting is the contrast of the LCD
* Numbers on the keypad are used to choose a contrast level between 0 and 9
* `#` saves the new contrast level
* `*` enters Mode 1 discarding the contrast if it was not saved
