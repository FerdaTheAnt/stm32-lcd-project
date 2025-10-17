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
