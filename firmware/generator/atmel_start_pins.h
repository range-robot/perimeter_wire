/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define EN_B GPIO(GPIO_PORTA, 0)
#define ERROR_B GPIO(GPIO_PORTA, 1)
#define IN1_B GPIO(GPIO_PORTA, 2)
#define IN2_B GPIO(GPIO_PORTA, 3)
#define V_SENSE GPIO(GPIO_PORTA, 4)
#define R_TEMP GPIO(GPIO_PORTA, 5)
#define IN1_A GPIO(GPIO_PORTA, 6)
#define IN2_A GPIO(GPIO_PORTA, 7)
#define EN_A GPIO(GPIO_PORTA, 8)
#define ERROR_A GPIO(GPIO_PORTA, 9)
#define LED2_B GPIO(GPIO_PORTA, 16)
#define LED2_G GPIO(GPIO_PORTA, 17)
#define LED2_R GPIO(GPIO_PORTA, 18)
#define LED1_B GPIO(GPIO_PORTA, 19)
#define LED1_G GPIO(GPIO_PORTA, 22)
#define LED1_R GPIO(GPIO_PORTA, 23)
#define USB_DM GPIO(GPIO_PORTA, 24)
#define USB_DP GPIO(GPIO_PORTA, 25)

#endif // ATMEL_START_PINS_H_INCLUDED
