/*
 * 003LedButtonExt.c
 *
 *  Created on: 16 Mar 2020
 *      Author: mateusz
 */

#include "stm32f407xx.h"

#define HIGH				1
#define BTN_PRESSED			HIGH

void delay()
{
	for(uint32_t i = 0; i != 500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;
	// Initialize variables to prevent undefined behaviour
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);


	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);


	//IRQ config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_14);
		}
	}


	return 0;
}
