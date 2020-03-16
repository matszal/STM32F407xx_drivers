#include "stm32f407xx.h"


int main(void)
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	//handle the interrupt
	PIO_IRQHandling(0);
}
