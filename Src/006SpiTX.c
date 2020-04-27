/*
 * 006SpiTX.c
 *
 *  Created on: 26 Apr 2020
 *      Author: mateusz
 */

#include "stm32f407xx.h"


// PB15 -> MOSI
// PB14 -> MISO
// PB13 -> SCLK
// PB12 -> NSS
// ALT FUN -> Mode 5

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

//	// MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//
//	// NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);
}

void SPI_Inits()
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig 	= SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed 	= SPI_CLK_SPEED_DEV2;
	SPI2handle.SPIConfig.SPI_DFF 		= SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL 		= SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA 		= SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM		= SPI_SSM_EN;

	SPI_Init(&SPI2handle);


}

int main()
{
	char user_data[] = "Hello world";
	// Function to initialise the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// SPI init fucntion
	SPI_Inits();

	// Enable SPI peripheral 
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(1);

	return 0;
}


