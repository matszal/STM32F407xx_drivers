/*
 * stm32f407xx_spi.h
 *
 *  Created on: 5 Apr 2020
 *      Author: mateusz
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

/**
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/**
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;		/* This holds the base address of the SPIx(x=0,1,2) peripheral */
	SPI_Config_t	SPIConfig;
}SPI_Handle_t;


/****************************************************
 *          APIs supported by this driver           *
 ****************************************************/

/**
 * Peripheral clock setup
 */
void SPIx_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * Data read and write
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);

/**
 * IRQ configuration and ISR handling 
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(uint8_t PinNumber);
void SPI_IRQPriorityConfig(SPI_Handle_t *pHandle);

#endif /* INC_STM32F407XX_SPI_H_ */
