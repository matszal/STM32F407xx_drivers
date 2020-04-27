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


/**
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER			0
#define SPI_DEVICE_MODE_SLAVE			1

/**
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/**
 * @SPI_SclkSpeed
 */
#define SPI_CLK_SPEED_DEV2				0
#define SPI_CLK_SPEED_DEV4				1
#define SPI_CLK_SPEED_DEV8				2
#define SPI_CLK_SPEED_DEV16				3
#define SPI_CLK_SPEED_DEV32				4
#define SPI_CLK_SPEED_DEV64				5
#define SPI_CLK_SPEED_DEV128			6
#define SPI_CLK_SPEED_DEV256			7

/**
 * @SPI_DFF
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/**
 * @CPOL
 */
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

/**
 * @CPHA
 */
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

/**
 * @SPI_SSM
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0

/**
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG	( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG	( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG	( 1 << SPI_SR_BSY )


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

/**
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F407XX_SPI_H_ */
