/*
 * stm32f407xx_spi.c
 *
 *  Created on: 5 Apr 2020
 *      Author: mateusz
 */

#include "stm32f407xx_spi.h"


/****************************************************
 *          APIs supported by this driver           *
 ****************************************************/

/**
 * Peripheral clock setup
 */
/***********************************************************************
 * @fn              SPIx_PeriClockControl
 * @brief           This function enables or disables peripheral clock
 *                  for the given SPI 
 * 
 * @param[in]       base address of the SPI peripheral
 * @param[in]       ENABLE or DISABLE macros
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }      
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }       
    }
}

/**
 * Init and De-Init
 */
/***********************************************************************
 * @fn              SPI_Init
 * @brief           This function initialise SPI
 * 
 * @param[in]       handle structure for the SPI
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    // 7. Config the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/***********************************************************************
 * @fn              SPI_DeInit
 * @brief           This function de-initialise SPI
 * 
 * @param[in]       handle structure for the SPI
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET(); 
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;   
}

/**
 * Data read and write
 */
/***********************************************************************
 * @fn              SPI_SendData
 * @brief           This is a implementation of SPI send data, this
 *                  function is a blocking function
 * 
 * @param[in]       handle structure for the SPI register definition
 * @param[in]       data pointer 
 * @param[in]       length of the buffer
 * 
 * @return          none
 * 
 * @note            This function is blocking
 * 
 ***********************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len)
{
    while (Len != 0)
    {
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16bit
        {
            pSPIx->DR = *((uint16_t*)pTXBuffer);
            Len -= 2;
            (uint16_t*)pTXBuffer++;
        }
        else //8bit
        {
        	pSPIx->DR = *pTXBuffer;
            Len --;
            pTXBuffer++;
        }
    }   
}

/***********************************************************************
 ******************* Other Peripheral Control APIs *********************
 ***********************************************************************/

/***********************************************************************
 * @fn              SPI_PeripheralControl
 * @brief           TODO
 * 
 * @param[in]       TODO
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
    }
}

/***********************************************************************
 * @fn              SPI_SSIConfig
 * @brief           TODO
 * 
 * @param[in]       TODO
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
    }
}


/***********************************************************************
 * @fn              SPI_SSOEConfig
 * @brief           TODO
 * 
 * @param[in]       TODO
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
    }
}
