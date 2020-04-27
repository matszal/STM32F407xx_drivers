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
void SPIx_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
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

    // Enable the periheral clock
    SPIx_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
    
    // Configure SPI_Cr1 register
    uint32_t tempreg = 0;

    // 1. Config the device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. Config the bus config
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // BIDI mode should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // BIDI mode should be set
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // BIDI mode should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        // RX only should be set
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Config the SPI clock speed
    switch (pSPIHandle->SPIConfig.SPI_SclkSpeed)
    {
        // Set specific bitfield to get clock speed rate f/DEVx
    case SPI_CLK_SPEED_DEV2:
        tempreg |= (0 << SPI_CR1_BR);
        break;
    case SPI_CLK_SPEED_DEV4:
        tempreg |= (1 << SPI_CR1_BR);
        break;
    case SPI_CLK_SPEED_DEV8:
        tempreg |= (2 << SPI_CR1_BR);
        break;
    case SPI_CLK_SPEED_DEV16:
        tempreg |= (3 << SPI_CR1_BR);
        break;
    case SPI_CLK_SPEED_DEV32:
        tempreg |= (4 << SPI_CR1_BR);
        break;
    case SPI_CLK_SPEED_DEV64:
        tempreg |= (5 << SPI_CR1_BR);
        break;
    case SPI_CLK_SPEED_DEV128:
        tempreg |= (6 << SPI_CR1_BR);
        break;
    case SPI_CLK_SPEED_DEV256:
        tempreg |= (7 << SPI_CR1_BR);
        break;
    
    default:
        break;
    }

    // 4. Config the data frame format
    if (pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_8BITS)
    {
        // Clear the DFF bit
        tempreg &= ~(1 << SPI_CR1_DFF);
    }
    else if (pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_16BITS)
    {
        // Set the DFF bit
        tempreg |= (1 << SPI_CR1_DFF);
    }
    
    // 5. Config the CPOL
    if (pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_LOW)
    {
        // Clear the CPOL reg
        tempreg &= ~(1 << SPI_CR1_CPOL);
    }
    else if (pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_HIGH)
    {
        // Set the CPOL reg
        tempreg |= (1 << SPI_CR1_CPOL);
    }

    // 6. Config the CPHA
    if (pSPIHandle->SPIConfig.SPI_CPHA == SPI_CPHA_LOW)
    {
        // Clear the CPHA reg
        tempreg &= ~(1 << SPI_CR1_CPHA);
    }
    else if (pSPIHandle->SPIConfig.SPI_CPHA == SPI_CPHA_HIGH)
    {
        // Set the CPHA reg
        tempreg |= (1 << SPI_CR1_CPHA);
    }

    // 7. Config the SSM
    if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_DI)
    {
        // Clear the SSM reg
        tempreg &= ~(1 << SPI_CR1_SSM);
    }
    else if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
    {
        // Set the SSM reg
        tempreg |= (1 << SPI_CR1_SSM);
    }

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

        while (SPI_GetFlagStatus(pSPIx, 1) != FLAG_RESET);

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
