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