/*
 * stm32f407xx.gpio.c
 *
 *  Created on: 27 Feb 2020
 *      Author: mateusz
 */

#include "stm32f407xx_gpio.h"

/**
 * Peripheral clock setup
 */
/***********************************************************************
 * @fn              GPIO_PeriClockControl
 * @brief           This function enables or disables peripheral clock
 *                  for the given GPIO port
 * 
 * @param[in]       base address of the GPIO peripheral
 * @param[in]       ENABLE or DISABLE macros
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }        
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }        
    }   
}

/**
 * Init and De-Init
 */
/***********************************************************************
 * @fn              GPIO_Init
 * @brief           This function initialise GPIO
 * 
 * @param[in]       handle structure for the GPIOs
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    //1. Configure the mode of the gpio pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
        pGPIOHandle->pGPIOx->MODER |= temp; //setting bit
    }
    else
    {
        // interrupt part, will add soon!
    }

    //2. Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
    pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting bit
    
    //3. Configure the pd pu settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
    pGPIOHandle->pGPIOx->PUPDR |= temp; //seting bit

    //4. configure the op type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
    pGPIOHandle->pGPIOx->OTYPER |= temp; //setting bit
    
    //5. Configure the alt functionality 
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode == GPIO_MODE_ALTFN)
    {
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2)); //clearing bit
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2)); //setting bit
    }

}

/***********************************************************************
 * @fn              GPIO_DeInit
 * @brief           This function de-initialise GPIO port
 * 
 * @param[in]       base address of the GPIO peripheral
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/**
 * Data read and write
 */
/***********************************************************************
 * @fn              GPIO_ReadFromInputPin
 * @brief           This function read from a GPIO input pin
 * 
 * @param[in]       base address of the GPIO peripheral
 * @param[in]       GPIO pin number
 * 
 * @return          digital HIGH or LOW
 * 
 * @note            none
 * 
 ***********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber )
{
    return 1;
}

/***********************************************************************
 * @fn              GPIO_ReadFromInputPort
 * @brief           This function read from a GPIO input port
 * 
 * @param[in]       base address of the GPIO peripheral
 * 
 * @return          digital HIGH or LOW
 * 
 * @note            none
 * 
 ***********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return 1;
}

/***********************************************************************
 * @fn              GPIO_WriteToOutputPin
 * @brief           This function write to a GPIO output pin
 * 
 * @param[in]       base address of the GPIO peripheral
 * @param[in]       GPIO pin number
 * @param[in]       digital HIGH or LOW
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

/***********************************************************************
 * @fn              GPIO_WriteToOutputPort
 * @brief           This function write to a GPIO output port
 * 
 * @param[in]       base address of the GPIO peripheral
 * @param[in]       digital HIGH or LOW
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/***********************************************************************
 * @fn              GPIO_ToggleOutputPin
 * @brief           This function toggles the GPIO pin
 * 
 * @param[in]       base address of the GPIO peripheral
 * @param[in]       GPIO pin number
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/**
 * IRQ configuration and ISR handling 
 */
/***********************************************************************
 * @fn              GPIO_IRQConfig
 * @brief           This function configures an interrupt routine
 * 
 * @param[in]       IRQ number
 * @param[in]       IRQ priority
 * @param[in]       ENABLE or DISABLE macros
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/***********************************************************************
 * @fn              GPIO_IRQHandling
 * @brief           This function handles an interrupt event
 * 
 * @param[in]       GPIO pin number
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
