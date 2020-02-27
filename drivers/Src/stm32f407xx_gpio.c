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
