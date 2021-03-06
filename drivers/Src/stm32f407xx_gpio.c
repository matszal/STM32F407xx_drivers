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
	 uint32_t temp=0; //temp. register

	 // Enable the peripheral clock
	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{
		//this part will code later . ( interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT )
		{
			//1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
		{
			//1 . configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
		{
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		//3 . enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
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
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }   
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
 * @return          digital HIGH or LOW from specific pin number
 * 
 * @note            none
 * 
 ***********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber )
{
    return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
}

/***********************************************************************
 * @fn              GPIO_ReadFromInputPort
 * @brief           This function read from a GPIO input port
 * 
 * @param[in]       base address of the GPIO peripheral
 * 
 * @return          digital HIGH or LOW from entire register
 * 
 * @note            none
 * 
 ***********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t)pGPIOx->IDR;
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
    if (Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);

    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
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
    pGPIOx->ODR = Value;
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
    pGPIOx->ODR ^= (1 << PinNumber);
}

/**
 * IRQ configuration and ISR handling 
 */
/***********************************************************************
 * @fn              GPIO_IRQInterruptConfig
 * @brief           This function configures an interrupt routine
 * 
 * @param[in]       IRQ number
 * @param[in]       ENABLE or DISABLE macros
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= (1 << IRQNumber % 32);
        }
        else if (IRQNumber >= 64  && IRQNumber < 96)
        {
            *NVIC_ISER2 |= (1 << IRQNumber % 64);
        }
        
    }
    else
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << IRQNumber % 32);
        }
        else if (IRQNumber >= 64  && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1 << IRQNumber % 64);
        }
    }
}

/**
 * IRQ configuration and ISR handling 
 */
/***********************************************************************
 * @fn              GPIO_IRQPriorityConfig
 * @brief           This function configures interrupt priority
 * 
 * @param[in]       IRQ number
 * @param[in]       IRQ priority
 * 
 * @return          none
 * 
 * @note            none
 * 
 ***********************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority <<  shift_amount);
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
    //clear the exti pr register corresponding to the pin number
    if(EXTI->PR |= (1 << PinNumber))
    {
        //clear the correspoding bit
        EXTI->PR &= ~(1 << PinNumber);
    }

}
