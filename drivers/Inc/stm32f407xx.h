/*
 * stm32f407xx.h
 *
 *  Created on: Feb 22, 2020
 *      Author: mateusz
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __vo        volatile

/**
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR                  0x08000000U         /* flash base address memory */
#define SRAM1_BASEADDR                  0x20000000U         /* SRAM1 base address memory 112KB */
#define SRAM2_BASEADDR                  0x20001C00U         /* SRAM2 base address memory 112KB * 1024 = 1C00h */
#define SRAM                            SRAM1_BASEADDR
#define ROM                             0x1FFF0000U         /* System memory base address */


/**
 * AHBx andAPBx Bus Peripheral base addresses
 */
 #define PERIPH_BASE                    0x40000000U         /* Base peripheral register memory address*/
 #define APB1PERIPH_BASE                PERIPH_BASE         /* APB1 register memory address*/
 #define APB2PERIPH_BASE                0x40010000U         /* APB2 register memory address*/
 #define AHB1PERIPH_BASE                0x40020000U         /* AHB1 register memory address*/
 #define AHB2PERIPH_BASE                0x50000000U         /* AHB2 register memory address*/


/**
 * Base addresses of peripherals hanging on AHB1 bus
 * GPIOx BASE address + offset
 */
#define GPIOA_BASEADDR                  AHB1PERIPH_BASE
#define GPIOB_BASEADDR                  (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR                  (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR                  (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR                  (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR                  (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR                  (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR                  (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR                  (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR                  (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR                  (AHB1PERIPH_BASE + 0x2800)


/**
 * Base addresses of peripherals hanging on APB1 bus
 * Specific peripheral base address + offset
 * We're developing drivers for:
 * I2C 1,2,3, SPI 2,3, USART 2,3, UART 4,5 atm. only
 */
#define I2C1_BASEADDR                   (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR                   (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR                   (APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR                   (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR                   (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR                 (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR                 (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR                  (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR                  (APB1PERIPH_BASE + 0x5000)


/**
 * Base addresses of peripherals hanging on APB2 bus
 * Specific peripheral base address + offset
 * We're developing drivers for:
 * SPI1, USART 1 & 6, EXTI, SYSCFG atm. only
 */
#define SPI1_BASEADDR                   (APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR                 (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR                 (APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR                   (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR                 (APB2PERIPH_BASE + 0x3800)


/*****************peripheral register definition structures********************/

typedef struct
{
    __vo uint32_t MODER;    /*  These bits configure the I/O direction mode
                                00: Input
                                01: General purpose output mode
                                10: Alternate function mode
                                11: Analog mode */

    __vo uint32_t OTYPER;   /*  These bits configure the output type of I/O port 
                                0: output push-pull (reset state)
                                1: output open-drain */

    __vo uint32_t OSPEEDR;  /*  These bits configre the I/O output speed
                                00: low speed
                                01: medium speed
                                10: high speed
                                11: vedry high speed */

    __vo uint32_t PUPDR;    /*  These bits configre the I/O pull-up or pull-down
                                00: no pull up, pull down
                                01: pull up
                                10: pull down
                                11: reserved */

    __vo uint32_t IDR;      /*  These bits are read only and can be accessed in word mode only
                                They contain input value of the corresponding I/O port */
    __vo uint32_t ODR;      /*  These bits can be read and written by software */
    __vo uint32_t BSRR;     /*  Port bit set reset register */
    __vo uint32_t LCKR;     /*  Port configuration lock register*/
    __vo uint32_t AFR[2];   /*  GPIO alternate function low + high register implemented
                                as an array. Check datasheet for all AF */

}GPIO_RegDef_t;


typedef struct
{
    __vo uint32_t   CR;             /* Clock control register: Reserved must be kept at reset value */
    __vo uint32_t   PLLCFGR;        /* This register is used to configure the PLL clock outputs */
    __vo uint32_t   CFGR;           /* RCC clock configuration register */
    __vo uint32_t   CIR;            /* RCC clock interrupt register */
    __vo uint32_t   AHB1RSTR;       /* RCC AHB1 peripheral reset register */
    __vo uint32_t   AHB2RSTR;       /* RCC AHB2 peripheral reset register */
    __vo uint32_t   AHB3RSTR;       /* RCC AHB3 peripheral reset register */
    uint32_t        RESERVED0;      /* this field is reserved */
    __vo uint32_t   APB1RSTR;       /* RCC APB1 peripheral reset register */
    __vo uint32_t   APB2RSTR;       /* RCC APB2 peripheral reset register */
    uint32_t        RESERVED1[2];   /* this field is reserved */
    __vo uint32_t   AHB1ENR;        /* RCC AHB1 peripheral clock enable register */
    __vo uint32_t   AHB2ENR;        /* RCC AHB2 peripheral clock enable register */
    __vo uint32_t   AHB3ENR;        /* RCC AHB3 peripheral clock enable register */
    uint32_t        RESERVED2;      /* this field is reserved */
    __vo uint32_t   APB1ENR;        /* RCC APB1 peripheral clock enable register */
    __vo uint32_t   APB2ENR;        /* RCC APB2 peripheral clock enable register */
    uint32_t        RESERVED3[2];   /* this field is reserved */
    __vo uint32_t   AHB1LPENR;      /* RCC AHB1 peripheral clock enable in low power mode register */
    __vo uint32_t   AHB2LPENR;      /* RCC AHB2 peripheral clock enable in low power mode register */
    __vo uint32_t   AHB3LPENR;      /* RCC AHB3 peripheral clock enable in low power mode register */
    uint32_t        RESERVED4;      /* this field is reserved */
    __vo uint32_t   APB1LPENR;      /* RCC APB1 peripheral clock enable in low power mode register */
    __vo uint32_t   APB2LPENR;      /* RCC APB2 peripheral clock enable in low power mode register */
    uint32_t        RESERVED5[2];   /* this field is reserved */
    __vo uint32_t   BDCR;           /* RCC Backup domain control register */
    __vo uint32_t   CSR;            /* RCC clock control & status register */
    uint32_t        RESERVED6[2];   /* this field is reserved */
    __vo uint32_t   SSCGR;          /* RCC spread spectrum clock generation register */
    __vo uint32_t   PLLI2SCFGR;     /* RCC PLLI2S configuration register */

}RCC_RegDef_t;

/**
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA               ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB               ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC               ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD               ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE               ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF               ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG               ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH               ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI               ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#endif /* INC_STM32F407XX_H_ */
