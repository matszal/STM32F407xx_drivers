/*
 * stm32f407xx.h
 *
 *  Created on: Feb 22, 2020
 *      Author: mateusz
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __vo        volatile

#include <stdint.h>
#include <string.h>

/*************START:Processor Specific Details***************************/
/**
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0                      ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1                      ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2                      ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3                      ((__vo uint32_t*)0xE000E10C)

/**
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ICER0                      ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                      ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                      ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                      ((__vo uint32_t*)0xE000E18C)


/**
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR               ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED          4

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
#define RCC_BASEADDR                    (AHB1PERIPH_BASE + 0x3800)


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

/**
 * Peripheral definition structure for GPIO register
 */
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

/**
 * Peripheral definition structure for RCC register
 */
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
 * Peripheral definition structure for EXTI register
 */
typedef struct
{
    __vo uint32_t   IMR;            /* Interrupt mask register. Bits 23:31 are reserved
                                        0: Interrupt request from line x is masked 
                                        1: Interrupt request from line x is not masked */
    __vo uint32_t   EMR;            /* Event mask register
                                        0: Interrupt request from line x is masked 
                                        1: Interrupt request from line x is not masked */
    __vo uint32_t   RTSR;           /* Rising trigger selection register
                                        0: Rising trigger disabled (for Event and Interrupt) for input line
                                        1: Rising trigger enabled (for Event and Interrupt) for input line */
    __vo uint32_t   FTSR;           /* Falling trigger selection register 
                                        0: Falling trigger disabled (for Event and Interrupt) for input line
                                        1: Falling trigger enabled (for Event and Interrupt) for input line */
    __vo uint32_t   SWIER;          /* Software interrupt event register
                                        If interrupt are enabled on line x in the EXTI_IMR register, writing '1' to SWIERx bit when it is
                                        set at '0' sets the corresponding pending bit in the EXTI_PR register, thus resulting in an
                                        interrupt request generation.This bit is cleared by clearing the corresponding bit in EXTI_PR
                                        (by writing a 1 to the bit */
    __vo uint32_t   PR;             /* Pending register 
                                        0: No trigger request occurred
                                        1: selected trigger request occurred
                                        This bit is set when the selected edge event arrives on the external interrupt line.
                                        This bit is cleared by programming it to ‘1’ */

}EXTI_RegDef_t;

/**
 * Peripheral definition structure for SYSCFG register
 */
typedef struct
{
    __vo uint32_t   MEMRMP;         /* Memory remap register */
    __vo uint32_t   PMC;            /* Peripheral mode configuration register */
    __vo uint32_t   EXTICR[4];      /* External interrupt configuration register 1-4 */
    __vo uint32_t   CMPCR;          /* Compensation cell controll register */
    
}SYSCFG_RegDef_t;


/**
 * Peripheral definition structure for SPI register
 */
typedef struct
{
    __vo uint32_t   CR1;            /* SPI Control Register 1 - not used in I2S mode */
    __vo uint32_t   CR2;            /* SPI Control Register 2 */
    __vo uint32_t   SR;             /* SPI Status REgister */
    __vo uint32_t   DR;             /* SPI Data Register */
    __vo uint32_t   CRCPR;          /* SPI CRC polynomial register - not used in I2S mode */
    __vo uint32_t   RXCRCR;         /* SPI RX CRC register - not used in I2S mode */
    __vo uint32_t   TXCRCR;         /* SPI TX CRC register - not used in I2S mode */
    __vo uint32_t   I2SCFGR;        /* SPI_I2S configuration register */
    __vo uint32_t   I2SPR;          /* SPI_I2S prescaler register */
    
}SPI_RegDef_t;



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

#define RCC                 ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG              ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

/**
 * Clock enable macros for GPIOx peripherals
 */
 #define GPIOA_PCLK_EN()                (RCC->AHB1ENR |= (1<<0))
 #define GPIOB_PCLK_EN()                (RCC->AHB1ENR |= (1<<1))
 #define GPIOC_PCLK_EN()                (RCC->AHB1ENR |= (1<<2))
 #define GPIOD_PCLK_EN()                (RCC->AHB1ENR |= (1<<3))
 #define GPIOE_PCLK_EN()                (RCC->AHB1ENR |= (1<<4))
 #define GPIOF_PCLK_EN()                (RCC->AHB1ENR |= (1<<5))
 #define GPIOG_PCLK_EN()                (RCC->AHB1ENR |= (1<<6))
 #define GPIOH_PCLK_EN()                (RCC->AHB1ENR |= (1<<7))
 #define GPIOI_PCLK_EN()                (RCC->AHB1ENR |= (1<<8))

 /**
  * Clock enable macros for I2Cx peripherals
  */
 #define I2C1_PCLK_EN()                 (RCC->APB1ENR |= (1<<21))
 #define I2C2_PCLK_EN()                 (RCC->APB1ENR |= (1<<22))
 #define I2C3_PCLK_EN()                 (RCC->APB1ENR |= (1<<23))

 /**
  * Clock enable macros for SPIx peripherals
  */
 #define SPI1_PCLK_EN()                 (RCC->APB2ENR |= (1<<12))
 #define SPI2_PCLK_EN()                 (RCC->APB1ENR |= (1<<14))
 #define SPI3_PCLK_EN()                 (RCC->APB1ENR |= (1<<15))

 /**
  * Clock enable macros for USARTx + UARTx peripherals
  */
 #define USART1_PCLK_EN()               (RCC->APB2ENR |= (1<<4))
 #define USART2_PCLK_EN()               (RCC->APB1ENR |= (1<<17))
 #define USART3_PCLK_EN()               (RCC->APB1ENR |= (1<<18))
 #define UART4_PCLK_EN()                (RCC->APB2ENR |= (1<<19))
 #define UART5_PCLK_EN()                (RCC->APB2ENR |= (1<<20))

/**
 * Clock enable macros for SYSCFG
 */
#define SYSFCG_PCLK_EN()                (RCC->APB2ENR |= (1<<14))

/**
 * Clock disable macros for GPIOx peripherals
 */
 #define GPIOA_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<0))
 #define GPIOB_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<1))
 #define GPIOC_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<2))
 #define GPIOD_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<3))
 #define GPIOE_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<4))
 #define GPIOF_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<5))
 #define GPIOG_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<6))
 #define GPIOH_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<7))
 #define GPIOI_PCLK_DI()                (RCC->AHB1ENR &= ~(1<<8))

 /**
  * Clock disable macros for I2Cx peripherals
  */
 #define I2C1_PCLK_DI()                 (RCC->APB1ENR &= ~(1<<21))
 #define I2C2_PCLK_DI()                 (RCC->APB1ENR &= ~(1<<22))
 #define I2C3_PCLK_DI()                 (RCC->APB1ENR &= ~(1<<23))

 /**
  * Clock disable macros for SPIx peripherals
  */
 #define SPI1_PCLK_DI()                 (RCC->APB2ENR &= ~(1<<12))
 #define SPI2_PCLK_DI()                 (RCC->APB1ENR &= ~(1<<14))
 #define SPI3_PCLK_DI()                 (RCC->APB1ENR &= ~(1<<15))

 /**
  * Clock disable macros for USARTx + UARTx peripherals
  */
 #define USART1_PCLK_DI()               (RCC->APB2ENR &= ~(1<<4))
 #define USART2_PCLK_DI()               (RCC->APB1ENR &= ~(1<<17))
 #define USART3_PCLK_DI()               (RCC->APB1ENR &= ~(1<<18))
 #define UART4_PCLK_DI()                (RCC->APB2ENR &= ~(1<<19))
 #define UART5_PCLK_DI()                (RCC->APB2ENR &= ~(1<<20))

/**
 * Clock disable macros for SYSCFG
 */
#define SYSFCG_PCLK_DI()                (RCC->APB2ENR &= ~(1<<14))

/**
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR |= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR |= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR |= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR |= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR |= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR |= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR |= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR |= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR |= ~(1 << 8)); }while(0)

/**
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)       ((x == GPIOA) ? 0 : \
                                        (x == GPIOB) ? 1 : \
                                        (x == GPIOC) ? 2 : \
                                        (x == GPIOD) ? 3 : \
                                        (x == GPIOE) ? 4 : \
                                        (x == GPIOF) ? 5 : \
                                        (x == GPIOG) ? 6 : \
                                        (x == GPIOH) ? 7 : 0 )

/**
 * Macros to reset SPIx peripeherals
 */
#define SPI1_REG_RESET()                do{(RCC->APB2RSTR |= (1 << 0)); (RCC->APB2RSTR |= ~(1 << 0)); }while(0)
#define SPI2_REG_RESET()                do{(RCC->APB1RSTR |= (1 << 0)); (RCC->APB1RSTR |= ~(1 << 0)); }while(0)
#define SPI3_REG_RESET()                do{(RCC->APB1RSTR |= (1 << 0)); (RCC->APB1RSTR |= ~(1 << 0)); }while(0)


/**
 * Interrupt vector table
 */
#define IRQ_NO_EXTI0                    6
#define IRQ_NO_EXTI1                    7
#define IRQ_NO_EXTI2                    8
#define IRQ_NO_EXTI3                    9
#define IRQ_NO_EXTI4                    10
#define IRQ_NO_EXTI9_5                  23
#define IRQ_NO_EXTI15_10                40

/**
 * Interrupt priority possible levels
 */
#define NVIC_IRQ_PRIO0                  0
#define NVIC_IRQ_PRIO1                  1
#define NVIC_IRQ_PRIO2                  2
#define NVIC_IRQ_PRIO3                  3
#define NVIC_IRQ_PRIO4                  4
#define NVIC_IRQ_PRIO5                  5
#define NVIC_IRQ_PRIO6                  6
#define NVIC_IRQ_PRIO7                  7
#define NVIC_IRQ_PRIO8                  8
#define NVIC_IRQ_PRIO9                  9
#define NVIC_IRQ_PRIO10                 10
#define NVIC_IRQ_PRIO11                 11
#define NVIC_IRQ_PRIO12                 12
#define NVIC_IRQ_PRIO13                 13
#define NVIC_IRQ_PRIO14                 14
#define NVIC_IRQ_PRIO15                 15

// generic macros
#define ENABLE                          1
#define DISABLE                         0
#define SET                             ENABLE
#define RESET                           DISABLE
#define GPIO_PIN_SET                    SET
#define GPIO_PIN_RESET                  RESET


/**
 * Bit position definitions of SPI_CR1 peripheral
 */
#define SPI_CR1_CPHA                    0
#define SPI_CR1_CPOL                    1
#define SPI_CR1_MSTR                    2
#define SPI_CR1_BR                      3
#define SPI_CR1_SPE                     6
#define SPI_CR1_LSBFIRST                7
#define SPI_CR1_SSI                     8
#define SPI_CR1_SSM                     9
#define SPI_CR1_RXONLY                  10
#define SPI_CR1_DFF                     11
#define SPI_CR1_CRCNEXT                 12
#define SPI_CR1_CRCEN                   13
#define SPI_CR1_BIDIOE                  14
#define SPI_CR1_BIDIMODE                15

/**
 * Bit position definitions of SPI_CR2 peripheral
 */
#define SPI_CR2_RXDMAEN                 0
#define SPI_CR2_TXDMAEN                 1
#define SPI_CR2_SSOE                    2
#define SPI_CR2_FRF                     4
#define SPI_CR2_ERRIE                   5
#define SPI_CR2_RXNEIE                  6
#define SPI_CR2_TXEIE                   7

/**
 * Bit position definitions of SPI_SR peripheral
 */
#define SPI_SR_RXNE                     0
#define SPI_SR_TXE                      1
#define SPI_SR_CHSIDE                   2
#define SPI_SR_UDR                      3
#define SPI_SR_CRCERR                   4
#define SPI_SR_MODF                     5
#define SPI_SR_OVR                      6
#define SPI_SR_BSY                      7
#define SPI_SR_FRE                      8


#include "stm32f407xx_gpio.h"

#endif /* INC_STM32F407XX_H_ */
