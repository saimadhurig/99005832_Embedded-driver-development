/* MCU specific Header File*/

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include<stdint.h>
#define __vo volatile

/*Base addresses of Memory Locations*/
#define FLASH_BASEADDR  0x08000000U//base_address of flash memory
#define SRAM1_BASEADDR  0x20000000U //base_address of sram1
#define SRAM2_BASEADDR  0x2001C000U//base_address of sram2
#define ROM_BASEADDR    0x1FFF0000U//base_address of ROM or system memory
#define SRAM_BASEADDR   0x20000000U//base_address of SRAM

/*Base addresses of BUS Peripherals*/
#define AHB3_BASEADDR  0xA0000000U//base address of AHB3 bus peripheral
#define AHB2_BASEADDR  0x50000000U//base address of AHB2 bus peripheral
#define AHB1_BASEADDR  0x40020000U//base address of AHB1 bus peripheral
#define APB2_BASEADDR  0x40010000U//base address of APB2 bus peripheral
#define APB1_BASEADDR  0x40000000U//base address of APB1 bus peripheral

/*GPIO Ports in AHB1*/
#define GPIOA_BASEADDR  AHB1_BASEADDR  //0x40020000U Base address of GPIOA
#define GPIOB_BASEADDR (AHB1_BASEADDR +0x00000400)//Base address of GPIOB
#define GPIOC_BASEADDR (GPIOB_BASEADDR+0x00000400)//Base address of GPIOC
#define GPIOD_BASEADDR (GPIOC_BASEADDR+0x00000400)//Base address of GPIOD
#define GPIOE_BASEADDR (GPIOD_BASEADDR+0x00000400)//Base address of GPIOE
#define GPIOF_BASEADDR (GPIOE_BASEADDR+0x00000400)//Base address of GPIOF
#define GPIOG_BASEADDR (GPIOF_BASEADDR+0x00000400)//Base address of GPIOG
#define GPIOH_BASEADDR (GPIOG_BASEADDR+0x00000400)//Base address of GPIOH
#define GPIOI_BASEADDR (GPIOH_BASEADDR+0x00000400)//Base address of GPIOI
#define GPIOJ_BASEADDR (GPIOI_BASEADDR+0x00000400)//Base address of GPIOJ
#define GPIOK_BASEADDR (GPIOJ_BASEADDR+0x00000400)//Base address of GPIOK
#define RCC_BASEADDR    0x40023800U//Base address of RCC

/*Base addresses of peripherals on APB1*/
#define I2C1_BASEADDR  0x40005400U //Base address of I2C1
#define I2C2_BASEADDR  (I2C1+0x00000400) //Base address of I2C2
#define I2C3_BASEADDR  (I2C2+0x00000400) //Base address of I2C3
#define SPI2_BASEADDR  (APB1+0x00003800) //Base address of SPI2
#define SPI3_BASEADDR  (SPI2+0x00000400) //Base address of SPI3
#define USART2_BASEADDR (APB1+0x4400) //Base address of USART2
#define USART3_BASEADDR (USART2+0x400)//Base address of USART3
#define USART4_BASEADDR (USART3+0x400)//Base address of USART4
#define USART5_BASEADDR (USART4+0x400)//Base address of USART5

/*Base addresses of peripherals on APB2*/
#define SPI1_BASEADDR  (APB2+0x3000)//Base address of SPI1
#define USART1_BASEADDR (APB2+0x1000)//Base address of USART1
#define USART6_BASEADDR (APB2+0x1400)//Base address of USART6
#define EXTI_BASEADDR  (APB2+0x3C00)//Base address of EXTI
#define SYSCFG_BASEADDR (APB2+0x3800)//Base address of SYSCFG

/*PERIPHERAL STRUCTURE DEFINITIONS*/
typedef struct{
__vo uint32_t MODER;
__vo uint32_t OTYPER;
__vo uint32_t OSPEEDR;
__vo uint32_t PUPDR;
__vo uint32_t IDR;
__vo uint32_t ODR;
__vo uint32_t BSRR;
__vo uint32_t LCKR;
__vo uint32_t AFR[2];
}GPIO_RegDef_t;

/*RCC REGISTER MAP*/
typedef struct{
uint32_t CR;
uint32_t PLLCFGR;
uint32_t CFGR;
uint32_t CIR;
uint32_t AHB1RSTR;
uint32_t AHB2RSTR;
uint32_t AHB3RSTR;
uint32_t RESERVED0;
uint32_t APB1RSTR;
uint32_t APB2RSTR;
uint32_t RESERVED1[2];
uint32_t AHB1ENR;
uint32_t AHB2ENR;
uint32_t AHB3ENR;
uint32_t RESERVED2;
uint32_t APB1ENR;
uint32_t APB2ENR;
uint32_t RESERVED3[2];
uint32_t AHB1LPENR;
uint32_t AHB2LPENR;
uint32_t AHB3LPENR;
uint32_t RESREVED4;
uint32_t APB1LPENR;
uint32_t APB2LPENR;
uint32_t RESERVED4[2];
uint32_t BDCR;
uint32_t CSR;
uint32_t RESERVED5[2];
uint32_t SSCGR;
uint32_t PLLI2SCFGR;
uint32_t PLLSAICFGR;
uint32_t DCKCFGR;
}RCC_RegDef_t;

#define GPIOA ((GPIO_RegDef_t*) (GPIOA_BASEADDR))
#define GPIOB (GPIO_RegDef_t*) (GPIOB_BASEADDR)
#define GPIOC (GPIO_RegDef_t*) (GPIOC_BASEADDR)
#define GPIOD (GPIO_RegDef_t*) (GPIOD_BASEADDR)
#define GPIOE (GPIO_RegDef_t*) (GPIOE_BASEADDR)
#define GPIOF (GPIO_RegDef_t*) GPIOF_BASEADDR
#define GPIOG (GPIO_RegDef_t*) GPIOG_BASEADDR
#define GPIOH (GPIO_RegDef_t*) GPIOH_BASEADDR
#define GPIOI (GPIO_RegDef_t*) GPIOI_BASEADDR
#define RCC   ((RCC_RegDef_t*)  RCC_BASEADDR)

#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()     (RCC->AHB1ENR |=(1<<8))

#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()     (RCC->AHB1ENR &= ~(1<<8))


/*GPIO RESET*/
#define GPIOA_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<0));   (RCC->AHB1RSTR&=~(1<<0));}while(0)
#define GPIOB_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<1));   (RCC->AHB1RSTR&=~(1<<1));}while(0)
#define GPIOC_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<2));   (RCC->AHB1RSTR&=~(1<<2));}while(0)
#define GPIOD_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<3));   (RCC->AHB1RSTR&=~(1<<3));}while(0)
#define GPIOE_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<4));   (RCC->AHB1RSTR&=~(1<<4));}while(0)
#define GPIOF_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<5));   (RCC->AHB1RSTR&=~(1<<5));}while(0)
#define GPIOG_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<6));   (RCC->AHB1RSTR&=~(1<<6));}while(0)
#define GPIOH_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<7));   (RCC->AHB1RSTR&=~(1<<7));}while(0)
#define GPIOI_REG_RESET()   do{(RCC->AHB1RSTR|=(1<<8));   (RCC->AHB1RSTR&=~(1<<8));}while(0)

#include "stm32f4xx_gpio_driver.h"

#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#endif
