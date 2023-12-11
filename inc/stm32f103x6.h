/*
 * stm32f103x6.h
 *
 *  Created on: Sep 21, 2023
 *      Author: salah
 */

#ifndef INC_STM32F103X6_H_
#define INC_STM32F103X6_H_

//.............
// includes
//.............

#include <stdlib.h>
#include "STD_TYPES.h"



//.............
// Base addresses for memories
//.............

#define Flash_Memory_Base						0x08000000UL
#define System_Memory_Base						0x1ffff000UL
#define SRAM_Memory_Base						0x20000000UL



#define Peripherals_Base						0x40000000UL
#define cortex_M3_internal_Peripherals_Base		0xE0000000UL
//NVIC RegisterMap
#define NVIC_BASE			(0xE000E100UL)
#define NVIC_ISER0			*((vuint32 *)(NVIC_BASE+0x000))
#define NVIC_ISER1			*((vuint32 *)(NVIC_BASE+0x004))
#define NVIC_ISER2			*((vuint32 *)(NVIC_BASE+0x008))
#define NVIC_ICER0			*((vuint32 *)(NVIC_BASE+0x080))
#define NVIC_ICER1			*((vuint32 *)(NVIC_BASE+0x084))
#define NVIC_ICER2			*((vuint32 *)(NVIC_BASE+0x088))


//.............
//Base addresses for AHB peripherals
//.............


//RCC
#define RCC_BASE								0x40021000UL




//.............
//Base addresses for APB2 peripherals
//.............


//GPIO
//A,B Fully included in LQFP48 package

#define GPIOA_BASE								0x40010800UL
#define GPIOB_BASE								0x40010C00UL

//C,D Partial included in LQFP48 package
#define GPIOC_BASE								0x40011000UL
#define GPIOD_BASE								0x40011400UL

//E not included in LQFP48 package
#define GPIOE_BASE								0x40011800UL

//EXTI
#define EXTI_BASE								0x40010400UL

//AFIO
#define AFIO_BASE								0x40010000UL

//USART1
#define USART1_BASE								0x40013800UL


//.............
//Base addresses for APB1 peripherals
//.............


//USART2
#define USART2_BASE								0x40004400UL

//USART3
#define USART3_BASE								0x40004800UL


//===========================================================================

//*****************************
//Peripherals registers
//*****************************

//*****************************
//Peripherals registers GPIO
//*****************************

typedef struct {
	vuint32 CRL;
	vuint32 CRH;
	vuint32 IDR;
	vuint32 ODR;
	vuint32 BSRR;
	vuint32 BRR;
	vuint32 LCKR;
}GPIO_TypeDef;



//*****************************
//Peripherals registers RCC
//*****************************

typedef struct {
	vuint32 CR;
	vuint32 CFGR;
	vuint32 CIR;
	vuint32 APB2RSTR;
	vuint32 APB1RSTR;
	vuint32 AHBENR;
	vuint32 APB2ENR;
	vuint32 APB1ENR;
	vuint32 BDCR;
	vuint32 CSR;
	vuint32 AHBSTR;
}RCC_TypeDef;





//*****************************
//Peripherals registers EXTI
//*****************************

typedef struct {
	vuint32 IMR;
	vuint32 EMR;
	vuint32 RTSR;
	vuint32 FTSR;
	vuint32 SWIER;
	vuint32 PR;
}EXTI_TypeDef;






//*****************************
//Peripherals registers AFIO
//*****************************

typedef struct {
	vuint32 EVCR;
	vuint32 MAPR;
	vuint32 EXTICR[4];
	vuint32 Reserved;
	vuint32 MAPR2;
}AFIO_TypeDef;





//*****************************
//Peripherals registers RCC
//*****************************
typedef struct {
	vuint32 SR ;
	vuint32 DR ;
	vuint32 BRR ;
	vuint32 CR1 ;
	vuint32 CR2 ;
	vuint32 CR3 ;
	vuint32 GTPR ;
}USART_TypeDef;
//================================================

//*****************************
//Peripherals instants
//*****************************

#define GPIOA							((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB							((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC							((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD							((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE							((GPIO_TypeDef *)GPIOE_BASE)


#define RCC								((RCC_TypeDef *)RCC_BASE)

#define EXTI							((EXTI_TypeDef *)EXTI_BASE)

#define AFIO							((AFIO_TypeDef *)AFIO_BASE)

#define USART1							((USART_TypeDef *)USART1_BASE)
#define USART2							((USART_TypeDef *)USART2_BASE)
#define USART3							((USART_TypeDef *)USART3_BASE)

//==================================================

//*****************************
//Clock Enable macros
//*****************************

#define RCC_GPIOA_CLK_EN()				(RCC->APB2ENR |= 1<<2)
#define RCC_GPIOB_CLK_EN()				(RCC->APB2ENR |= 1<<3)
#define RCC_GPIOC_CLK_EN()				(RCC->APB2ENR |= 1<<4)
#define RCC_GPIOD_CLK_EN()				(RCC->APB2ENR |= 1<<5)
#define RCC_GPIOE_CLK_EN()				(RCC->APB2ENR |= 1<<6)

//AFIO Clock Enable
#define RCC_AFIO_CLK_EN()				(RCC->APB2ENR |= 1<<0)

//USART Clock Enable
#define RCC_USART1_CLK_EN()				(RCC->APB2ENR |= 1<<14)
#define RCC_USART2_CLK_EN()				(RCC->APB1ENR |= 1<<17)
#define RCC_USART3_CLK_EN()				(RCC->APB1ENR |= 1<<18)
//RCC Reset Mechanism
#define RCC_USART1_RESET()				(RCC->APB2RSTR |= 1<<14)
#define RCC_USART2_RESET()				(RCC->APB1RSTR |= 1<<17)
#define RCC_USART3_RESET()				(RCC->APB1RSTR |= 1<<18)



//========================
//IVT
//========================

//EXTI
#define EXTI0_IRQ		6
#define EXTI1_IRQ		7
#define EXTI2_IRQ		8
#define EXTI3_IRQ		9
#define EXTI4_IRQ		10
#define EXTI5_IRQ		23
#define EXTI6_IRQ		23
#define EXTI7_IRQ		23
#define EXTI8_IRQ		23
#define EXTI9_IRQ		23
#define EXTI10_IRQ		40
#define EXTI11_IRQ		40
#define EXTI12_IRQ		40
#define EXTI13_IRQ		40
#define EXTI14_IRQ		40
#define EXTI15_IRQ		40

//USART
#define USART1_IRQ		37
#define USART2_IRQ		38
#define USART3_IRQ		39

//=========================================
//NVIC IRQ Enable/Disable macros
//=========================================

#define NVIC_IRQ6_EXTI0_Enable			(NVIC_ISER0|=(1<<6))
#define NVIC_IRQ7_EXTI1_Enable			(NVIC_ISER0|=(1<<7))
#define NVIC_IRQ8_EXTI2_Enable			(NVIC_ISER0|=(1<<8))
#define NVIC_IRQ9_EXTI3_Enable			(NVIC_ISER0|=(1<<9))
#define NVIC_IRQ10_EXTI4_Enable			(NVIC_ISER0|=(1<<10))
#define NVIC_IRQ23_EXTI5_9_Enable		(NVIC_ISER0|=(1<<23))
#define NVIC_IRQ40_EXTI10_15_Enable		(NVIC_ISER1|=(1<<8))

#define NVIC_IRQ6_EXTI0_Disable			(NVIC_ICER0|=(1<<6))
#define NVIC_IRQ7_EXTI1_Disable			(NVIC_ICER0|=(1<<7))
#define NVIC_IRQ8_EXTI2_Disable			(NVIC_ICER0|=(1<<8))
#define NVIC_IRQ9_EXTI3_Disable			(NVIC_ICER0|=(1<<9))
#define NVIC_IRQ10_EXTI4_Disable		(NVIC_ICER0|=(1<<10))
#define NVIC_IRQ23_EXTI5_9_Disable		(NVIC_ICER0|=(1<<23))
#define NVIC_IRQ40_EXTI10_15_Disable	(NVIC_ICER1|=(1<<8))

//USART
#define NVIC_IRQ37_USART1_Enable			(NVIC_ISER1 |= 1<< (USART1_IRQ-32) )
#define NVIC_IRQ38_USART2_Enable			(NVIC_ISER1 |= 1<< (USART2_IRQ-32) )
#define NVIC_IRQ39_USART3_Enable			(NVIC_ISER1 |= 1<< (USART3_IRQ-32) )

#define NVIC_IRQ37_USART1_Disable			(NVIC_ICER1 |= 1<< (USART1_IRQ-32) )
#define NVIC_IRQ38_USART2_Disable			(NVIC_ICER1 |= 1<< (USART2_IRQ-32) )
#define NVIC_IRQ39_USART3_Disable			(NVIC_ICER1 |= 1<< (USART3_IRQ-32) )

#endif /* INC_STM32F103X6_H_ */
