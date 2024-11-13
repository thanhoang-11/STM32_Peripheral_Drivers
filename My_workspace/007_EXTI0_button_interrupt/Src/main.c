
#include <stdint.h>
#include <stdio.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define RCC_BASE_ADDR			0x40023800UL
#define RCC_AHB1ENR				(RCC_BASE_ADDR + 0x30)
#define RCC_APB2ENR				(RCC_BASE_ADDR + 0x44)

#define GPIOA_BASE_ADDR			0x40020000UL
#define GPIOA_MODER				(GPIOA_BASE_ADDR + 0x00)

#define EXTI_BASE_ADDR			0x40013C00UL
#define EXTI_IMR				(EXTI_BASE_ADDR + 0x00)

#define EXTI_FTSR				(EXTI_BASE_ADDR + 0x0C)

#define EXTI_PR					(EXTI_BASE_ADDR + 0x14)

#define SYSCFG_BASE_ADDR		0x40013800UL
#define SYSCFG_EXTICR1			(SYSCFG_BASE_ADDR + 0x08)

#define NVIC_ISER0              0xE000E100UL

int main(void)
{
	//1. Enable clock for AHB1
	uint32_t *pRCC_AHB1ENR = (uint32_t*)RCC_AHB1ENR;
	*pRCC_AHB1ENR |= (1 << 0);

	//2. Enable clock for APB2
	uint32_t *pRCC_APB2ENR = (uint32_t*)RCC_APB2ENR;
	*pRCC_APB2ENR |= (1 << 14);


	//2. Config PA0 Pin is input
	uint32_t *pGPIOA_MODER = (uint32_t*)GPIOA_MODER;
	*pGPIOA_MODER &= ~(3 << 0);

	// Linking between a GPIO port  and the relevant EXTI line
	uint32_t *pSYSCFG_EXTICR1 = (uint32_t*)SYSCFG_EXTICR1;
	*pSYSCFG_EXTICR1 &= ~(0xf << 0);

	//3. Config for interrupt mask register
	uint32_t *pEXTI_IMR = (uint32_t*)EXTI_IMR;
	*pEXTI_IMR |= (1 << 0);

	//4. Falling trigger enabled (for Event and Interrupt) for input line
	uint32_t *pEXTI_FTSR = (uint32_t*)EXTI_FTSR;
	*pEXTI_FTSR |= (1 << 0);

	// 8. Enable EXTI0 interrupt in NVIC
	uint32_t *pNVIC_ISER0 = (uint32_t*)NVIC_ISER0;
	*pNVIC_ISER0 |= (1 << 6);    // Enable interrupt for EXTI0 (IRQ6)
    /* Loop forever */
	for(;;);
}

void EXTI0_IRQHandler(void){
	printf("Executing EXTI0_IRQHandler\n");

	//clear pending bit
	uint32_t *pEXTI_PR = (uint32_t*)EXTI_PR;
	*pEXTI_PR |= (1 << 0);
}
