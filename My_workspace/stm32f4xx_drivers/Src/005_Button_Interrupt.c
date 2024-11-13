#include <stm32f407xx.h>

void delay(void){
	for(uint32_t i = 0; i < 50000; i++);
}

int main(void){

	GPIO_Handle_t GPIO_Button, GPIO_Led;

	GPIO_Button.pGPIOx 	= GPIOA;
	GPIO_Led.pGPIOx 	= GPIOD;

	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_0;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IT_FT;
	GPIO_Button.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;

	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber			= GPIO_PIN_NO_12;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_FAST;

	GPIO_Init(GPIO_Led);
	GPIO_Init(GPIO_Button);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 10);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void){
	delay();
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	GPIO_IRQHandling(GPIO_PIN_NO_0);
}
