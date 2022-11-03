
#include<string.h>
#include "stm32f407xx.h"


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;

	// initialization each and every member element of these structures to 0
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);



	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GPIOBtn);

	// IRQ configurations
	// priority is optional here
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	while(1);

	return 0;
}


void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
