

#include "stm32f407xx.h"
#include<string.h>


// PB14 -> MISO SPI2
// PB15 -> MOSI SPI2
// PB13 -> SC   SPI2
// PB12 -> NSS  SPI2
// ALT fun mode : AF0


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}



void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;		// sclk of 8Mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM  =  SPI_SSM_DI; 				// hardware slave managment

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t  GPIOBtn;

	// configuration of button pin
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GPIOBtn);
}

int main(void)
{

	char user_data[] = "hello world";

	GPIO_ButtonInit();
	while(1)
	{
	// wait until button is pressed
	while ( ! (GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)));
	delay();

	// this function is used to initialize GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize SPI2 peripheral parameters
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	// enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);
	
	// first send length information
	uint8_t dataLen = strlen(user_data);
	SPI_SendData(SPI2, &dataLen,1);
	// to send data
	SPI_SendData(SPI2, (uint8_t*)user_data,strlen(user_data));


	// disable SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
