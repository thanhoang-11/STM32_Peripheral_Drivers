#include "stm32f407xx.h"
#include <string.h>
/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * Alternate function mode: 5
 */

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx 									= GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode		= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType 			= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl 		= GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed 			= GPIO_SPEED_FAST;

	//SCLK PIN
	SPIPins.GPIO_PinConfig.GPIO_PinNumber			= GPIO_PIN_NO_13;
	GPIO_Init(SPIPins);

	// MISO PIN
	SPIPins.GPIO_PinConfig.GPIO_PinNumber			= GPIO_PIN_NO_14;
	GPIO_Init(SPIPins);

	// MOSI PIN
	SPIPins.GPIO_PinConfig.GPIO_PinNumber			= GPIO_PIN_NO_15;
	GPIO_Init(SPIPins);

	// NSS PIN
	SPIPins.GPIO_PinConfig.GPIO_PinNumber			= GPIO_PIN_NO_12;
	GPIO_Init(SPIPins);
}


void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx						= SPI2;
	SPI2handle.SPIConfig.SPI_DeviceMode		= SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_BusConfig		= SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DFF			= SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_SclkSpeed		= SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_CPOL			= SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPHA			= SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM			= SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}

int main(void){

	char user_data[] = "Hello world";

	//This function used to initialize the GPIO PINs to behave  as SPI2 pins
	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);

	uint8_t data_len = strlen(user_data);
	SPI_SendData(SPI2, &data_len, 1);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//Lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}

