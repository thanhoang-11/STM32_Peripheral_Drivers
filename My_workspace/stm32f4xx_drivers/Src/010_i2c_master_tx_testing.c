/*
 * 010_i2c_master_tx_testing.c
 *
 *  Created on: Oct 10, 2024
 *      Author: Asus
 */
#include "stm32f407xx.h"
//#include <stdio.h>
#include <string.h>

#define MY_ADDR		0x61
#define SLAVE_ADDR	0x68

uint8_t some_data[] = "Hoang dep trai\n";

I2C_Handle_t I2C1Handle;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB6 -> SCL <- A5
 * PB9 -> SDA <- A4
 *
 */
void I2C1_GPIOInits(void){
	GPIO_Handle_t I2C_GPIOHandle;
	I2C_GPIOHandle.pGPIOx								= GPIOB;
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_ALTFN;
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinAltFunMode	= 4;
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_OD;
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_SPEED_FAST;

	//SCL Pin (PB6)
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_6;
	GPIO_Init(&I2C_GPIOHandle);

	//SDA Pin (PB9)
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_9;
	GPIO_Init(&I2C_GPIOHandle);
}

void I2C1_Inits(void){
	I2C1Handle.pI2Cx						= I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl	= I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed		= I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_DeviceAddress	= MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle	= I2C_FM_DUTY_2;

	I2C_Init(&I2C1Handle);
}


void GPIO_ButtonInit(void){

	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx 								= GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void){

	//Button init
	GPIO_ButtonInit();

	//I2C Pin inits
	I2C1_GPIOInits();

	//I2C1 peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1){

		//Wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR);
	}


	return 0;
}

