/*
 * 011_i2c_master_rx_testing.c
 *
 *  Created on: Oct 10, 2024
 *      Author: Asus
 */
#include "stm32f407xx.h"
#include <stdio.h>

extern void initialise_monitor_handles();

#define MY_ADDR		0x61
#define SLAVE_ADDR	0x68

uint8_t cmd_code[] = {0x51, 0x52};
uint8_t len = 0;
uint8_t Rxbuffer[500];
I2C_Handle_t I2C1Handle;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB6 -> SCL <- A5
 * PB7 -> SDA <- A4
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

	//SDA Pin (PB7)
	I2C_GPIOHandle.GPIO_PinConfig.GPIO_PinNumber		= GPIO_PIN_NO_7;
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

	initialise_monitor_handles();

	printf("Application is running\n");

	//Init button
	GPIO_ButtonInit();

	//Init I2C pin
	I2C1_GPIOInits();

	//Configure I2C1
	I2C1_Inits();

	//Enable I2C
	I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1){

		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		I2C_MasterSendData(&I2C1Handle, cmd_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		I2C_MasterSendData(&I2C1Handle, (cmd_code + 1), 1, SLAVE_ADDR, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, Rxbuffer, len, SLAVE_ADDR, I2C_DISABLE_SR);

		Rxbuffer[len+1] = '\0';
		printf("Data: %s", Rxbuffer);

	}

	return 0;
}

