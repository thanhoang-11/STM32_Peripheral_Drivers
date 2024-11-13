################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/016_rtc_lcd.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/016_rtc_lcd.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/016_rtc_lcd.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/Users/ASUS/Documents/Embedded-C/MCU1/My_workspace/stm32f4xx_drivers/drivers/Inc" -I"D:/Users/ASUS/Documents/Embedded-C/MCU1/My_workspace/stm32f4xx_drivers/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/016_rtc_lcd.cyclo ./Src/016_rtc_lcd.d ./Src/016_rtc_lcd.o ./Src/016_rtc_lcd.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

