################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/SysTick.c \
../source/adc.c \
../source/buttons.c \
../source/can.c \
../source/can_open.c \
../source/drivers.c \
../source/flash_wrp.c \
../source/gpio.c \
../source/leds.c \
../source/main.c \
../source/millis.c \
../source/sensors.c \
../source/uart.c 

C_DEPS += \
./source/SysTick.d \
./source/adc.d \
./source/buttons.d \
./source/can.d \
./source/can_open.d \
./source/drivers.d \
./source/flash_wrp.d \
./source/gpio.d \
./source/leds.d \
./source/main.d \
./source/millis.d \
./source/sensors.d \
./source/uart.d 

OBJS += \
./source/SysTick.o \
./source/adc.o \
./source/buttons.o \
./source/can.o \
./source/can_open.o \
./source/drivers.o \
./source/flash_wrp.o \
./source/gpio.o \
./source/leds.o \
./source/main.o \
./source/millis.o \
./source/sensors.o \
./source/uart.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S06JBD64 -DCPU_LPC55S06JBD64_cm33 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=0 -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\utilities" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\drivers" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\device" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\uart" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\lists" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\CMSIS" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\source" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/SysTick.d ./source/SysTick.o ./source/adc.d ./source/adc.o ./source/buttons.d ./source/buttons.o ./source/can.d ./source/can.o ./source/can_open.d ./source/can_open.o ./source/drivers.d ./source/drivers.o ./source/flash_wrp.d ./source/flash_wrp.o ./source/gpio.d ./source/gpio.o ./source/leds.d ./source/leds.o ./source/main.d ./source/main.o ./source/millis.d ./source/millis.o ./source/sensors.d ./source/sensors.o ./source/uart.d ./source/uart.o

.PHONY: clean-source

