################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/SysTick.c \
../source/adc.c \
../source/can.c \
../source/can_open.c \
../source/drivers.c \
../source/gpio.c \
../source/main.c \
../source/millis.c \
../source/sensors.c \
../source/uart.c 

C_DEPS += \
./source/SysTick.d \
./source/adc.d \
./source/can.d \
./source/can_open.d \
./source/drivers.d \
./source/gpio.d \
./source/main.d \
./source/millis.d \
./source/sensors.d \
./source/uart.d 

OBJS += \
./source/SysTick.o \
./source/adc.o \
./source/can.o \
./source/can_open.o \
./source/drivers.o \
./source/gpio.o \
./source/main.o \
./source/millis.o \
./source/sensors.o \
./source/uart.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S06JBD64 -DCPU_LPC55S06JBD64_cm33 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -DSDK_DEBUGCONSOLE=0 -DSDK_OS_BAREMETAL -I"/home/asolari/FSAE/code/fsae_ecu/utilities" -I"/home/asolari/FSAE/code/fsae_ecu/drivers" -I"/home/asolari/FSAE/code/fsae_ecu/device" -I"/home/asolari/FSAE/code/fsae_ecu/component/uart" -I"/home/asolari/FSAE/code/fsae_ecu/component/lists" -I"/home/asolari/FSAE/code/fsae_ecu/CMSIS" -I"/home/asolari/FSAE/code/fsae_ecu/source" -I"/home/asolari/FSAE/code/fsae_ecu/board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/SysTick.d ./source/SysTick.o ./source/adc.d ./source/adc.o ./source/can.d ./source/can.o ./source/can_open.d ./source/can_open.o ./source/drivers.d ./source/drivers.o ./source/gpio.d ./source/gpio.o ./source/main.d ./source/main.o ./source/millis.d ./source/millis.o ./source/sensors.d ./source/sensors.o ./source/uart.d ./source/uart.o

.PHONY: clean-source

