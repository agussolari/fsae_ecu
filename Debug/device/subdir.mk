################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../device/system_LPC55S06.c 

C_DEPS += \
./device/system_LPC55S06.d 

OBJS += \
./device/system_LPC55S06.o 


# Each subdirectory must supply rules for building sources it contributes
device/%.o: ../device/%.c device/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S06JBD64 -DCPU_LPC55S06JBD64_cm33 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -DSDK_DEBUGCONSOLE=0 -DSDK_OS_BAREMETAL -I"/home/asolari/FSAE/code/fsae_ecu/utilities" -I"/home/asolari/FSAE/code/fsae_ecu/drivers" -I"/home/asolari/FSAE/code/fsae_ecu/device" -I"/home/asolari/FSAE/code/fsae_ecu/component/uart" -I"/home/asolari/FSAE/code/fsae_ecu/component/lists" -I"/home/asolari/FSAE/code/fsae_ecu/CMSIS" -I"/home/asolari/FSAE/code/fsae_ecu/source" -I"/home/asolari/FSAE/code/fsae_ecu/board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-device

clean-device:
	-$(RM) ./device/system_LPC55S06.d ./device/system_LPC55S06.o

.PHONY: clean-device

