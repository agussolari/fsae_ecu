################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/uart/fsl_adapter_usart.c 

C_DEPS += \
./component/uart/fsl_adapter_usart.d 

OBJS += \
./component/uart/fsl_adapter_usart.o 


# Each subdirectory must supply rules for building sources it contributes
component/uart/%.o: ../component/uart/%.c component/uart/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S06JBD64 -DCPU_LPC55S06JBD64_cm33 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\utilities" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\drivers" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\device" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\uart" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\lists" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\CMSIS" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\source" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-component-2f-uart

clean-component-2f-uart:
	-$(RM) ./component/uart/fsl_adapter_usart.d ./component/uart/fsl_adapter_usart.o

.PHONY: clean-component-2f-uart

