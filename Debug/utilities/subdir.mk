################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utilities/fsl_assert.c \
../utilities/fsl_debug_console.c \
../utilities/fsl_str.c \
../utilities/semihost_hardfault.c 

S_UPPER_SRCS += \
../utilities/fsl_memcpy.S 

C_DEPS += \
./utilities/fsl_assert.d \
./utilities/fsl_debug_console.d \
./utilities/fsl_str.d \
./utilities/semihost_hardfault.d 

OBJS += \
./utilities/fsl_assert.o \
./utilities/fsl_debug_console.o \
./utilities/fsl_memcpy.o \
./utilities/fsl_str.o \
./utilities/semihost_hardfault.o 


# Each subdirectory must supply rules for building sources it contributes
utilities/%.o: ../utilities/%.c utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S06JBD64 -DCPU_LPC55S06JBD64_cm33 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\utilities" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\drivers" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\device" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\uart" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\lists" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\CMSIS" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\source" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/%.o: ../utilities/%.S utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -D__REDLIB__ -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\source" -g3 -gdwarf-4 -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-utilities

clean-utilities:
	-$(RM) ./utilities/fsl_assert.d ./utilities/fsl_assert.o ./utilities/fsl_debug_console.d ./utilities/fsl_debug_console.o ./utilities/fsl_memcpy.o ./utilities/fsl_str.d ./utilities/fsl_str.o ./utilities/semihost_hardfault.d ./utilities/semihost_hardfault.o

.PHONY: clean-utilities

