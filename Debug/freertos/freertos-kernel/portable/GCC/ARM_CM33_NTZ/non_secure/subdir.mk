################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/mpu_wrappers_v2_asm.c \
../freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.c \
../freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c 

C_DEPS += \
./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/mpu_wrappers_v2_asm.d \
./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.d \
./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.d 

OBJS += \
./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/mpu_wrappers_v2_asm.o \
./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.o \
./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.o 


# Each subdirectory must supply rules for building sources it contributes
freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/%.o: ../freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/%.c freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S06JBD64 -DCPU_LPC55S06JBD64_cm33 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=0 -DSDK_OS_FREE_RTOS -DTX_INCLUDE_USER_DEFINE_FILE -DFSL_RTOS_THREADX -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\utilities" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\drivers" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\device" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\uart" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\component\lists" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\CMSIS" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\freertos\freertos-kernel\include" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\freertos\freertos-kernel\portable\GCC\ARM_CM33_NTZ\non_secure" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\source" -I"G:\Mi unidad\ITBA - Agus\FSAE\fsae_ecu\board" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-freertos-2f-freertos-2d-kernel-2f-portable-2f-GCC-2f-ARM_CM33_NTZ-2f-non_secure

clean-freertos-2f-freertos-2d-kernel-2f-portable-2f-GCC-2f-ARM_CM33_NTZ-2f-non_secure:
	-$(RM) ./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/mpu_wrappers_v2_asm.d ./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/mpu_wrappers_v2_asm.o ./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.d ./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.o ./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.d ./freertos/freertos-kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.o

.PHONY: clean-freertos-2f-freertos-2d-kernel-2f-portable-2f-GCC-2f-ARM_CM33_NTZ-2f-non_secure

