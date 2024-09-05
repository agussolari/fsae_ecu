################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/board.c \
../board/clock_config.c \
../board/pin_mux.c 

C_DEPS += \
./board/board.d \
./board/clock_config.d \
./board/pin_mux.d 

OBJS += \
./board/board.o \
./board/clock_config.o \
./board/pin_mux.o 


# Each subdirectory must supply rules for building sources it contributes
board/%.o: ../board/%.c board/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S06JBD64 -DCPU_LPC55S06JBD64_cm33 -DMCUXPRESSO_SDK -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DNDEBUG -D__REDLIB__ -DSDK_DEBUGCONSOLE=0 -DTIMER_PORT_TYPE_CTIMER=1 -DSDK_OS_BAREMETAL -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/utilities" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/drivers" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/device" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/component/uart" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/component/lists" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/CMSIS" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/component/timer" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/component/timer_manager" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/source" -I"/home/asolari/FSAE/code/FSAE_ECU/ECU_fsl/board" -Os -fno-common -g -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-board

clean-board:
	-$(RM) ./board/board.d ./board/board.o ./board/clock_config.d ./board/clock_config.o ./board/pin_mux.d ./board/pin_mux.o

.PHONY: clean-board

