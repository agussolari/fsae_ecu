################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include device/subdir.mk
-include component/uart/subdir.mk
-include component/lists/subdir.mk
-include board/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := fsae_ecu
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: fsae_ecu.axf

# Tool invocations
fsae_ecu.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
<<<<<<< HEAD
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="ECU_FSAE.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -T ECU_FSAE_Debug.ld -o "ECU_FSAE.axf" $(OBJS) $(USER_OBJS) $(LIBS)
=======
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="fsae_ecu.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -T fsae_ecu_Debug.ld -o "fsae_ecu.axf" $(OBJS) $(USER_OBJS) $(LIBS)
>>>>>>> driver_object
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) fsae_ecu.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "fsae_ecu.axf"; # arm-none-eabi-objcopy -v -O binary "fsae_ecu.axf" "fsae_ecu.bin" ; # checksum -p LPC55S06 -d "fsae_ecu.bin";
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
