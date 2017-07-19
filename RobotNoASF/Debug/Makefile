################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../Device_Startup/ \
../IMU-DMP


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../communication.c \
../fc_interface.c \
../IMU-DMP/inv_mpu_CUSTOM.c \
../IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.c \
../light_sens_interface.c \
../motor_driver.c \
../opt_interface.c \
../imu_interface.c \
../prox_sens_interface.c \
../robotdefines.c \
../twimux_interface.c \
../Device_Startup/startup_sam4n.c \
../Device_Startup/system_sam4n.c \
../main.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
communication.o \
fc_interface.o \
IMU-DMP/inv_mpu_CUSTOM.o \
IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.o \
light_sens_interface.o \
motor_driver.o \
opt_interface.o \
imu_interface.o \
prox_sens_interface.o \
robotdefines.o \
twimux_interface.o \
Device_Startup/startup_sam4n.o \
Device_Startup/system_sam4n.o \
main.o

OBJS_AS_ARGS +=  \
communication.o \
fc_interface.o \
IMU-DMP/inv_mpu_CUSTOM.o \
IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.o \
light_sens_interface.o \
motor_driver.o \
opt_interface.o \
imu_interface.o \
prox_sens_interface.o \
robotdefines.o \
twimux_interface.o \
Device_Startup/startup_sam4n.o \
Device_Startup/system_sam4n.o \
main.o

C_DEPS +=  \
communication.d \
fc_interface.d \
IMU-DMP/inv_mpu_CUSTOM.d \
IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.d \
light_sens_interface.d \
motor_driver.d \
opt_interface.d \
imu_interface.d \
prox_sens_interface.d \
robotdefines.d \
twimux_interface.d \
Device_Startup/startup_sam4n.d \
Device_Startup/system_sam4n.d \
main.d

C_DEPS_AS_ARGS +=  \
communication.d \
fc_interface.d \
IMU-DMP/inv_mpu_CUSTOM.d \
IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.d \
light_sens_interface.d \
motor_driver.d \
opt_interface.d \
imu_interface.d \
prox_sens_interface.d \
robotdefines.d \
twimux_interface.d \
Device_Startup/startup_sam4n.d \
Device_Startup/system_sam4n.d \
main.d

OUTPUT_FILE_PATH +=RobotNoASF.elf

OUTPUT_FILE_PATH_AS_ARGS +=RobotNoASF.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+=  \
../Device_Startup/sam4n8c_flash.ld \
../Device_Startup/sam4n8c_sram.ld \
../Device_Startup/sam4n_flash.ld \
../Device_Startup/sam4n_sram.ld


# AVR32/GNU C Compiler





























./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.2.1
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAM4N8C__ -DDEBUG -DMPL_LOG_NDEBUG=1 -DEMPL -DUSE_DMP -DMPU9250  -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\cmsis\4.2.0\CMSIS\Include" -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\SAM4N_DFP\1.0.49\include"  -O0 -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m4 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

IMU-DMP/%.o: ../IMU-DMP/%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.2.1
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAM4N8C__ -DDEBUG -DMPL_LOG_NDEBUG=1 -DEMPL -DUSE_DMP -DMPU9250  -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\cmsis\4.2.0\CMSIS\Include" -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\SAM4N_DFP\1.0.49\include"  -O0 -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m4 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

Device_Startup/%.o: ../Device_Startup/%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.2.1
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAM4N8C__ -DDEBUG -DMPL_LOG_NDEBUG=1 -DEMPL -DUSE_DMP -DMPU9250  -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\cmsis\4.2.0\CMSIS\Include" -I"D:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\SAM4N_DFP\1.0.49\include"  -O0 -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m4 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : 6.2.1
	$(QUOTE)D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -mthumb -Wl,-Map="RobotNoASF.map" -Wl,--start-group -lm  -Wl,--end-group -L"..\\Device_Startup"  -Wl,--gc-sections -mcpu=cortex-m4 -Tsam4n8c_flash.ld  
	@echo Finished building target: $@
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O binary "RobotNoASF.elf" "RobotNoASF.bin"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "RobotNoASF.elf" "RobotNoASF.hex"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary "RobotNoASF.elf" "RobotNoASF.eep" || exit 0
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objdump.exe" -h -S "RobotNoASF.elf" > "RobotNoASF.lss"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "RobotNoASF.elf" "RobotNoASF.srec"
	"D:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-size.exe" "RobotNoASF.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "RobotNoASF.elf" "RobotNoASF.a" "RobotNoASF.hex" "RobotNoASF.bin" "RobotNoASF.lss" "RobotNoASF.eep" "RobotNoASF.map" "RobotNoASF.srec"
	