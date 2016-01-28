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
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../main.c \
../spi_driver.c \
../usart_driver.c \
../Matrix.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
main.o \
spi_driver.o \
usart_driver.o \
Matrix.o

OBJS_AS_ARGS +=  \
main.o \
spi_driver.o \
usart_driver.o \
Matrix.o

C_DEPS +=  \
main.d \
spi_driver.d \
usart_driver.d \
Matrix.d

C_DEPS_AS_ARGS +=  \
main.d \
spi_driver.d \
usart_driver.d \
Matrix.d

OUTPUT_FILE_PATH +=avr_alpha.elf

OUTPUT_FILE_PATH_AS_ARGS +=avr_alpha.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler









./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 0.0.0
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DF_CPU=16000000UL  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -mmcu=atxmega128d3 -c -gdwarf-2 -std=gnu99 -lprintf_flt -lm -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
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
	@echo Invoking: AVR/GNU Linker : 0.0.0
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="avr_alpha.map" -Wl,--start-group -Wl,-lm -Wl,-lprintf_flt  -Wl,--end-group -Wl,--gc-sections -mmcu=atxmega128d3 -Wl,-u,vfprintf  
	@echo Finished building target: $@
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "avr_alpha.elf" "avr_alpha.hex"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "avr_alpha.elf" "avr_alpha.eep" || exit 0
	"C:\WinAVR-20100110\bin\avr-objdump.exe" -h -S "avr_alpha.elf" > "avr_alpha.lss"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "avr_alpha.elf" "avr_alpha.srec"
	"C:\WinAVR-20100110\bin\avr-size.exe" "avr_alpha.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "avr_alpha.elf" "avr_alpha.a" "avr_alpha.hex" "avr_alpha.lss" "avr_alpha.eep" "avr_alpha.map" "avr_alpha.srec" "avr_alpha.usersignatures"
	