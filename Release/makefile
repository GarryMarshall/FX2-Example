CC = sdcc
CC_OPTS = -mmcs51 --model-small
ASM = sdas8051
ASM_OPTS = -losgp

# Identify the source file containing main() so that we can ensure it is first on the link list
C_MAIN = ../main.c

# Name of the firmware file to be created
FIRMWARE_FILE = firmware

C_SRCS = $(wildcard ../*.c)
ASM_SRCS = $(wildcard ../*.asm)
MAIN_REL := $(patsubst ../%.c,%.rel,$(C_MAIN))
RELOCATABLES := $(MAIN_REL) $(filter-out $(MAIN_REL), $(patsubst ../%,%,$(C_SRCS:.c=.rel) $(ASM_SRCS:.asm=.rel)))
ADDITIONAL_RELOCATABLES =

all: $(FIRMWARE_FILE).hex $(FIRMWARE_FILE).bin

%.rel: ../%.c
	$(CC) -c $(CC_OPTS) -o "$@" "$<"
	@echo -n $(@:%.rel=%.d) $(dir $@) > $(@:%.rel=%.d) 
	@$(CC) -c -MM $(CC_OPTS)  "$<" >> $(@:%.rel=%.d)

%.rel: ../%.asm
	$(ASM) $(ASM_OPTS) -o "$@" "$<"

$(FIRMWARE_FILE).hex: $(RELOCATABLES) $(ADDITIONAL_RELOCATABLES)
	$(CC) $(CC_OPTS)  -o "$@" $(RELOCATABLES) $(ADDITIONAL_RELOCATABLES) $(LIBS)

$(FIRMWARE_FILE).bin: $(FIRMWARE_FILE).hex
	objcopy -Iihex "$<" -Obinary "$@"

clean:
	-rm -rf *.rel *.d *.asm *.lst *.rst *.sym *.lk *.map *.mem *.hex *.bin

.PHONY: all clean
