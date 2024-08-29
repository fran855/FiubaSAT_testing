PROJECT_NAME = fiubasat

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

INCLUDES = \
	-I../include \
	-I../lib/rtos \
	-I../lib/libopencm3/include

SOURCES = \
	main.c \
	blink.c \
	test.c \
	uart.c \
	../lib/rtos/heap_4.c \
	../lib/rtos/list.c \
	../lib/rtos/port.c \
	../lib/rtos/tasks.c \
	../lib/rtos/opencm3.c \
	../lib/rtos/queue.c

CFLAGS = -std=c99 -Wall -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls \
	-Wmissing-prototypes -Wstrict-prototypes -fno-common -ffunction-sections -fdata-sections \
	-Os -g -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -DSTM32F1 $(INCLUDES)

LDFLAGS = -T./stm32f103c8t6.ld -nostartfiles -Wl,--gc-sections

LDLIBS = -L../lib/libopencm3/lib -lopencm3_stm32f1

OBJS = $(SOURCES:.c=.o)

all: $(PROJECT_NAME).elf

$(PROJECT_NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@
	$(OBJCOPY) -O binary $@ $(PROJECT_NAME).bin
	$(SIZE) $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(PROJECT_NAME).elf $(PROJECT_NAME).bin

.PHONY: all clean