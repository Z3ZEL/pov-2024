# Makefile for the led project

BUILD_DIR = build
WORK_DIR = pd6
FREQ = 13000000
MCU_TARGET = atmega328p

all:
	mkdir -p $(BUILD_DIR)
	avr-gcc -mmcu=atmega328p -DF_CPU=$(FREQ) -Os -o $(BUILD_DIR)/firmware.elf $(WORK_DIR)/main.c
	avr-objcopy -O binary $(BUILD_DIR)/firmware.elf $(BUILD_DIR)/firmware.bin

install:
	avrdude -B 0.1 -c usbasp -p $(MCU_TARGET) -V -U flash:w:$(BUILD_DIR)/firmware.bin
	
clean:
	rm -rf $(BUILD_DIR)
