# Makefile for the led project

BUILD_DIR = build
WORK_DIR = pd6
FREQ = 13000000

all:
	mkdir -p $(BUILD_DIR)
	avr-gcc -mmcu=atmega328p -DF_CPU=$(FREQ) -Os -o $(BUILD_DIR)/firmware.elf $(WORK_DIR)/main.c
	avr-objcopy -O binary $(BUILD_DIR)/firmware.elf $(BUILD_DIR)/firmware.bin

install:
	avrdude -p atmega328p -c usbasp -P /dev/ttyACM0 -U flash:w:$(BUILD_DIR)/firmware.bin
	
clean:
	rm -rf $(BUILD_DIR)
