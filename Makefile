# Makefile for the led project

BUILD_DIR = build
WORK_DIR = src
FREQ = 13000000
MCU_TARGET = atmega328p




all:
	mkdir -p $(BUILD_DIR)
	avr-gcc -mmcu=atmega328p -DF_CPU=$(FREQ) -Os -o $(BUILD_DIR)/firmware.elf $(WORK_DIR)/main.c
	avr-objcopy -O binary $(BUILD_DIR)/firmware.elf $(BUILD_DIR)/firmware.bin

install: all
	avrdude -B 0.1 -c usbasp -p $(MCU_TARGET) -V -U flash:w:$(BUILD_DIR)/firmware.bin

link:
	sudo rfcomm bind /dev/rfcomm0 $(DEVICE_ADDR)
	sudo chmod 777 /dev/rfcomm0
shell:
	cu -l /dev/rfcomm0


clean:
	rm -rf $(BUILD_DIR)
