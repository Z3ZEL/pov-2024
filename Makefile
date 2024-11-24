# Makefile for the led project

BUILD_DIR = build
WORK_DIR = src
FREQ = 13000000
MCU_TARGET = atmega328p

SRC_FILES := $(wildcard $(WORK_DIR)/*.c)
OBJ_FILES := $(patsubst $(WORK_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRC_FILES))

$(BUILD_DIR)/%.o: $(WORK_DIR)/%.c
	avr-gcc -mmcu=$(MCU_TARGET) -DF_CPU=$(FREQ) -Os -c -o $@ $<

build: $(OBJ_FILES)
	mkdir -p $(BUILD_DIR)
	avr-gcc -mmcu=$(MCU_TARGET) -DF_CPU=$(FREQ) -Os -o $(BUILD_DIR)/firmware.bin $(OBJ_FILES) 

install: build
	avrdude -B 0.1 -c usbasp -p $(MCU_TARGET) -V -U flash:w:$(BUILD_DIR)/firmware.bin

link:
	sudo rfcomm bind /dev/rfcomm0 $(DEVICE_ADDR)
	sudo chmod 777 /dev/rfcomm0
shell:
	cu -l /dev/rfcomm0


clean:
	rm -rf $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)
