# D6_firmware - Makefile Linux (GNU make)
#
# Usage :
#   make                      # build Debug   -> build/Debug/D6_firmware.{elf,bin,hex,map}
#   make CONFIG=Release       # build Release
#   make flash                # programme la carte via ST-LINK (STM32_Programmer_CLI)
#   make gdbserver            # lance ST-LINK_gdbserver
#   make check                # vérifications statiques de l'ELF
#   make check-qemu           # test de fumée sous QEMU (qemu-system-arm)
#   make debug                # GDB connecté à "make gdbserver"
#   make info                 # affiche les chemins d'outils détectés
#   make hal                  # télécharge le HAL STM32F1 dans third_party/ (si pas de STM32CubeF1 local)
#
# Outils, chemins et options : voir mk/stm32f1.mk et config.mk.example.

TARGET := D6_firmware

APP_SOURCES := \
	Src/adc.c Src/dma.c Src/eeprom.c Src/gpio.c Src/main.c Src/spi.c \
	Src/stm32f1xx_hal_msp.c Src/stm32f1xx_it.c Src/syscalls.c Src/system_stm32f1xx.c \
	Src/tim.c Src/usart.c \
	Src/ADF4351.cpp Src/Command.cpp Src/D6Board.cpp Src/debouncedButton.cpp \
	Src/Loop.cpp Src/Serial.cpp

APP_INCLUDES := Inc
STARTUP      := startup/startup_stm32f103xb.s
LDSCRIPT     ?= STM32F103C8_FLASH.ld

include mk/stm32f1.mk

# Test de fumée sous QEMU (machine stm32vldiscovery, STM32F100RB) : variante liée pour 8 Ko de RAM
QEMU      ?= qemu-system-arm
QEMU_DIR  := $(BUILD_ROOT)/qemu
QEMU_LD   := $(QEMU_DIR)/qemu.ld
QEMU_LD_SRC := STM32F103C8_FLASH.ld
QEMU_ELF  := $(QEMU_DIR)/$(CONFIG)/$(TARGET).elf

.PHONY: check-qemu
check-qemu: $(QEMU_LD)
	$(MAKE) --no-print-directory LDSCRIPT=$(QEMU_LD) BUILD_DIR=$(QEMU_DIR)/$(CONFIG) $(QEMU_ELF)
	QEMU="$(QEMU)" GDB="$(GDB)" scripts/qemu_smoke.sh $(QEMU_ELF)

$(QEMU_LD): $(QEMU_LD_SRC) | $(QEMU_DIR)
	sed -e 's/^_estack = 0x20005000;/_estack = 0x20002000;/' \
	    -e 's/^\(RAM (xrw) *: ORIGIN = 0x20000000, LENGTH = \)20K/\18K/' $< > $@
	grep -q '^_estack = 0x20002000;' $@ && grep -q 'LENGTH = 8K' $@

$(QEMU_DIR):
	mkdir -p $@
