# Règles communes de build STM32F103 (GNU make, Linux) avec les outils ST en ligne de commande :
#   - STM32CubeCLT (toolchain GNU-tools-for-STM32, STM32CubeProgrammer, ST-LINK gdbserver)
#   - le package firmware STM32CubeF1 (HAL + CMSIS)
#
# Inclus par le Makefile du firmware et par celui de tests/D6_blinkTest.
# Le Makefile appelant définit avant l'inclusion :
#   TARGET, APP_SOURCES, APP_INCLUDES, STARTUP, LDSCRIPT, DEFINES (optionnel)
#
# Tous les chemins sont des variables surchargeables, par ordre de priorité :
#   ligne de commande  >  config.mk (racine du dépôt, non versionné)  >  environnement  >  détection automatique
# Voir config.mk.example.

D6_ROOT := $(patsubst %/,%,$(dir $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))))

-include $(D6_ROOT)/config.mk

CONFIG ?= Debug

# ---------------------------------------------------------------------------
# Emplacement des outils
# ---------------------------------------------------------------------------

# Racine d'installation de STM32CubeCLT (par défaut : la plus récente trouvée sous /opt/st)
STM32CUBECLT_PATH ?= $(lastword $(sort $(wildcard /opt/st/stm32cubeclt_*)))

# Répertoire bin de la toolchain arm-none-eabi.
# Vide => les outils sont cherchés dans le PATH.
ifeq ($(origin GCC_PATH),undefined)
  GCC_PATH := $(if $(wildcard $(STM32CUBECLT_PATH)/GNU-tools-for-STM32/bin/arm-none-eabi-gcc),$(STM32CUBECLT_PATH)/GNU-tools-for-STM32/bin)
endif
PREFIX ?= arm-none-eabi-
TOOL    = $(if $(GCC_PATH),$(GCC_PATH)/)$(PREFIX)$(1)

CC      := $(call TOOL,gcc)
CXX     := $(call TOOL,g++)
OBJCOPY := $(call TOOL,objcopy)
SIZE    := $(call TOOL,size)
GDB     ?= $(call TOOL,gdb)
NM      := $(call TOOL,nm)

# Programmation / debug
# Sans STM32CubeCLT détecté : répertoire de STM32_Programmer_CLI trouvé dans le PATH
STM32_PROGRAMMER_PATH ?= $(if $(STM32CUBECLT_PATH),$(STM32CUBECLT_PATH)/STM32CubeProgrammer/bin,$(patsubst %/,%,$(dir $(shell command -v STM32_Programmer_CLI 2>/dev/null))))
STM32_PROGRAMMER_CLI  ?= $(if $(wildcard $(STM32_PROGRAMMER_PATH)/STM32_Programmer_CLI),$(STM32_PROGRAMMER_PATH)/STM32_Programmer_CLI,STM32_Programmer_CLI)
STLINK_GDBSERVER      ?= $(if $(wildcard $(STM32CUBECLT_PATH)/STLink-gdb-server/bin/ST-LINK_gdbserver),$(STM32CUBECLT_PATH)/STLink-gdb-server/bin/ST-LINK_gdbserver,ST-LINK_gdbserver)
STLINK_PORT           ?= SWD
GDB_PORT              ?= 61234
GDB_HOST              ?= localhost
# Commande GDB de reset de la cible après "load" (ST-LINK_gdbserver : monitor reset)
GDB_RESET             ?= monitor reset

# ---------------------------------------------------------------------------
# Package firmware STM32CubeF1 (HAL + CMSIS)
# ---------------------------------------------------------------------------
# Par ordre de recherche :
#   1. STM32CUBE_FW_F1 fourni explicitement
#   2. le dépôt STM32Cube (STM32CubeMX / STM32CubeIDE) : ~/STM32Cube/Repository/STM32Cube_FW_F1_V*
#   3. third_party/STM32CubeF1 (rempli par "make hal")
# Les sous-répertoires peuvent aussi être surchargés individuellement :
#   HAL_DRIVER_DIR, CMSIS_DEVICE_DIR, CMSIS_CORE_DIR
LOCAL_FW_F1          := $(D6_ROOT)/third_party/STM32CubeF1
STM32CUBE_REPOSITORY ?= $(HOME)/STM32Cube/Repository
STM32CUBE_FW_F1      ?= $(or $(lastword $(sort $(wildcard $(STM32CUBE_REPOSITORY)/STM32Cube_FW_F1_V*))),$(LOCAL_FW_F1))

HAL_DRIVER_DIR     ?= $(STM32CUBE_FW_F1)/Drivers/STM32F1xx_HAL_Driver
CMSIS_DEVICE_DIR   ?= $(STM32CUBE_FW_F1)/Drivers/CMSIS/Device/ST/STM32F1xx
CMSIS_CORE_DIR     ?= $(STM32CUBE_FW_F1)/Drivers/CMSIS

# Versions téléchargées par "make hal" (dépôts GitHub STMicroelectronics)
HAL_DRIVER_VERSION   ?= v1.1.10
CMSIS_DEVICE_VERSION ?= v4.3.5
CMSIS_CORE_VERSION   ?= v5.9.0_20250520

# Tous les modules HAL/LL : ceux non activés dans stm32f1xx_hal_conf.h
# compilent à vide, et --gc-sections retire le code non utilisé.
HAL_SOURCES := $(filter-out %_template.c,$(wildcard $(HAL_DRIVER_DIR)/Src/*.c))

INCLUDES := $(APP_INCLUDES) \
	$(HAL_DRIVER_DIR)/Inc \
	$(HAL_DRIVER_DIR)/Inc/Legacy \
	$(CMSIS_DEVICE_DIR)/Include \
	$(CMSIS_CORE_DIR)/Include

DEFINES += USE_HAL_DRIVER STM32F103xB STM32F103C8 ARM_MATH_CM3 USE_HAL_LEGACY flash_layout

# ---------------------------------------------------------------------------
# Configuration Debug / Release
# ---------------------------------------------------------------------------
# Debug en -Og : en -O0 le firmware dépasse 32 Ko et déborderait sur la zone EEPROM
ifeq ($(CONFIG),Debug)
  OPT     ?= -Og
  DEFINES += DEBUG=1
else ifeq ($(CONFIG),Release)
  OPT     ?= -O3
  DEFINES += NDEBUG=1 RELEASE=1
else
  $(error CONFIG doit valoir Debug ou Release (reçu : $(CONFIG)))
endif

BUILD_ROOT ?= build
BUILD_DIR  ?= $(BUILD_ROOT)/$(CONFIG)

# ---------------------------------------------------------------------------
# Options de compilation
# ---------------------------------------------------------------------------
MCU      := -mcpu=cortex-m3 -mthumb
COMMON   := $(MCU) $(OPT) -ggdb -ffunction-sections -fdata-sections -Wall \
            $(addprefix -D,$(DEFINES)) $(addprefix -I,$(INCLUDES))
# CFLAGS / CXXFLAGS / ASFLAGS / LDFLAGS fournis par l'utilisateur sont ajoutés à la fin
ALL_CFLAGS   = $(COMMON) -std=gnu11 $(CFLAGS)
ALL_CXXFLAGS = $(COMMON) -std=gnu++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics $(CXXFLAGS)
ALL_ASFLAGS  = $(MCU) -x assembler-with-cpp $(addprefix -D,$(DEFINES)) $(ASFLAGS)
ALL_LDFLAGS  = $(MCU) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs \
               -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--print-memory-usage \
               $(LD_EXTRA) $(LDFLAGS)

# binutils >= 2.39 : supprime l'avertissement "LOAD segment with RWX permissions"
# (le script de lien CubeMX déclare la RAM en xrw)
ifeq ($(shell $(CC) $(MCU) -Wl,--no-warn-rwx-segments -nostartfiles -x c -o /dev/null - </dev/null 2>&1 | grep -c no-warn-rwx),0)
  LD_EXTRA := -Wl,--no-warn-rwx-segments
endif

# ---------------------------------------------------------------------------
# Règles
# ---------------------------------------------------------------------------
MK_FILES    := $(MAKEFILE_LIST)
ALL_SOURCES := $(APP_SOURCES) $(HAL_SOURCES) $(STARTUP)
OBJECTS     := $(addprefix $(BUILD_DIR)/obj/,$(addsuffix .o,$(basename $(notdir $(ALL_SOURCES)))))
# Les objets sont nommés d'après le seul nom de fichier : refuser deux sources de même nom
DUPLICATES  := $(strip $(foreach n,$(sort $(notdir $(basename $(ALL_SOURCES)))),$(if $(word 2,$(filter %/$(n) $(n),$(basename $(ALL_SOURCES)))),$(n))))
ifneq ($(DUPLICATES),)
  $(error Sources de même nom (objets en conflit) : $(DUPLICATES))
endif
vpath %.c   $(sort $(dir $(APP_SOURCES) $(HAL_SOURCES)))
vpath %.cpp $(sort $(dir $(APP_SOURCES)))
vpath %.s   $(dir $(STARTUP))

ELF := $(BUILD_DIR)/$(TARGET).elf
BIN := $(BUILD_DIR)/$(TARGET).bin
HEX := $(BUILD_DIR)/$(TARGET).hex

.PHONY: all clean distclean flash erase gdbserver debug info hal check-hal check

all: $(ELF) $(BIN) $(HEX)
	@$(SIZE) $(ELF)

$(ELF): $(OBJECTS) $(LDSCRIPT)
	$(CXX) $(OBJECTS) $(ALL_LDFLAGS) -o $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/obj/%.o: %.c $(MK_FILES) | $(BUILD_DIR)/obj check-hal
	$(CC) $(ALL_CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/obj/%.o: %.cpp $(MK_FILES) | $(BUILD_DIR)/obj check-hal
	$(CXX) $(ALL_CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/obj/%.o: %.s $(MK_FILES) | $(BUILD_DIR)/obj
	$(CC) $(ALL_ASFLAGS) -c $< -o $@

$(BUILD_DIR)/obj:
	mkdir -p $@

check-hal:
	@test -f "$(HAL_DRIVER_DIR)/Inc/stm32f1xx_hal.h" || { \
	  echo "*** HAL STM32F1 introuvable dans '$(HAL_DRIVER_DIR)'."; \
	  echo "*** Définir STM32CUBE_FW_F1 (ou HAL_DRIVER_DIR/CMSIS_*_DIR), ou lancer 'make hal'."; \
	  exit 1; }
	@test -f "$(CMSIS_DEVICE_DIR)/Include/stm32f1xx.h" || { \
	  echo "*** CMSIS device STM32F1 introuvable dans '$(CMSIS_DEVICE_DIR)'."; exit 1; }
	@test -f "$(CMSIS_CORE_DIR)/Include/core_cm3.h" || { \
	  echo "*** CMSIS core introuvable dans '$(CMSIS_CORE_DIR)'."; exit 1; }

-include $(OBJECTS:.o=.d)

# Vérifications statiques de l'ELF (vecteurs, pile, taille sous la zone EEPROM)
FLASH_LIMIT ?= 0x08008000
check: $(ELF)
	NM="$(NM)" OBJCOPY="$(OBJCOPY)" $(D6_ROOT)/scripts/check_elf.sh $(ELF) $(FLASH_LIMIT)

# Téléchargement du HAL/CMSIS dans third_party/STM32CubeF1 (même arborescence qu'un package STM32CubeF1)
hal:
	HAL_DRIVER_VERSION=$(HAL_DRIVER_VERSION) CMSIS_DEVICE_VERSION=$(CMSIS_DEVICE_VERSION) \
	CMSIS_CORE_VERSION=$(CMSIS_CORE_VERSION) $(D6_ROOT)/scripts/fetch_hal.sh $(LOCAL_FW_F1)

flash: $(HEX)
	"$(STM32_PROGRAMMER_CLI)" -c port=$(STLINK_PORT) -w $(HEX) -v -rst

erase:
	"$(STM32_PROGRAMMER_CLI)" -c port=$(STLINK_PORT) -e all

# Serveur GDB ST-LINK (SWD, persistant : reste actif quand GDB se déconnecte)
gdbserver:
	@test -n "$(STM32_PROGRAMMER_PATH)" || { \
	  echo "*** STM32CubeProgrammer introuvable : définir STM32CUBECLT_PATH ou STM32_PROGRAMMER_PATH."; exit 1; }
	"$(STLINK_GDBSERVER)" -p $(GDB_PORT) -d -e -cp "$(STM32_PROGRAMMER_PATH)"

# Session GDB : se connecte au serveur ("make gdbserver" dans un autre terminal),
# programme l'ELF, fait un reset et s'arrête sur main()
debug: $(ELF)
	"$(GDB)" -q $(ELF) \
	  -ex "target extended-remote $(GDB_HOST):$(GDB_PORT)" \
	  -ex "load" -ex "$(GDB_RESET)" -ex "break main" -ex "continue"

info:
	@echo "TARGET                = $(TARGET)"
	@echo "CONFIG                = $(CONFIG)"
	@echo "STM32CUBECLT_PATH     = $(STM32CUBECLT_PATH)"
	@echo "GCC_PATH              = $(if $(GCC_PATH),$(GCC_PATH),(PATH))"
	@echo "CC                    = $(CC)  [$(shell $(CC) -dumpversion 2>/dev/null || echo introuvable)]"
	@echo "STM32_PROGRAMMER_CLI  = $(STM32_PROGRAMMER_CLI)"
	@echo "STLINK_GDBSERVER      = $(STLINK_GDBSERVER)"
	@echo "GDB                   = $(GDB)"
	@echo "STM32CUBE_FW_F1       = $(STM32CUBE_FW_F1)"
	@echo "HAL_DRIVER_DIR        = $(HAL_DRIVER_DIR)"
	@echo "CMSIS_DEVICE_DIR      = $(CMSIS_DEVICE_DIR)"
	@echo "CMSIS_CORE_DIR        = $(CMSIS_CORE_DIR)"

clean:
	rm -rf $(BUILD_DIR)

distclean:
	rm -rf $(BUILD_ROOT)
