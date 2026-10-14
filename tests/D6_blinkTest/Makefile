# D6_blinkTest - Makefile Linux (GNU make) : clignotement des LED PA11 / PB1
#
# Usage (depuis ce répertoire, ou "make -C tests/D6_blinkTest" depuis la racine) :
#   make | make CONFIG=Release | make flash | make info
#
# Outils, chemins et options : voir ../../mk/stm32f1.mk et ../../config.mk.example.

TARGET := D6_blinkTest

APP_SOURCES  := LEDBlink.cpp system_stm32f1xx.c ../../Src/syscalls.c
APP_INCLUDES := .
STARTUP      := ../../startup/startup_stm32f103xb.s
LDSCRIPT     ?= ../../STM32F103C8_FLASH.ld

include ../../mk/stm32f1.mk
