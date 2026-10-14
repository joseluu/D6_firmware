#!/bin/sh
# Télécharge le HAL STM32F1 et CMSIS depuis les dépôts GitHub de STMicroelectronics
# et les installe avec l'arborescence d'un package STM32CubeF1 :
#
#   <dest>/Drivers/STM32F1xx_HAL_Driver
#   <dest>/Drivers/CMSIS/Device/ST/STM32F1xx
#   <dest>/Drivers/CMSIS/Include
#
# Usage : scripts/fetch_hal.sh [dest]        (défaut : third_party/STM32CubeF1)
# Variables : HAL_DRIVER_VERSION, CMSIS_DEVICE_VERSION, CMSIS_CORE_VERSION, ST_GITHUB, GIT
set -eu

DEST=${1:-third_party/STM32CubeF1}
HAL_DRIVER_VERSION=${HAL_DRIVER_VERSION:-v1.1.10}
CMSIS_DEVICE_VERSION=${CMSIS_DEVICE_VERSION:-v4.3.5}
CMSIS_CORE_VERSION=${CMSIS_CORE_VERSION:-v5.9.0_20250520}
ST_GITHUB=${ST_GITHUB:-https://github.com/STMicroelectronics}
GIT=${GIT:-git}

fetch() { # repo version destination
    if [ -d "$3/.git" ]; then
        echo "== $1 déjà présent dans $3 (supprimer le répertoire pour le re-télécharger)"
        return
    fi
    echo "== $1 $2 -> $3"
    mkdir -p "$(dirname "$3")"
    "$GIT" -c advice.detachedHead=false clone --quiet --depth 1 --branch "$2" "$ST_GITHUB/$1.git" "$3"
}

fetch stm32f1xx_hal_driver "$HAL_DRIVER_VERSION" "$DEST/Drivers/STM32F1xx_HAL_Driver"
fetch cmsis_device_f1 "$CMSIS_DEVICE_VERSION" "$DEST/Drivers/CMSIS/Device/ST/STM32F1xx"

# Seul CMSIS/Core/Include est nécessaire : on le copie dans Drivers/CMSIS/Include
if [ ! -f "$DEST/Drivers/CMSIS/Include/core_cm3.h" ]; then
    TMP=$(mktemp -d)
    trap 'rm -rf "$TMP"' EXIT
    fetch cmsis_core "$CMSIS_CORE_VERSION" "$TMP/cmsis_core"
    mkdir -p "$DEST/Drivers/CMSIS/Include"
    cp "$TMP"/cmsis_core/Include/*.h "$DEST/Drivers/CMSIS/Include/"
    cp "$TMP"/cmsis_core/LICENSE* "$DEST/Drivers/CMSIS/" 2>/dev/null || true
fi

echo "HAL STM32F1 installé dans $DEST"
