# D6_firmware

This is an alternate firmware for the D6 spectrum analyzer with tracking source

D6 board can also be used with the ADI PLL test software ADF435x (use the modified version in the ADF435x repo)

discussions en francais sur le sujet, voir ici: https://gitter.im/D6_firmware/D6_en_fran%C3%A7ais

<img src="https://raw.githubusercontent.com/joseluu/D6_firmware/master/Documents/Photos/D6_and_STLinkV2.jpg" width="300">





## Compilation sous Linux (outils ST en ligne de commande)

Outils nécessaires :

- [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) (testé avec la 1.21.0) : toolchain `arm-none-eabi` (GNU-tools-for-STM32), STM32CubeProgrammer, ST-LINK gdbserver.
  À défaut, n'importe quelle toolchain `arm-none-eabi-gcc` récente (ARM GNU Toolchain, paquet `gcc-arm-none-eabi` de la distribution) convient pour compiler.
- Le HAL STM32F1 (package STM32CubeF1 V1.8.7, HAL v1.1.10). Il est trouvé automatiquement s'il a été installé par STM32CubeMX/STM32CubeIDE dans `~/STM32Cube/Repository`. Sinon, `make hal` le télécharge depuis les dépôts GitHub de STMicroelectronics dans `third_party/STM32CubeF1`.
- GNU make, git.

```sh
make hal                 # une seule fois, si pas de STM32CubeF1 installé
make                     # build Debug   -> build/Debug/D6_firmware.{elf,bin,hex,map}
make CONFIG=Release      # build Release -> build/Release/...
make flash               # programmation via ST-LINK (STM32_Programmer_CLI)
make CONFIG=Release flash
make gdbserver           # ST-LINK_gdbserver sur le port 61234
make info                # affiche les outils et chemins utilisés
make clean | distclean
```

### Emplacement des outils

Les chemins sont des variables make. Par défaut, le Makefile prend le STM32CubeCLT le plus récent installé dans `/opt/st/stm32cubeclt_*`, et sinon cherche les outils dans le `PATH`. Pour changer un chemin :

- en ligne de commande : `make STM32CUBECLT_PATH=/opt/st/stm32cubeclt_1.21.0`
- dans l'environnement : `export STM32CUBE_FW_F1=~/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7`
- dans un fichier `config.mk` local, non versionné (copier `config.mk.example`).

| Variable | Rôle | Défaut |
|---|---|---|
| `STM32CUBECLT_PATH` | racine de STM32CubeCLT | dernier `/opt/st/stm32cubeclt_*` |
| `GCC_PATH` | répertoire `bin` de la toolchain (vide = `PATH`) | `$(STM32CUBECLT_PATH)/GNU-tools-for-STM32/bin` |
| `PREFIX` | préfixe des outils | `arm-none-eabi-` |
| `STM32CUBE_FW_F1` | racine d'un package STM32CubeF1 | dernier `~/STM32Cube/Repository/STM32Cube_FW_F1_V*`, sinon `third_party/STM32CubeF1` |
| `HAL_DRIVER_DIR`, `CMSIS_DEVICE_DIR`, `CMSIS_CORE_DIR` | composants HAL/CMSIS, un par un | sous `$(STM32CUBE_FW_F1)/Drivers` |
| `STM32_PROGRAMMER_CLI` | STM32CubeProgrammer CLI | `$(STM32CUBECLT_PATH)/STM32CubeProgrammer/bin/STM32_Programmer_CLI` |
| `STLINK_GDBSERVER` | ST-LINK gdbserver | `$(STM32CUBECLT_PATH)/STLink-gdb-server/bin/ST-LINK_gdbserver` |
| `CONFIG` | `Debug` ou `Release` | `Debug` |
| `OPT` | niveau d'optimisation | `-Og` (Debug), `-O3` (Release) |
| `CFLAGS`, `CXXFLAGS`, `LDFLAGS` | options ajoutées à celles du Makefile | vide |

### Note sur la mémoire flash

L'émulation EEPROM (`Inc/eeprom.h`) utilise la flash à partir de `0x08008000`. Le script de lien limite donc le code à 32 Ko : un débordement est refusé au moment du lien au lieu d'écraser les paramètres sauvegardés. C'est pour cela que le Debug est compilé en `-Og` : en `-O0`, il dépasse 32 Ko.

Les anciens fichiers VisualGDB (Windows) sont conservés : le projet VisualGDB utilise maintenant `Makefile.visualgdb`.
