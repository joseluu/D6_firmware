# D6_firmware

[![build](https://github.com/joseluu/D6_firmware/actions/workflows/build.yml/badge.svg)](https://github.com/joseluu/D6_firmware/actions/workflows/build.yml)

This is an alternate firmware for the D6 spectrum analyzer with tracking source

D6 board can also be used with the ADI PLL test software ADF435x (use the modified version in the ADF435x repo)

discussions en francais sur le sujet, voir ici: https://gitter.im/D6_firmware/D6_en_fran%C3%A7ais

<img src="https://raw.githubusercontent.com/joseluu/D6_firmware/master/Documents/Photos/D6_and_STLinkV2.jpg" width="300">





## Compilation sous Linux (outils ST en ligne de commande)

### Outils

- [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) 1.21.0 : toolchain `arm-none-eabi` (GNU-tools-for-STM32, GCC 14), STM32CubeProgrammer et ST-LINK gdbserver.
  Pour compiler seulement, n'importe quelle toolchain `arm-none-eabi-gcc` récente convient (testé avec GCC 13.2, 14.2 et 15.2) : ARM GNU Toolchain, xPack ou paquet `gcc-arm-none-eabi` de la distribution.
- Le HAL STM32F1 : package STM32CubeF1 V1.8.7 (HAL v1.1.10, CMSIS device v4.3.5). Le Makefile le trouve tout seul s'il a été installé par STM32CubeMX ou STM32CubeIDE dans `~/STM32Cube/Repository`. Sinon, `make hal` le télécharge depuis les dépôts GitHub de STMicroelectronics dans `third_party/STM32CubeF1`.
- GNU make et git. Pour `make check-qemu` : `qemu-system-arm` et `arm-none-eabi-gdb`.

### Commandes

```sh
make hal                 # une seule fois, si STM32CubeF1 n'est pas installé
make                     # build Debug   -> build/Debug/D6_firmware.{elf,bin,hex,map}
make CONFIG=Release      # build Release -> build/Release/...
make check               # vérifications statiques de l'ELF (vecteurs, pile, taille sous la zone EEPROM)
make check-qemu          # test de démarrage sous QEMU (sans matériel)
make flash               # programmation via ST-LINK (STM32_Programmer_CLI)
make erase               # effacement complet de la flash, paramètres EEPROM compris
make gdbserver           # ST-LINK_gdbserver sur le port 61234 (dans un premier terminal)
make debug               # GDB : programmation, reset, arrêt sur main() (dans un second terminal)
make info                # outils et chemins utilisés
make clean | distclean

make -C tests/D6_blinkTest [CONFIG=Release] [flash]   # programme de test : clignotement des LED
```

### Emplacement des outils

Les chemins sont des variables make. Par défaut, le Makefile prend le STM32CubeCLT le plus récent installé dans `/opt/st/stm32cubeclt_*`, et sinon cherche les outils dans le `PATH`. Pour changer un chemin :

- en ligne de commande : `make STM32CUBECLT_PATH=/opt/st/stm32cubeclt_1.21.0` ;
- dans l'environnement : `export STM32CUBE_FW_F1=~/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7` ;
- dans un fichier `config.mk` local à la racine du dépôt, non versionné : copier `config.mk.example`. Il s'applique aussi à `tests/D6_blinkTest`.

| Variable | Rôle | Défaut |
|---|---|---|
| `STM32CUBECLT_PATH` | racine de STM32CubeCLT | dernier `/opt/st/stm32cubeclt_*` |
| `GCC_PATH` | répertoire `bin` de la toolchain (vide = `PATH`) | `$(STM32CUBECLT_PATH)/GNU-tools-for-STM32/bin` |
| `PREFIX` | préfixe des outils | `arm-none-eabi-` |
| `STM32CUBE_FW_F1` | racine d'un package STM32CubeF1 (chemin absolu) | dernier `~/STM32Cube/Repository/STM32Cube_FW_F1_V*`, sinon `third_party/STM32CubeF1` |
| `HAL_DRIVER_DIR`, `CMSIS_DEVICE_DIR`, `CMSIS_CORE_DIR` | composants HAL/CMSIS, un par un | sous `$(STM32CUBE_FW_F1)/Drivers` |
| `STM32_PROGRAMMER_CLI` | STM32CubeProgrammer CLI | `$(STM32CUBECLT_PATH)/STM32CubeProgrammer/bin/STM32_Programmer_CLI` |
| `STM32_PROGRAMMER_PATH` | répertoire de STM32CubeProgrammer, pour `gdbserver` | idem, ou répertoire de `STM32_Programmer_CLI` dans le `PATH` |
| `STLINK_GDBSERVER` | ST-LINK gdbserver | `$(STM32CUBECLT_PATH)/STLink-gdb-server/bin/ST-LINK_gdbserver` |
| `STLINK_PORT` | port de STM32_Programmer_CLI | `SWD` |
| `GDB`, `GDB_HOST`, `GDB_PORT`, `GDB_RESET` | session `make debug` | GDB de la toolchain, `localhost`, `61234`, `monitor reset` |
| `CONFIG` | `Debug` ou `Release` | `Debug` |
| `OPT` | niveau d'optimisation | `-Og` (Debug), `-O3` (Release) |
| `CFLAGS`, `CXXFLAGS`, `ASFLAGS`, `LDFLAGS` | options ajoutées à celles du Makefile | vide |
| `FLASH_LIMIT` | fin autorisée de l'image flash (`make check`) | `0x08008000` |
| `QEMU` | émulateur pour `make check-qemu` | `qemu-system-arm` |

### Organisation

- `Makefile`, `tests/D6_blinkTest/Makefile` : liste des sources de chaque programme ;
- `mk/stm32f1.mk` : détection des outils, options et règles communes ;
- `scripts/fetch_hal.sh` (`make hal`), `scripts/check_elf.sh` (`make check`), `scripts/qemu_smoke.sh` (`make check-qemu`) ;
- `.github/workflows/build.yml` : intégration continue (build Debug et Release avec GCC 14, contrôles statiques, test QEMU, binaires en artefacts) ;
- `Makefile.visualgdb`, `*.vgdbsettings`, `*.vcxproj` : ancien projet VisualGDB (Windows), conservé tel quel.

### Mémoire flash et EEPROM

L'émulation EEPROM (`Inc/eeprom.h`) utilise la flash à partir de `0x08008000`. Le script de lien limite donc le code à 32 Ko : un débordement est refusé au moment du lien au lieu d'écraser les paramètres sauvegardés. C'est pour cela que le Debug est compilé en `-Og` : en `-O0`, il dépasse 32 Ko.

### Test sous QEMU

QEMU n'émule pas le STM32F103. `make check-qemu` utilise la machine `stm32vldiscovery` (STM32F100, 8 Ko de RAM) avec une variante du script de lien, et vérifie :

- le démarrage jusqu'à `main()` ;
- l'initialisation de `.data` et de `.bss` ;
- les interruptions SysTick ;
- l'absence de HardFault ;
- l'arrêt attendu dans `SystemClock_Config()`, puisque l'oscillateur HSE n'est pas émulé.

La suite (UART, ADF4351, EEPROM) se teste sur la carte.
