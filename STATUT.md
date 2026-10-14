# Statut

Mis à jour à chaque étape de [PLAN.md](PLAN.md). Branche : `claude/affectionate-cerf-8osob9`.

| # | Étape | État |
|---|-------|------|
| - | Build Linux initial (Makefile, `make hal`, variables d'outils) | fait (commit e6f8c93, incomplet, voir étape 0) |
| 0 | Réparer le commit initial | fait |
| 1 | Toolchain GCC 14 | fait |
| 2 | Audit de compatibilité HAL v1.1.10 | fait |
| 3 | Avertissements de compilation | à faire |
| 4 | Appels système newlib | à faire |
| 5 | Makefile `tests/D6_blinkTest` | à faire |
| 6 | Intégration continue GitHub Actions | à faire |
| 7 | Debug en ligne de commande | à faire |
| 8 | Test de fumée hors matériel | à faire |
| 9 | Documentation finale | à faire |

## Journal

### Travail initial
- Nouveau `Makefile` GNU make pour Linux. L'ancien Makefile VisualGDB est renommé `Makefile.visualgdb`.
- Chemins d'outils réglables : `STM32CUBECLT_PATH`, `GCC_PATH`, `STM32CUBE_FW_F1`, `HAL_DRIVER_DIR`, etc., en ligne de commande, dans l'environnement ou dans `config.mk`.
- `make hal` télécharge HAL v1.1.10, CMSIS device v4.3.5 et CMSIS core depuis GitHub.
- Script de lien limité à 32 Ko (zone EEPROM à 0x08008000) ; Debug compilé en `-Og`.
- `return` manquants corrigés dans `EEProm_Read`, `DebouncedButton_buttonState` et `getInputBinary`.
- Testé avec arm-none-eabi-gcc 13.2.1 (paquet Ubuntu) : Debug 21 684 o, Release 23 492 o de flash.

### Étape 0 : réparer le commit initial
- Cause : un `git stash` / `git stash apply` fait pendant les tests avait désindexé les fichiers modifiés, et le commit e6f8c93 ne contenait que les fichiers nouveaux.
- Les fichiers manquants sont committés. Vérification faite sur un `git clone` de la branche distante : `make hal && make` et `make CONFIG=Release` réussissent.

### Étape 1 : toolchain GCC 14
- developer.arm.com est bloqué par le proxy de la session. J'ai utilisé à la place la toolchain xPack GNU Arm Embedded (releases GitHub), même version majeure que STM32CubeCLT 1.21.0 (GCC 14), ainsi que GCC 15 pour anticiper les prochaines versions.
- Résultats (`GCC_PATH=.../bin`) :

| GCC | Config | Flash | RAM | Avertissements |
|-----|--------|-------|-----|----------------|
| 13.2.1 (Ubuntu) | Debug | 21 684 o | 2 788 o | 30 |
| 14.2.1 (xPack) | Debug | 20 428 o | 2 784 o | 31 |
| 14.2.1 (xPack) | Release | 22 496 o | 2 792 o | 33 |
| 15.2.1 (xPack) | Debug | 20 564 o | 2 784 o | 31 |
| 15.2.1 (xPack) | Release | 23 352 o | 2 792 o | 33 |

- Aucun avertissement ne vient du HAL v1.1.10 ni de CMSIS : tous sont dans `Src/` et `Inc/`.
- En Release apparaissent `-Wrestrict` (strncpy avec zones qui se recouvrent) et `-Wmaybe-uninitialized` (`serialObject`) dans `Src/Serial.cpp`. Ce sont peut-être de vrais bugs, analysés à l'étape 3.
- 6 avertissements de lien newlib (`_write`, `_read`, ...) : traités à l'étape 4.

### Étape 2 : audit de compatibilité HAL v1.1.10 / CMSIS v4.3.5
- **`Inc/stm32f1xx_hal_conf.h`** : les modules activés (ADC, CORTEX, DMA, FLASH, GPIO, PWR, RCC, SPI, TIM, UART) n'ont pas changé. Le modèle v1.1.10 définit explicitement `USE_HAL_*_REGISTER_CALLBACKS` et `USE_SPI_CRC`, absents ici. Une macro non définie vaut 0 dans un `#if`, le résultat était donc le même. Je les ai ajoutées à 0U pour suivre le modèle : le .bin Debug et Release est identique octet pour octet avant et après.
- **`Src/system_stm32f1xx.c`** (CMSIS V4.2.0 de 2017) : conservé. Différences avec le modèle v4.3.5 :
  - l'ancien fichier remet RCC à zéro dans `SystemInit()` ;
  - il écrit toujours `SCB->VTOR = FLASH_BASE` (le nouveau seulement si `USER_VECT_TAB_ADDRESS`) ;
  - `SystemCoreClock` démarre à 72 MHz (8 MHz dans le modèle).
  Aucune de ces différences n'empêche de compiler avec le nouveau HAL, et le comportement est celui du build VisualGDB, qui compilait déjà ce fichier. Ne pas le remplacer sans tester sur la carte.
- **`startup/startup_stm32f103xb.s`** : seule différence avec le modèle actuel, `SystemInit` est appelé après la copie de .data et la mise à zéro de .bss (au lieu d'avant). Pas de conséquence ici, puisque `SystemInit` n'utilise pas de variable globale initialisée. Conservé.
- **Point de vigilance** : le build VisualGDB utilisait le startup `.c` du BSP VisualGDB, le nouveau build utilise le startup `.s` du dépôt (CubeMX/Atollic, cohérent avec `STM32F103C8_FLASH.ld`). J'ai vérifié sur l'ELF :
  - tous les handlers d'interruption du code (`SysTick`, `USART1`, `ADC1_2`, `DMA1_Channel5`, les exceptions Cortex) sont des symboles forts présents dans la table des vecteurs ;
  - les callbacks HAL définis en C++ (`HAL_UART_RxCpltCallback`, `TxCplt`, `Error`) sont bien en `extern "C"` et remplacent les versions `weak` du HAL ;
  - la table des vecteurs est à 0x08000000 et `_estack` vaut 0x20005000.
