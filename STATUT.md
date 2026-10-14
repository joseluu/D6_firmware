# Statut

Mis à jour à chaque étape de [PLAN.md](PLAN.md). Branche : `claude/affectionate-cerf-8osob9`.

| # | Étape | État |
|---|-------|------|
| - | Build Linux initial (Makefile, `make hal`, variables d'outils) | fait (commit e6f8c93, incomplet, voir étape 0) |
| 0 | Réparer le commit initial | en cours |
| 1 | Toolchain GCC 14 | à faire |
| 2 | Audit de compatibilité HAL v1.1.10 | à faire |
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
