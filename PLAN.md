# Plan : build Linux de D6_firmware avec les outils ST actuels

Objectif : compiler le projet sous Linux avec GNU make et les outils ST en ligne de commande (STM32CubeCLT 1.21.0, GCC 14, STM32CubeF1 V1.8.7 / HAL v1.1.10), avec des chemins d'outils réglables par variables.
Branche de travail : `claude/affectionate-cerf-8osob9`. Chaque étape fait l'objet d'un commit, et STATUT.md est mis à jour à chaque étape.

| # | Étape | Contenu | Critère de fin |
|---|-------|---------|----------------|
| 0 | Réparer le commit initial | Le premier commit poussé ne contenait que les fichiers nouveaux. Il faut committer le Makefile, le README, le script de lien et les corrections de sources, puis vérifier le build depuis un clone du dépôt distant. | Un `git clone` de la branche suivi de `make hal && make` réussit. |
| 1 | Toolchain GCC 14 | Compiler avec la même version majeure de GCC que STM32CubeCLT 1.21 (GCC 14). Si developer.arm.com n'est pas accessible, chercher une autre source (paquet de distribution, conteneur). | Debug et Release compilent avec GCC 14, et la liste des avertissements est relevée. |
| 2 | Audit de compatibilité HAL v1.1.10 | Comparer `Inc/stm32f1xx_hal_conf.h`, `Src/system_stm32f1xx.c` et `startup/startup_stm32f103xb.s` aux modèles de STM32CubeF1 V1.8.7 / cmsis_device_f1 v4.3.5, et mettre à jour ce qui doit l'être sans changer le comportement. | Aucune incompatibilité connue ; les différences sont documentées. |
| 3 | Avertissements de compilation | Corriger les avertissements `-Wall` du code applicatif (variables inutilisées, ordre d'initialisation, comparaisons signé/non signé, `switch` incomplets) sans changer le comportement. | Build Debug et Release sans avertissement dans `Src/` et `Inc/`. |
| 4 | Appels système newlib | Traiter les avertissements « _write/_read/... is not implemented » : ajouter au build le `Src/syscalls.c` existant, ou documenter pourquoi il en reste exclu. | Lien sans avertissement, ou raison écrite dans STATUT.md. |
| 5 | Makefile Linux pour `tests/D6_blinkTest` | Même approche et mêmes variables que le Makefile principal. | `make -C tests/D6_blinkTest` produit un fichier .elf. |
| 6 | Intégration continue GitHub Actions | Build Debug et Release à chaque push (`make hal`, puis la toolchain ARM), avec les fichiers .elf/.hex/.bin en artefacts. | Workflow ajouté. Il ne peut pas être exécuté depuis cette session, ce qui sera signalé. |
| 7 | Debug en ligne de commande | Ajouter une cible `make debug` (arm-none-eabi-gdb connecté à ST-LINK_gdbserver) et, en option, une configuration VS Code (STM32CubeIDE for VS Code / Cortex-Debug). | Cibles écrites et documentées. |
| 8 | Test de fumée hors matériel | Étudier la faisabilité d'un démarrage sous QEMU ou Renode jusqu'à `main()`. À défaut, vérifier l'ELF de façon statique (table des vecteurs, Reset_Handler, adresses, taille sous 32 Ko). | Script de vérification ajouté (cible `make check`). |
| 9 | Documentation finale | README à jour, synthèse dans STATUT.md, liste des points à valider sur le matériel (flash, EEPROM). | Push final. |

Hors périmètre : modifier le comportement du firmware, régénérer le projet avec STM32CubeMX, fusionner dans `master` ou ouvrir une PR (à décider par le propriétaire du dépôt).
