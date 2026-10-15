# Statut

Mis à jour à chaque étape de [PLAN.md](PLAN.md). Branche : `claude/affectionate-cerf-8osob9`.

## Synthèse

Toutes les étapes du plan sont terminées.
- Le firmware et le test blink se compilent sous Linux avec GNU make, sans aucun avertissement, avec GCC 13.2, 14.2 (version majeure de STM32CubeCLT 1.21.0) et 15.2, et le HAL STM32CubeF1 v1.1.10.
- Les chemins des outils se règlent par variables : ligne de commande, environnement ou `config.mk`.
- Contrôles automatiques : vérifications statiques de l'ELF et démarrage sous QEMU.
- CI GitHub Actions vert (Debug et Release).
- Rien n'a été fusionné dans `master` et aucune PR n'a été ouverte.

### À valider sur la carte D6 (impossible dans cette session : pas de matériel, pas de STM32CubeCLT réel)
1. `make flash` avec STM32CubeCLT 1.21.0 et un ST-LINK V2 ; `make gdbserver` + `make debug`.
2. Démarrage complet : horloge HSE/PLL à 72 MHz, LED, bouton.
3. Liaison série et protocole NWT, en particulier les commandes binaires (`i`), concernées par la correction `memmove` de `Serial.cpp`.
4. EEPROM : lecture et écriture des paramètres (`EEProm_Read` renvoie maintenant la valeur lue ; avant, la valeur de retour était indéfinie).
5. Pilotage des deux ADF4351 et mesures ADC (balayages `a`/`b`/`w`/`x`).
6. Remarque : la page 1 de l'émulation EEPROM est à `EEPROM_START_ADDRESS + 0x10000` = 0x08018000, donc au-delà des 64 Ko d'un STM32F103C8. Cela ne fonctionne que si la puce a en réalité 128 Ko (cas fréquent des C8). C'est antérieur à ce travail et non modifié.

| # | Étape | État |
|---|-------|------|
| - | Build Linux initial (Makefile, `make hal`, variables d'outils) | fait (commit e6f8c93, incomplet, voir étape 0) |
| 0 | Réparer le commit initial | fait |
| 1 | Toolchain GCC 14 | fait |
| 2 | Audit de compatibilité HAL v1.1.10 | fait |
| 3 | Avertissements de compilation | fait |
| 4 | Appels système newlib | fait |
| 5 | Makefile `tests/D6_blinkTest` | fait |
| 6 | Intégration continue GitHub Actions | fait |
| 7 | Debug en ligne de commande | fait |
| 8 | Test de fumée hors matériel | fait |
| 9 | Documentation finale | fait |

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
| 13.2.1 (Ubuntu) | Debug | 21 684 o | 2 788 o | 31 |
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

### Étape 3 : avertissements de compilation
Il ne reste plus aucun avertissement dans `Src/` et `Inc/`, en Debug comme en Release, avec GCC 13.2, 14.2 et 15.2.

| GCC | Debug | Release |
|-----|-------|---------|
| 13.2.1 | 21 904 o | 23 712 o |
| 14.2.1 | 20 448 o | 22 516 o |
| 15.2.1 | 20 580 o | 23 348 o |

**Corrections qui changent le comportement (vrais bugs) :**
- `Src/Serial.cpp`, `fgetc` et `fgetsNonBlocking` décalaient le tampon de réception sur lui-même avec `strncpy`. Les zones se recouvrent, ce qui est un comportement indéfini. En plus, `strncpy` s'arrête au premier octet nul et remplit la suite de zéros, ce qui corrompait les données binaires reçues (commandes NWT `i`, via `getInputBinary`). Remplacé par `memmove`.
- `Src/Loop.cpp`, `getInputBinary(unsigned long&)` : `inputBytes[3] << 24` décalait un `int` jusque dans le bit de signe dès que l'octet valait 0x80 ou plus (comportement indéfini). Conversion en `unsigned long` avant le décalage.

**Corrections sans effet sur le comportement :**
- ordre des listes d'initialisation (`Serial.h`/`.cpp`, `debouncedButton.h`) ;
- variables et instruction inutilisées (`Loop.cpp`, `Register_Set` dans `ADF4351.cpp`) ;
- comparaison signé/non signé ;
- `default:` dans le `switch` de `doNWT` ;
- dans les callbacks UART, `return` après `Error_Handler()` (qui ne rend jamais la main) ; l'avertissement « maybe-uninitialized » était un faux positif ;
- le code d'erreur UART est rangé dans une globale `volatile uartLastError`, mise à jour à chaque appel, à la place d'une `static` locale initialisée une seule fois.

**Observation non corrigée (comportement du protocole, à décider) :** dans `doNWT`, `RESTART_IF_FAILED` remet `state = eFrame` mais n'interrompt pas l'analyse. `bCommandReady` passe ensuite à `true` et la commande est exécutée quand même avec des arguments incomplets. Par ailleurs, après une commande sans `bCommandReady` (`e`, `m`, `n`, `o`, `r`, ou un code inconnu), l'état reste `eCode`.

### Étape 4 : appels système newlib
- Origine des avertissements : `operator new` (libstdc++) mène à `abort`, `raise`, `_kill` et `_getpid`, et le stdio de newlib-nano tire `_write`, `_read`, `_close` et `_lseek`. Avec `--specs=nosys.specs`, ces fonctions venaient de libnosys, sous forme de bouchons qui déclenchent l'avertissement « is not implemented ».
- Après le ramasse-miettes du linker (`--gc-sections`), `_write`, `_read`, `_close` et `_lseek` n'étaient de toute façon pas dans le binaire. En revanche, `_sbrk` (utilisé par les deux `new` de l'initialisation) restait la version libnosys, qui ne vérifie pas que le tas ne déborde pas sur la pile.
- Solution : ajout au build de `Src/syscalls.c`, déjà présent dans le dépôt (version Atollic/STM32CubeIDE). Son `_sbrk` renvoie `ENOMEM` en cas de collision tas/pile, et il fournit `_kill`, `_getpid`, etc.
- Résultat : **0 avertissement** (compilation et lien) avec GCC 13.2, 14.2 et 15.2, en Debug et en Release. Le binaire grossit d'environ 40 octets.
- À savoir : `_write` et `_read` de `syscalls.c` appellent `__io_putchar` et `__io_getchar`, déclarées `weak` et non définies dans le projet. Si un jour on utilise `printf`, il faudra définir `__io_putchar` (par exemple vers l'UART), sinon l'appel aboutira à une HardFault.

### Étape 5 : Makefile Linux pour `tests/D6_blinkTest`
- La détection des outils, les variables et les règles sont passées du `Makefile` racine vers un fichier commun **`mk/stm32f1.mk`**. Le `Makefile` du firmware et celui du test ne contiennent plus que leurs sources (`TARGET`, `APP_SOURCES`, `APP_INCLUDES`, `STARTUP`, `LDSCRIPT`).
- `config.mk` est cherché à la racine du dépôt, y compris quand on lance make depuis `tests/D6_blinkTest`.
- `tests/D6_blinkTest/Makefile` utilise ses propres `LEDBlink.cpp`, `system_stm32f1xx.c` et `stm32f1xx_hal_conf.h`, plus le startup, le script de lien et `syscalls.c` du firmware. L'ancien Makefile VisualGDB du test est renommé `Makefile.visualgdb`, et ses `.vgdbsettings` sont mis à jour.
- Nouveau contrôle : make s'arrête avec une erreur si deux sources portent le même nom de fichier (les objets sont rangés à plat dans `build/<config>/obj`). Testé.
- Vérifications :
  - les .bin du firmware sont identiques octet pour octet à ceux du commit précédent, en Debug et en Release (le découpage est neutre) ;
  - `make -C tests/D6_blinkTest` (Debug 1 820 o, Release 2 304 o) : 0 avertissement.

### Étape 6 : intégration continue GitHub Actions
- `.github/workflows/build.yml` : à chaque push et pull request, sur ubuntu-24.04, en Debug et en Release :
  - installation de la toolchain xPack GCC 14.2.1-1.1 (même version majeure que STM32CubeCLT 1.21), vérifiée par SHA-256 et mise en cache ;
  - `make hal` (avec cache) ;
  - build du firmware et du test blink ;
  - publication des `.elf`, `.hex`, `.bin` et `.map` en artefacts.
- STM32CubeCLT n'est pas utilisé en CI : son téléchargement exige un compte st.com et l'acceptation de la licence.
- Les commandes du workflow ont été rejouées en local sur une copie des fichiers suivis par git : OK.

### Étape 7 : debug en ligne de commande
- `make gdbserver` lance `ST-LINK_gdbserver` en SWD, en mode persistant (`-e`), sur le port `GDB_PORT` (61234). Le chemin de STM32CubeProgrammer passé avec `-cp` vient de STM32CubeCLT ou, à défaut, du répertoire de `STM32_Programmer_CLI` trouvé dans le PATH. Si rien n'est trouvé, make s'arrête avec un message clair au lieu de lancer le serveur avec un chemin vide.
- `make debug` lance `arm-none-eabi-gdb` (celui de la toolchain, variable `GDB`) sur l'ELF. GDB se connecte à `GDB_HOST:GDB_PORT`, fait `load`, puis `$(GDB_RESET)` (par défaut `monitor reset`), et s'arrête sur `main()`.
- Testé contre le serveur GDB de QEMU (`GDB_PORT=1234 GDB_RESET="monitor system_reset"`) : programmation, reset et arrêt sur `main()` à `HAL_Init()`. Non testé avec un vrai ST-LINK (pas de matériel dans cette session).
- La configuration VS Code (option du plan) n'est pas ajoutée : les chemins dépendent de chaque poste, et `make gdbserver` / `make debug` couvrent l'usage en ligne de commande.

### Étape 8 : tests hors matériel
- **`make check`** (`scripts/check_elf.sh`), vérifications statiques de l'ELF :
  - la table des vecteurs est à 0x08000000 et le SP initial vaut `_estack` ;
  - le vecteur reset vaut `Reset_Handler|1` ;
  - tous les vecteurs non nuls pointent en flash et en mode Thumb (la constante `BootRAM` 0xF108F85F est ignorée) ;
  - l'image flash se termine avant la zone EEPROM (`FLASH_LIMIT`, par défaut 0x08008000).
  OK pour le firmware et pour le test blink, en Debug et en Release. Test négatif : avec `FLASH_LIMIT` à 0x08005000, le contrôle échoue comme prévu.
- **`make check-qemu`** (`scripts/qemu_smoke.sh`) : QEMU 8.2, machine `stm32vldiscovery` (STM32F100RB, Cortex-M3, 8 Ko de RAM). Le firmware est relié avec une variante du script de lien (pile à 0x20002000, RAM 8 Ko) dans `build/qemu/`, puis piloté par GDB. Contrôles :
  - `main()` est atteint ;
  - `.data` est copié (`SystemCoreClock` = 72000000) et `.bss` est à zéro (`uwTick` = 0) ;
  - aucune HardFault ;
  - arrêt dans `Error_Handler()` depuis `SystemClock_Config()`. C'est attendu, car QEMU n'émule pas l'oscillateur HSE du STM32F1 ;
  - les interruptions SysTick fonctionnent (`uwTick` = 101 après le timeout HSE de 100 ms du HAL).
  OK en Debug et en Release. Test négatif : l'ELF normal (pile à 0x20005000, hors des 8 Ko de RAM de QEMU) échoue comme prévu.
- Limite : QEMU n'émule ni RCC, ni GPIO, ni SPI, ni l'ADF4351 du STM32F1. Le comportement après la configuration d'horloge (UART, protocole NWT, EEPROM) reste à valider sur la carte.
- Ces deux contrôles sont ajoutés au CI. Le premier run CI ([38656970746](https://github.com/joseluu/D6_firmware/actions/runs/38656970746)), build Debug et Release, est vert.

### Étape 9 : documentation finale
- README : section Linux réécrite (outils, commandes, tableau complet des variables, organisation des fichiers, EEPROM, QEMU) et badge CI.
- `config.mk.example` complété (`GDB*`, `FLASH_LIMIT`, `QEMU`). Testé : un `config.mk` qui fixe `CONFIG = Release` est bien pris en compte.
- Relecture complète du diff par rapport à `master` : une coquille corrigée dans `stm32f1xx_hal_conf.h` (`USE_HAL_SMARTCARD_REGISTER_CALLBACKS0U`, sans espace ; sans effet sur le binaire, vérifié identique).
- Run CI [38657531524](https://github.com/joseluu/D6_firmware/actions/runs/38657531524) (build, `make check`, `make check-qemu`) : vert en Debug et en Release.
