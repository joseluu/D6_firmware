#!/bin/sh
# Test de fumée du firmware sous QEMU (machine stm32vldiscovery : Cortex-M3, STM32F100RB).
#
# Il faut un ELF lié pour 8 Ko de RAM (cible "make check-qemu"). QEMU n'émule ni la RCC
# ni l'oscillateur HSE du STM32F1 : on vérifie donc le démarrage jusqu'à l'échec attendu
# de SystemClock_Config(), ce qui valide :
#   - la table des vecteurs, la pile et Reset_Handler
#   - la copie de .data (SystemCoreClock = 72 MHz à l'entrée de main) et la mise à zéro de .bss (uwTick = 0)
#   - les interruptions SysTick (uwTick augmente pendant le timeout HSE du HAL)
#   - l'absence de HardFault
#
# Usage : scripts/qemu_smoke.sh <elf>
# Variables : QEMU (qemu-system-arm), GDB (arm-none-eabi-gdb), QEMU_GDB_PORT (1234), TIMEOUT (30 s)
set -eu

ELF=$1
QEMU=${QEMU:-qemu-system-arm}
GDB=${GDB:-arm-none-eabi-gdb}
PORT=${QEMU_GDB_PORT:-1234}
TIMEOUT=${TIMEOUT:-30}

command -v "$QEMU" >/dev/null || { echo "*** $QEMU introuvable (paquet qemu-system-arm)"; exit 2; }
command -v "$GDB" >/dev/null || { echo "*** $GDB introuvable"; exit 2; }

TMP=$(mktemp -d)
"$QEMU" -M stm32vldiscovery -nographic -monitor none -serial none -S -gdb tcp:127.0.0.1:"$PORT" \
	-kernel "$ELF" > "$TMP/qemu.log" 2>&1 &
QPID=$!
trap 'kill $QPID 2>/dev/null || true; rm -rf "$TMP"' EXIT
sleep 1

cat > "$TMP/cmds.gdb" <<EOF
set pagination off
set confirm off
target remote 127.0.0.1:$PORT
break main
break Error_Handler
break HardFault_Handler
continue
printf "CHECK SystemCoreClock=%u\n", SystemCoreClock
printf "CHECK uwTick_main=%u\n", uwTick
continue
bt
printf "CHECK uwTick_error=%u\n", uwTick
kill
EOF
timeout "$TIMEOUT" "$GDB" -q -batch -x "$TMP/cmds.gdb" "$ELF" > "$TMP/gdb.log" 2>&1 || true

val() { sed -n "s/^CHECK $1=//p" "$TMP/gdb.log"; }
FAIL=0
ok()   { echo "  OK    $*"; }
fail() { echo "  ECHEC $*"; FAIL=1; }

echo "Test QEMU de $ELF"
if grep -q "^Breakpoint 1, main ()" "$TMP/gdb.log"; then
	ok "main() atteint"
	[ "$(val SystemCoreClock)" = "72000000" ] && ok ".data initialisé (SystemCoreClock = 72000000)" \
		|| fail ".data : SystemCoreClock = '$(val SystemCoreClock)'"
	[ "$(val uwTick_main)" = "0" ] && ok ".bss à zéro (uwTick = 0 à l'entrée de main)" \
		|| fail ".bss : uwTick = '$(val uwTick_main)'"
else
	fail "main() non atteint (.data et .bss non vérifiés)"
fi
if grep -q "HardFault_Handler" "$TMP/gdb.log" && grep -q "^Breakpoint 3," "$TMP/gdb.log"; then
	fail "HardFault"
else
	ok "pas de HardFault"
fi
grep -q "in SystemClock_Config" "$TMP/gdb.log" && grep -q "^Breakpoint 2, Error_Handler" "$TMP/gdb.log" \
	&& ok "Error_Handler depuis SystemClock_Config (attendu : HSE non émulé)" \
	|| fail "arrêt attendu dans Error_Handler depuis SystemClock_Config"
t=$(val uwTick_error)
[ -n "$t" ] && [ "$t" -gt 0 ] && ok "interruptions SysTick actives (uwTick = $t)" || fail "SysTick : uwTick = '$t'"

if [ "$FAIL" -ne 0 ]; then
	echo "--- journal GDB"; cat "$TMP/gdb.log"; echo "--- journal QEMU"; cat "$TMP/qemu.log"
	echo "Test QEMU : ECHEC"; exit 1
fi
echo "Test QEMU : OK"
