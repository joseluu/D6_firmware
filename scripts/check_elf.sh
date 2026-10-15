#!/bin/sh
# Vérifications statiques de l'ELF du firmware (sans matériel) :
#   - table des vecteurs à 0x08000000, SP initial = _estack, reset = Reset_Handler (Thumb)
#   - tous les vecteurs non nuls pointent en flash, en mode Thumb
#   - l'image flash s'arrête avant la zone d'émulation EEPROM
#
# Usage : scripts/check_elf.sh <elf> [limite_flash]     (défaut : 0x08008000)
# Variables : NM, OBJCOPY (défaut : arm-none-eabi-nm / arm-none-eabi-objcopy)
set -eu

ELF=$1
LIMIT=$((${2:-0x08008000}))
NM=${NM:-arm-none-eabi-nm}
OBJCOPY=${OBJCOPY:-arm-none-eabi-objcopy}
FLASH_BASE=$((0x08000000))

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
FAIL=0
ok()   { echo "  OK    $*"; }
fail() { echo "  ECHEC $*"; FAIL=1; }
sym()  { v=$("$NM" "$ELF" | awk -v s="$1" '$3 == s { print $1; exit }'); [ -n "$v" ] && echo $((0x$v)); }
hex()  { printf '0x%08x' "$1"; }

echo "Vérification de $ELF"

vt=$(sym g_pfnVectors || true)
[ "$vt" = "$FLASH_BASE" ] && ok "table des vecteurs à $(hex "$FLASH_BASE")" \
                          || fail "table des vecteurs à ${vt:+$(hex "$vt")} au lieu de $(hex "$FLASH_BASE")"

# Mots de 32 bits de la table, en petit-boutiste, indépendamment de l'hôte
"$OBJCOPY" -O binary -j .isr_vector "$ELF" "$TMP/vt.bin"
od -An -v -tu1 "$TMP/vt.bin" | tr -s ' \n' '\n\n' | grep -v '^$' |
	awk '{ b[n++] = $1 } END { for (i = 0; i + 3 < n; i += 4) print b[i] + 256 * (b[i+1] + 256 * (b[i+2] + 256 * b[i+3])) }' > "$TMP/vt.txt"
nvec=$(wc -l < "$TMP/vt.txt")

sp=$(sed -n 1p "$TMP/vt.txt")
estack=$(sym _estack)
[ "$sp" = "$estack" ] && ok "SP initial = _estack = $(hex "$sp")" \
                      || fail "SP initial $(hex "$sp") différent de _estack $(hex "$estack")"

reset=$(sed -n 2p "$TMP/vt.txt")
rh=$(sym Reset_Handler)
[ "$reset" = $((rh | 1)) ] && ok "vecteur reset = Reset_Handler | 1 = $(hex "$reset")" \
                           || fail "vecteur reset $(hex "$reset") différent de Reset_Handler | 1 ($(hex $((rh | 1))))"

bad=0
i=0
while read -r v; do
	i=$((i + 1))
	[ "$i" -eq 1 ] && continue
	[ "$v" -eq 0 ] && continue
	# BootRAM des startups STM32F1 : constante 0xF108F85F, pas une adresse
	[ "$v" -eq $((0xF108F85F)) ] && continue
	if [ $((v & 1)) -ne 1 ] || [ "$v" -lt "$FLASH_BASE" ] || [ "$v" -ge "$LIMIT" ]; then
		fail "vecteur $((i - 1)) = $(hex "$v") (hors flash ou pas en mode Thumb)"
		bad=1
	fi
done < "$TMP/vt.txt"
[ "$bad" -eq 0 ] && ok "$((nvec - 1)) vecteurs en flash et en mode Thumb"

"$OBJCOPY" -O binary "$ELF" "$TMP/fw.bin"
end=$((FLASH_BASE + $(wc -c < "$TMP/fw.bin")))
[ "$end" -le "$LIMIT" ] && ok "image flash $(hex "$FLASH_BASE")-$(hex "$end"), avant la zone EEPROM $(hex "$LIMIT")" \
                        || fail "image flash jusqu'à $(hex "$end"), recouvre la zone EEPROM $(hex "$LIMIT")"

[ "$FAIL" -eq 0 ] && echo "Vérifications statiques : OK" || { echo "Vérifications statiques : ECHEC"; exit 1; }
