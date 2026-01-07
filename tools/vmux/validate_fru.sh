#!/bin/sh
set -eu

# Resolve directory of this script to locate sibling helpers
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Detect virtual I2C bus created by virtual-mux-host bound at bus 3 addr 0x51
find_virtual_bus() {
	dev="3-0051"
	root="/sys/bus/i2c/devices/$dev"
	if [ -d "$root" ]; then
		busdir=$(find "$root" -maxdepth 3 -type d -name 'i2c-*' 2>/dev/null | head -n1 || true)
		if [ -n "$busdir" ]; then
			bname=$(basename "$busdir")
			case "$bname" in
				i2c-*) echo "${bname#i2c-}"; return 0 ;;
			esac
		fi
	fi
	# Fallback to known default (e.g., 90) if detection fails
	echo "${DRIVER_VBUS_OVERRIDE:-90}"
	return 0
}

# Validation harness for FRU programming with or without the host driver.
# - Runs 10 (default) erase/program/verify cycles per FRU
# - Uses provided golden images in --expected-dir
# - Erase uses all-0xFF images (auto-created as needed)
#
# Modes:
#   --mode driver      : use downstream bus/addr with i2ctransfer and write_*_with_driver.sh
#   --mode no-driver   : use vmux/write_fru_no_driver.sh; requires --link-bus and --client-addr
#
# Required files in expected dir:
#   fru_50_16.bin, fru_51_16.bin, fru_52_8.bin, fru_53_8.bin, fru_54_8.bin, fru_55_8.bin, fru_56_8.bin
#
# Example:
#   sh vmux/validate_fru.sh --mode no-driver --link-bus 3 --client-addr 0x51 --expected-dir vmux/expected --cycles 10
#   sh vmux/validate_fru.sh --mode driver --expected-dir vmux/expected --cycles 10

usage() {
	echo "Usage:"
	echo "  $0 --mode <driver|no-driver> [--link-bus <N>] [--client-addr <0xYY>] --expected-dir <DIR> [--cycles <N>]"
	echo
	echo "Modes:"
	echo "  driver    : Uses write_8bit_fru_with_driver.sh / write_16bit_fru_with_driver.sh and i2ctransfer for verify"
	echo "  no-driver : Uses write_fru_no_driver.sh (requires --link-bus and --client-addr)"
}

require_cmd() {
	command -v "$1" >/dev/null 2>&1 || { echo "Error: '$1' not found"; exit 1; }
}

hex_to_int() {
	case "$1" in
		0x*|0X*) printf "%d" "$(( $1 ))" ;;
		*) printf "%d" "$1" ;;
	esac
}

# Fixed FRU inventory (virt, bits, driver downstream bus, driver downstream addr)
fru_inventory() {
	cat <<'EOF'
0x50 16 1  0x50
0x51 16 7  0x50
0x52  8 10 0x50
0x53  8 70 0x54
0x54  8 70 0x55
0x55  8 71 0x54
0x56  8 71 0x55
EOF
}

# Create all-0xFF files if not present
ensure_ff_files() {
	FF256="${EXPECTED_DIR}/ff256.bin"
	FF8192="${EXPECTED_DIR}/ff8192.bin"
	[ -f "$FF256" ] || dd if=/dev/zero bs=256 count=1 2>/dev/null | tr '\0' '\377' > "$FF256"
	[ -f "$FF8192" ] || dd if=/dev/zero bs=8192 count=1 2>/dev/null | tr '\0' '\377' > "$FF8192"
}

# Read helpers (output to file)
read_driver_8() {
	bus="$1"; addr="$2"; out="$3"
	: > "$out"
	# Single 256B read from offset 0
	# write offset 0, then read 256
	i2ctransfer -y -f "$bus" w1@"$addr" 0x00 r256 > "$out"
}

read_driver_16() {
	bus="$1"; addr="$2"; out="$3"
	: > "$out"
	offset=0
	while [ "$offset" -lt 8192 ]; do
		remain=$(( 8192 - offset ))
		chunk=256
		[ $chunk -gt $remain ] && chunk=$remain
		hi=$(( (offset >> 8) & 255 ))
		lo=$(( offset & 255 ))
		# write 16-bit offset then read 'chunk'
		i2ctransfer -y -f "$bus" w2@"$addr" "$(printf '0x%02x' "$hi")" "$(printf '0x%02x' "$lo")" r${chunk} >> "$out"
		offset=$(( offset + chunk ))
	done
}

read_driver_8_bin() {
	bus="$1"; addr="$2"; out="$3"
	len=256
	toks="$(i2ctransfer -y -f "$bus" w1@"$addr" 0x00 r${len}@"$addr")"
	outstr=""
	for t in $toks; do
		h=${t#0x}
		outstr="$outstr\\x$h"
	done
	# shellcheck disable=SC2059
	printf "$outstr" > "$out"
}

read_driver_16_bin() {
	bus="$1"; addr="$2"; out="$3"
	: > "$out"
	offset=0
	while [ "$offset" -lt 8192 ]; do
		remain=$(( 8192 - offset ))
		chunk=256
		[ $chunk -gt $remain ] && chunk=$remain
		hi=$(( (offset >> 8) & 255 ))
		lo=$(( offset & 255 ))
		toks="$(i2ctransfer -y -f "$bus" w2@"$addr" "$(printf '0x%02x' "$hi")" "$(printf '0x%02x' "$lo")" r${chunk}@"$addr")"
		outstr=""
		for t in $toks; do
			h=${t#0x}
			outstr="$outstr\\x$h"
		done
		# shellcheck disable=SC2059
		printf "$outstr" >> "$out"
		offset=$(( offset + chunk ))
	done
}

read_nodriver() {
	# Inputs: virt, bits, out
	virt="$1"; bits="$2"; out="$3"
	if [ "$bits" -eq 8 ]; then
		sh "$SCRIPT_DIR/write_fru_no_driver.sh" --bus "$LINK_BUS" --address "$CLIENT_ADDR" --virt-address "$virt" --bits 8 --out "$out" >/dev/null
	else
		sh "$SCRIPT_DIR/write_fru_no_driver.sh" --bus "$LINK_BUS" --address "$CLIENT_ADDR" --virt-address "$virt" --bits 16 --out "$out" >/dev/null
	fi
}

# Write helpers
write_driver_8() {
	bus="$1"; addr="$2"; bin="$3"
	# default PAGE_SIZE is 16; force 8 to match device
	"$SCRIPT_DIR/write_8bit_fru_with_driver.sh" "$bus" "$addr" "$bin" 8
}

write_driver_16() {
	bus="$1"; addr="$2"; bin="$3"
	# default PAGE_SIZE 32 is OK; pass explicitly
	"$SCRIPT_DIR/write_16bit_fru_with_driver.sh" "$bus" "$addr" "$bin" 32
}

write_nodriver() {
	# Inputs: virt, bits, bin
	virt="$1"; bits="$2"; bin="$3"
	if [ "$bits" -eq 8 ]; then
		sh "$SCRIPT_DIR/write_fru_no_driver.sh" --bus "$LINK_BUS" --address "$CLIENT_ADDR" --virt-address "$virt" --bits 8  --input-bin-file "$bin" --write-delay-ms 10 >/dev/null
	else
		sh "$SCRIPT_DIR/write_fru_no_driver.sh" --bus "$LINK_BUS" --address "$CLIENT_ADDR" --virt-address "$virt" --bits 16 --input-bin-file "$bin" --write-delay-ms 10 >/dev/null
	fi
}

# Compare files; returns 0 if equal
files_equal() {
	cmp -s "$1" "$2"
}

MODE=""
LINK_BUS=""
CLIENT_ADDR=""
EXPECTED_DIR=""
CYCLES=10

while [ $# -gt 0 ]; do
	case "$1" in
		--mode) MODE="$2"; shift 2;;
		--link-bus) LINK_BUS="$(hex_to_int "$2")"; shift 2;;
		--client-addr) CLIENT_ADDR="$(printf '0x%02x' "$(hex_to_int "$2")")"; shift 2;;
		--expected-dir) EXPECTED_DIR="$2"; shift 2;;
		--cycles) CYCLES="$2"; shift 2;;
		-h|--help) usage; exit 0;;
		*) echo "Unknown arg: $1"; usage; exit 1;;
	esac
done

[ -n "$MODE" ] || { echo "Error: --mode is required"; usage; exit 1; }
[ -n "$EXPECTED_DIR" ] || { echo "Error: --expected-dir is required"; usage; exit 1; }
[ "$MODE" = "driver" ] || [ "$MODE" = "no-driver" ] || { echo "Error: --mode must be driver or no-driver"; exit 1; }
require_cmd i2ctransfer
require_cmd cmp
[ "$MODE" = "driver" ] || { [ -n "$LINK_BUS" ] && [ -n "$CLIENT_ADDR" ] || { echo "Error: --link-bus and --client-addr required for no-driver"; exit 1; }; }

ensure_ff_files

echo "Validation start: mode=$MODE cycles=$CYCLES expected_dir=$EXPECTED_DIR"

# Iterate inventory
fru_inventory | while read -r VIRT_BITS BUS RAWADDR || [ -n "$VIRT_BITS" ]; do
	# lines in form: virt bits bus addr
	# Shell read collapses multiple spaces, so parse carefully:
	set -- $VIRT_BITS $BUS $RAWADDR
	virt="$1"; bits="$2"; d_bus="$3"; d_addr="$4"
	# Expected file
	if [ "$bits" -eq 8 ]; then
		exp="${EXPECTED_DIR}/fru_$(printf '%02x' "$(hex_to_int "$virt")")_8.bin"
		size_expect=256
		ff_img="${EXPECTED_DIR}/ff256.bin"
	else
		exp="${EXPECTED_DIR}/fru_$(printf '%02x' "$(hex_to_int "$virt")")_16.bin"
		size_expect=8192
		ff_img="${EXPECTED_DIR}/ff8192.bin"
	fi
	# The above 'end' is a typo; fix with a valid if/fi structure
done >/dev/null 2>&1 || true

# Actual processing loop (reparse with a normal while)
fru_inventory | while read -r VIRT BITS DBUS DADDR; do
	[ -n "$VIRT" ] || continue
	# Resolve driver virtual bus dynamically, ignore DBUS from inventory
	VBUS="$(find_virtual_bus)"
	# In driver mode, target FRU address equals the VIRT address on the virtual bus
	if [ "$MODE" = "driver" ]; then
		TARGET_ADDR="$VIRT"
	else
		TARGET_ADDR="$DADDR"
	fi
	bits="$(hex_to_int "$BITS")"
	if [ "$bits" -eq 8 ]; then
		exp="${EXPECTED_DIR}/fru_$(printf '%02x' "$(hex_to_int "$VIRT")")_8.bin"
		size_expect=256
		ff_img="${EXPECTED_DIR}/ff256.bin"
	else
		exp="${EXPECTED_DIR}/fru_$(printf '%02x' "$(hex_to_int "$VIRT")")_16.bin"
		size_expect=8192
		ff_img="${EXPECTED_DIR}/ff8192.bin"
	fi
	[ -r "$exp" ] || { echo "Error: expected image not found: $exp"; exit 1; }
	act_size=$(stat -c '%s' "$exp" 2>/dev/null || echo 0)
	[ "$act_size" -eq "$size_expect" ] || { echo "Error: wrong size for $exp (got $act_size, want $size_expect)"; exit 1; }

	echo "FRU virt=$VIRT bits=$bits start cycles=$CYCLES"
	cycle=1
	while [ "$cycle" -le "$CYCLES" ]; do
		echo "  Cycle $cycle/$CYCLES: erase..."
		if [ "$MODE" = "driver" ]; then
			if [ "$bits" -eq 8 ]; then
				write_driver_8 "$VBUS" "$TARGET_ADDR" "$ff_img"
				tmp_out="$(mktemp)"
				read_driver_8_bin "$VBUS" "$TARGET_ADDR" "$tmp_out"
			else
				write_driver_16 "$VBUS" "$TARGET_ADDR" "$ff_img"
				tmp_out="$(mktemp)"
				read_driver_16_bin "$VBUS" "$TARGET_ADDR" "$tmp_out"
			fi
		else
			write_nodriver "$VIRT" "$bits" "$ff_img"
			tmp_out="$(mktemp)"
			read_nodriver "$VIRT" "$bits" "$tmp_out"
		fi
		if files_equal "$ff_img" "$tmp_out"; then
			echo "    Erase verify: OK"
		else
			echo "    Erase verify: FAIL"; rm -f "$tmp_out"; exit 1
		fi
		rm -f "$tmp_out"

		echo "  Cycle $cycle/$CYCLES: program..."
		if [ "$MODE" = "driver" ]; then
			if [ "$bits" -eq 8 ]; then
				write_driver_8 "$VBUS" "$TARGET_ADDR" "$exp"
				tmp_out="$(mktemp)"
				read_driver_8_bin "$VBUS" "$TARGET_ADDR" "$tmp_out"
			else
				write_driver_16 "$VBUS" "$TARGET_ADDR" "$exp"
				tmp_out="$(mktemp)"
				read_driver_16_bin "$VBUS" "$TARGET_ADDR" "$tmp_out"
			fi
		else
			write_nodriver "$VIRT" "$bits" "$exp"
			tmp_out="$(mktemp)"
			read_nodriver "$VIRT" "$bits" "$tmp_out"
		fi
		if files_equal "$exp" "$tmp_out"; then
			echo "    Program verify: OK"
		else
			echo "    Program verify: FAIL"; rm -f "$tmp_out"; exit 1
		fi
		rm -f "$tmp_out"

		cycle=$(( cycle + 1 ))
	done
	echo "FRU virt=$VIRT bits=$bits: COMPLETED ($CYCLES cycles)"
done

echo "All FRUs validated."


