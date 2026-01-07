#!/usr/bin/env bash
set -euo pipefail

usage() {
	echo "Usage: $(basename "$0") BUS ADDR FILE [PAGE_SIZE]"
	echo "  BUS       : i2c bus number (e.g., 90)"
	echo "  ADDR      : 7-bit device address (e.g., 0x50)"
	echo "  FILE      : path to binary to write"
	echo "  PAGE_SIZE : optional, default 32 (bytes); typical 24c64 pages are 32"
}

if [[ $# -lt 3 || $# -gt 4 ]]; then
	usage
	exit 1
fi

BUS="$1"
ADDR="$2"
BIN="$3"
PAGE_SIZE="${4:-32}"

if ! command -v i2ctransfer >/dev/null 2>&1; then
	echo "Error: i2ctransfer not found in PATH" >&2
	exit 1
fi
if [[ ! -r "$BIN" ]]; then
	echo "Error: file '$BIN' not readable" >&2
	exit 1
fi
if ! [[ "$PAGE_SIZE" =~ ^[0-9]+$ ]]; then
	echo "Error: PAGE_SIZE must be a positive integer" >&2
	exit 1
fi

SIZE=$(stat -c '%s' "$BIN")
OFFSET=0

echo "Writing 16-bit addressed EEPROM @ bus=$BUS addr=$ADDR size=${SIZE}B page=${PAGE_SIZE}B"

while (( OFFSET < SIZE )); do
	REMAIN=$(( SIZE - OFFSET ))
	# Do not cross page boundary
	PAGE_OFF=$(( OFFSET % PAGE_SIZE ))
	CHUNK=$(( PAGE_SIZE - PAGE_OFF ))
	if (( CHUNK > REMAIN )); then CHUNK=$REMAIN; fi

	# Extract CHUNK bytes from BIN starting at OFFSET and render as 0xHH tokens
	BYTES_STR=$(dd if="$BIN" bs=1 skip="$OFFSET" count="$CHUNK" status=none | hexdump -v -e '1/1 "0x%02x "')
	# shellcheck disable=SC2206
	DATA_TOKENS=($BYTES_STR)

	# Two-byte internal address: high then low
	ADDR_HI=$(printf "0x%02x" $(( (OFFSET >> 8) & 0xff )))
	ADDR_LO=$(printf "0x%02x" $(( OFFSET & 0xff )))

	# Perform a single write: [ADDR_HI][ADDR_LO][DATA...]
	LEN=$(( 2 + CHUNK ))
	i2ctransfer -f -y "$BUS" w${LEN}@"$ADDR" "$ADDR_HI" "$ADDR_LO" "${DATA_TOKENS[@]}"

	# Write cycle time (tWR), conservative 10ms
	sleep 0.01

	OFFSET=$(( OFFSET + CHUNK ))
done

echo "Done."



