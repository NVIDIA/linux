#!/bin/sh
set -eu

# Orchestrates full validation:
# - Optionally unbind/bind virtual-mux-host driver around validation runs
# - Runs no-driver and/or driver validation phases via validate_fru.sh
#
# Requirements:
#   - Root privileges (for bind/unbind)
#   - /sys/bus/i2c/drivers/virtual-mux-host present on target
#
# Usage examples:
#   sh vmux/run_full_validation.sh --phases both --link-bus 3 --client-addr 0x51 --expected-dir vmux/expected --cycles 10
#   sh vmux/run_full_validation.sh --phases driver --expected-dir vmux/expected

usage() {
	echo "Usage:"
	echo "  $0 --phases <both|driver|no-driver> --expected-dir <DIR> [--link-bus <N>] [--client-addr <0xYY>] [--cycles <N>]"
	echo
	echo "Notes:"
	echo "  - For phases including 'no-driver', you must provide --link-bus and --client-addr."
	echo "  - This script preserves the initial binding state of virtual-mux-host and restores it on exit."
}

hex_to_int() {
	case "$1" in
		0x*|0X*) printf "%d" "$(( $1 ))" ;;
		*) printf "%d" "$1" ;;
	esac
}

require_path() {
	[ -e "$1" ] || { echo "Error: path not found: $1"; exit 1; }
}

# Resolve script dir for relative calls
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

PHASES=""
LINK_BUS=""
CLIENT_ADDR=""
EXPECTED_DIR=""
CYCLES=10

while [ $# -gt 0 ]; do
	case "$1" in
		--phases) PHASES="$2"; shift 2;;
		--link-bus) LINK_BUS="$(hex_to_int "$2")"; shift 2;;
		--client-addr) CLIENT_ADDR="$(printf '0x%02x' "$(hex_to_int "$2")")"; shift 2;;
		--expected-dir) EXPECTED_DIR="$2"; shift 2;;
		--cycles) CYCLES="$2"; shift 2;;
		-h|--help) usage; exit 0;;
		*) echo "Unknown arg: $1"; usage; exit 1;;
	esac
done

[ -n "$PHASES" ] || { echo "Error: --phases is required"; usage; exit 1; }
[ -n "$EXPECTED_DIR" ] || { echo "Error: --expected-dir is required"; usage; exit 1; }
case "$PHASES" in
	both|driver|no-driver) : ;;
	*) echo "Error: --phases must be one of: both|driver|no-driver"; exit 1;;
esac

VMUX_HOST_DRV="/sys/bus/i2c/drivers/virtual-mux-host"
require_path "$VMUX_HOST_DRV"

# Defaults for environments where link bus and client are fixed
if [ -z "$LINK_BUS" ]; then LINK_BUS=3; fi
if [ -z "$CLIENT_ADDR" ]; then CLIENT_ADDR="0x51"; fi

# Device id in sysfs is "<bus>-<addr 4 hex>"
ADDR_DEC=$(hex_to_int "$CLIENT_ADDR")
DEVICE_ID="$(printf "%d-%04x" "$LINK_BUS" "$ADDR_DEC")"

DEV_NODE=""
is_bound() {
	# Consider bound if driver lists the device symlink
	[ -L "$VMUX_HOST_DRV/$DEVICE_ID" ]
}

bind_driver() {
	echo "$DEVICE_ID" > "$VMUX_HOST_DRV/bind"
	# Wait for binding to settle
	tries=0
	while [ $tries -lt 50 ]; do
		if is_bound; then return 0; fi
		usleep 100000
		tries=$(( tries + 1 ))
	done
	echo "Error: bind timed out for $DEVICE_ID" >&2
	exit 1
}

unbind_driver() {
	# If already not bound, return
	if ! is_bound; then return 0; fi
	# Attempt unbind, ignore immediate write errors to be robust
	{ echo "$DEVICE_ID" > "$VMUX_HOST_DRV/unbind"; } 2>/dev/null || true
	tries=0
	while [ $tries -lt 50 ]; do
		if ! is_bound; then return 0; fi
		usleep 100000
		tries=$(( tries + 1 ))
	done
	echo "Error: unbind timed out for $DEVICE_ID" >&2
	exit 1
}

INITIAL_BOUND=0
if [ -n "$DEVICE_ID" ]; then
	if is_bound; then INITIAL_BOUND=1; fi
fi

restore_binding() {
	if [ -z "$DEVICE_ID" ]; then return 0; fi
	if [ $INITIAL_BOUND -eq 1 ]; then
		is_bound || bind_driver
	else
		is_bound && unbind_driver
	fi
}
trap restore_binding EXIT INT TERM

echo "Orchestration start: phases=$PHASES expected_dir=$EXPECTED_DIR cycles=$CYCLES"

# Phase: driver (run first if 'both')
if [ "$PHASES" = "both" ] || [ "$PHASES" = "driver" ]; then
	if ! is_bound; then
		echo "Binding virtual-mux-host to $DEVICE_ID for driver validation..."
		bind_driver
		# Give the virtual bus some time to enumerate
		usleep 500000
	fi
	echo "Running driver validation..."
	sh "$SCRIPT_DIR/validate_fru.sh" --mode driver --expected-dir "$EXPECTED_DIR" --cycles "$CYCLES"
	echo "Driver validation completed."
fi

# Phase: no-driver (run after driver if 'both')
if [ "$PHASES" = "both" ] || [ "$PHASES" = "no-driver" ]; then
	if is_bound; then
		echo "Unbinding virtual-mux-host from $DEVICE_ID for no-driver validation..."
		unbind_driver
	fi
	echo "Running no-driver validation..."
	sh "$SCRIPT_DIR/validate_fru.sh" --mode no-driver --link-bus "$LINK_BUS" --client-addr "$CLIENT_ADDR" --expected-dir "$EXPECTED_DIR" --cycles "$CYCLES"
	echo "No-driver validation completed."
fi

echo "All phases done."


