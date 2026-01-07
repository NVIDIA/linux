#!/bin/sh
set -eu

# virtual-mux-fru-ash.sh
# BusyBox ash compatible userspace host for virtual-mux client (RI v3, CRC on)
# Inputs:
#   --bus N            (link bus)
#   --address 0xYY     (client address on link bus)
#   --virt-address 0xVV (virtual FRU address, e.g. 0x50)
#   --bits 8|16        (FRU offset width)
# Behavior:
#   - If stdin has data → WRITE from offset 0 (page=16 for 8-bit, 64 for 16-bit)
#   - If stdin is tty   → READ from offset 0 (256 bytes for 8-bit, 8192 for 16-bit)

usage() {
	echo "Usage:"
	echo "  $0 --bus <N> --address <0xYY> --virt-address <0xVV> --bits <8|16> [--out <file>] [--debug] [--no-crc]"
	echo
	echo "Inputs:"
	echo "  --bus            Link I2C bus number (where virtual-mux-client is attached)"
	echo "  --address        Client 7-bit address on link bus (e.g., 0x2c)"
	echo "  --virt-address   Virtual FRU address (e.g., 0x50)"
	echo "  --bits           FRU offset width: 8 or 16"
	echo "  --input-bin-file Binary file to WRITE (e.g., from dd)"
	echo "  --page-size      EEPROM page size in bytes (default: 16 for 8-bit, 32 for 16-bit)"
	echo "  --write-delay-ms Inter-page write delay in ms (default: 10)"
	echo
	echo "Behavior:"
	echo "  - Read (no stdin): offset=0; length=256 (8-bit) or 8192 (16-bit); outputs raw to stdout (or --out file)"
	echo "  - Write (--input-bin-file): programs from offset=0; page=16 (8-bit) or 64 (16-bit); 10ms delay"
	echo "  - CRC16 framing is always enabled"
	echo "    (use --no-crc for debugging if needed)"
	echo
	echo "Examples:"
	echo "  Read (8-bit):  sh $0 --bus 1 --address 0x2c --virt-address 0x50 --bits 8  --out fru.bin"
	echo "  Read (16-bit): sh $0 --bus 1 --address 0x2c --virt-address 0x50 --bits 16 --out fru.bin"
	echo "  Write (8-bit):  sh $0 --bus 1 --address 0x2c --virt-address 0x50 --bits 8  --input-bin-file fru.bin"
	echo "  Write (16-bit): sh $0 --bus 1 --address 0x2c --virt-address 0x50 --bits 16 --input-bin-file fru16.bin"
}

require_cmd() {
	command -v "$1" >/dev/null 2>&1 || { echo "Error: '$1' not found"; exit 1; }
}

hex_to_int() {
	# Accepts decimal or 0xNN
	case "$1" in
		0x*|0X*) printf "%d" "$(( $1 ))" ;;
		*) printf "%d" "$1" ;;
	esac
}

int_to_hex_byte() {
	printf "0x%02x" "$(( $1 & 255 ))"
}

sleep_ms() {
	usleep $(( $1 * 1000 ))
}

# Lightweight timestamp for debug logs (seconds since boot with fraction)
ts_now() {
	IFS=' ' read -r ts _ < /proc/uptime
	printf '%s' "$ts"
}

# CRC16-IBM (poly 0x8005 reflected, init 0), kernel table as in lib/crc16.c
# Recurrence (kernel): crc = (crc >> 8) ^ table[(crc ^ byte) & 0xff]
CRC16_TABLE_HEX="\
0x0000 0xC0C1 0xC181 0x0140 0xC301 0x03C0 0x0280 0xC241 \
0xC601 0x06C0 0x0780 0xC741 0x0500 0xC5C1 0xC481 0x0440 \
0xCC01 0x0CC0 0x0D80 0xCD41 0x0F00 0xCFC1 0xCE81 0x0E40 \
0x0A00 0xCAC1 0xCB81 0x0B40 0xC901 0x09C0 0x0880 0xC841 \
0xD801 0x18C0 0x1980 0xD941 0x1B00 0xDBC1 0xDA81 0x1A40 \
0x1E00 0xDEC1 0xDF81 0x1F40 0xDD01 0x1DC0 0x1C80 0xDC41 \
0x1400 0xD4C1 0xD581 0x1540 0xD701 0x17C0 0x1680 0xD641 \
0xD201 0x12C0 0x1380 0xD341 0x1100 0xD1C1 0xD081 0x1040 \
0xF001 0x30C0 0x3180 0xF141 0x3300 0xF3C1 0xF281 0x3240 \
0x3600 0xF6C1 0xF781 0x3740 0xF501 0x35C0 0x3480 0xF441 \
0x3C00 0xFCC1 0xFD81 0x3D40 0xFF01 0x3FC0 0x3E80 0xFE41 \
0xFA01 0x3AC0 0x3B80 0xFB41 0x3900 0xF9C1 0xF881 0x3840 \
0x2800 0xE8C1 0xE981 0x2940 0xEB01 0x2BC0 0x2A80 0xEA41 \
0xEE01 0x2EC0 0x2F80 0xEF41 0x2D00 0xEDC1 0xEC81 0x2C40 \
0xE401 0x24C0 0x2580 0xE541 0x2700 0xE7C1 0xE681 0x2640 \
0x2200 0xE2C1 0xE381 0x2340 0xE101 0x21C0 0x2080 0xE041 \
0xA001 0x60C0 0x6180 0xA141  0x6300 0xA3C1 0xA281 0x6240 \
0x6600 0xA6C1 0xA781 0x6740 0xA501 0x65C0 0x6480 0xA441 \
0x6C00 0xACC1 0xAD81 0x6D40 0xAF01 0x6FC0 0x6E80 0xAE41 \
0xAA01 0x6AC0 0x6B80 0xAB41 0x6900 0xA9C1 0xA881 0x6840 \
0x7800 0xB8C1 0xB981 0x7940 0xBB01 0x7BC0 0x7A80 0xBA41 \
0xBE01 0x7EC0 0x7F80 0xBF41 0x7D00 0xBDC1 0xBC81 0x7C40 \
0xB401 0x74C0 0x7580 0xB541 0x7700 0xB7C1 0xB681 0x7640 \
0x7200 0xB2C1 0xB381 0x7340 0xB101  0x71C0 0x7080 0xB041 \
0x5000 0x90C1 0x9181 0x5140 0x9301 0x53C0 0x5280 0x9241 \
0x9601 0x56C0 0x5780 0x9741 0x5500 0x95C1 0x9481 0x5440 \
0x9C01 0x5CC0 0x5D80 0x9D41 0x5F00 0x9FC1 0x9E81 0x5E40 \
0x5A00 0x9AC1 0x9B81 0x5B40 0x9901 0x59C0 0x5880 0x9841 \
0x8801 0x48C0 0x4980 0x8941 0x4B00 0x8BC1 0x8A81 0x4A40 \
0x4E00 0x8EC1 0x8F81 0x4F40 0x8D01 0x4DC0 0x4C80 0x8C41 \
0x4400 0x84C1 0x8581 0x4540 0x8701 0x47C0 0x4680 0x8641 \
0x8201 0x42C0 0x4380 0x8341 0x4100 0x81C1 0x8081 0x4040"

# Initialize fast CRC16 table variables T0..T255 for O(1) lookup
init_crc16_table() {
	set -- $CRC16_TABLE_HEX
	i=0
	while [ $i -lt 256 ]; do
		eval "T$i=\"\$1\""
		shift
		i=$(( i + 1 ))
	done
}

crc16_tbl_lookup() {
	idx=$(( $1 & 255 ))
	# shellcheck disable=SC2016
	eval "echo \$T$idx"
}
# Kernel-style crc16_byte(crc, data) exactly as in lib/crc16.c
crc16_byte() {
	crc_in="$1"
	data_in="$2"
	# arithmetic expansion handles 0xNN tokens directly
	data_val=$(( data_in ))
	idx=$(( (crc_in ^ data_val) & 0xFF ))
	t_hex="$(crc16_tbl_lookup "$idx")"
	# arithmetic expansion handles 0xNN table entries
	t_dec=$(( t_hex ))
	echo $(( ((crc_in >> 8) ^ t_dec) & 0xFFFF ))
}

# Kernel-style crc16(crc, bytes...) exactly as in lib/crc16.c
crc16() {
	crc="$1"; shift
	pos=0
	for b in "$@"; do
		crc_prev="$crc"
		crc="$(crc16_byte "$crc" "$b")"
		# CRC debug disabled
		pos=$(( pos + 1 ))
	done
	printf "%d" "$crc"
}

# Build RI v3 SubmitXfer request as space-separated "0xNN" tokens in REQ_BYTES
build_submit_frame() {
	# Uses globals: SEQ, NMSGS, MSG_DESC_AND_DATA, USE_CRC
	# Produces: REQ_BYTES
	HDR_A="0x52 0x49 0x03 0x01 $(int_to_hex_byte "$SEQ") 0x0c"
	FLAGS="0x01 0x00" # CRC on
	RESV="0x00 0x00"
	# Payload header (nmsgs, retry=0, timeout=0, cookie=0)
	PAYLOAD_HDR="$(int_to_hex_byte "$NMSGS") 0x00 0x00 0x00 0x00 0x00 0x00 0x00"
	# Pre-total, pre-crc body:
	BODY="$PAYLOAD_HDR $MSG_DESC_AND_DATA"
	# Compute total = header(12) + body bytes count + crc(2)
	# Count tokens in BODY
	set -- $BODY
	body_count=$#
	total=$(( 12 + body_count + 2 ))
	total_lo=$(( total & 255 ))
	total_hi=$(( (total >> 8) & 255 ))
	TOTALTOK="$(int_to_hex_byte "$total_lo") $(int_to_hex_byte "$total_hi")"
	REQ_NOCRC="$HDR_A $TOTALTOK $FLAGS $RESV $BODY"
	# Compute CRC over REQ_NOCRC
	set -- $REQ_NOCRC
	crc_val=$(crc16 0 "$@")
	crc_lo=$(printf '0x%02x' $(( crc_val & 0xFF )))
	crc_hi=$(printf '0x%02x' $(( (crc_val >> 8) & 0xFF )))
	# CRC summary debug disabled
	REQ_BYTES="$REQ_NOCRC $crc_lo $crc_hi"
}

drain_bytes_if_any() {
	bus="$1"; addr="$2"; remain="$3"
	[ "$remain" -le 0 ] && return 0
	left="$remain"
	while [ "$left" -gt 0 ]; do
		chunk="$left"
		[ "$chunk" -gt 32 ] && chunk=32
		i2ctransfer -y -f "$bus" r${chunk}@"$addr" >/dev/null || true
		left=$(( left - chunk ))
	done
}

submit_and_fetch_response() {
	# Inputs: REQ_BYTES, LINK_BUS, CLIENT_ADDR, SEQ
	# Outputs: RESP_PAYLOAD (tokens), RESP_TOTAL
	bus="$LINK_BUS"; addr="$CLIENT_ADDR"
	# send
	set -- $REQ_BYTES
	wlen=$#
	if [ "${DEBUG:-0}" -eq 1 ]; then
		printf 'vmux [%s]: submit len=%s addr=%s\n' "$(ts_now)" "$wlen" "$addr"
		printf 'vmux [%s]: submit bytes: %s\n' "$(ts_now)" "$REQ_BYTES"
	fi
	i2ctransfer -y -f "$bus" w${wlen}@"$addr" "$@" >/dev/null
	# give client a moment to process
	sleep_ms 6
	# poll
	tries=0
	while [ "$tries" -lt 50 ]; do
		hdr="$(i2ctransfer -y -f "$bus" r12@"$addr" 2>/dev/null || true)"
		set -- $hdr
		[ "${DEBUG:-0}" -eq 1 ] && printf 'vmux [%s]: resp hdr try=%s: %s\n' "$(ts_now)" "$tries" "$hdr"
		if [ $# -ne 12 ]; then
			echo "short header" >&2
			exit 1
		fi
		magic0="$1"; magic1="$2"; version="$3"; msg_type="$4"; rseq="$5"; hlen="$6"; total_lo="$7"; total_hi="$8"; flags_lo="$9"; flags_hi="${10}"
		# validate magic/version
		if [ "$magic0" != "0x52" ] || [ "$magic1" != "0x4f" ]; then
			echo "bad magic" >&2
			exit 1
		fi
		[ "$version" = "0x03" ] || { echo "bad version" >&2; exit 1; }
		total=$(( $(hex_to_int "$total_lo") | ($(hex_to_int "$total_hi") << 8) ))
		# NOT_READY
		if [ "$msg_type" = "0x82" ]; then
			sleep_ms 6
			tries=$(( tries + 1 ))
			continue
		fi
		# wrong type/seq, drain and retry
		want_seq="$(int_to_hex_byte "$SEQ")"
		if [ "$msg_type" != "0x81" ] || [ "$rseq" != "$want_seq" ]; then
			drain_bytes_if_any "$bus" "$addr" $(( total - 12 ))
			sleep_ms 4
			tries=$(( tries + 1 ))
			continue
		fi
		remain=$(( total - 12 ))
		if [ "$remain" -gt 0 ]; then
			pl="$(i2ctransfer -y -f "$bus" r${remain}@"$addr")"
			RESP_PAYLOAD="$pl"
		else
			RESP_PAYLOAD=""
		fi
		RESP_TOTAL="$total"
		return 0
	done
	echo "timeout waiting for response" >&2
	exit 1
}

build_read_msgs() {
	# Inputs: VIRT_ADDR, ADDR_WIDTH, OFFSET, LENGTH
	# Output: MSG_DESC_AND_DATA, NMSGS=2
	if [ "$ADDR_WIDTH" -eq 8 ]; then
		offtok="$(int_to_hex_byte "$OFFSET")"
		MSG0_LEN="0x01 0x00"
		MSG0_DATA="$offtok"
	else
		off_hi=$(( (OFFSET >> 8) & 255 ))
		off_lo=$(( OFFSET & 255 ))
		MSG0_LEN="0x02 0x00"
		MSG0_DATA="$(int_to_hex_byte "$off_hi") $(int_to_hex_byte "$off_lo")"
	fi
	len_lo=$(( LENGTH & 255 ))
	len_hi=$(( (LENGTH >> 8) & 255 ))
	MSG1_LEN="$(int_to_hex_byte "$len_lo") $(int_to_hex_byte "$len_hi")"
	MSG_DESC_AND_DATA="$(int_to_hex_byte "$VIRT_ADDR") 0x00 $MSG0_LEN $MSG0_DATA $(int_to_hex_byte "$VIRT_ADDR") 0x01 $MSG1_LEN"
	NMSGS=2
}

build_write_msg_for_chunk() {
	# Inputs: VIRT_ADDR, ADDR_WIDTH, CHUNK_OFFSET, DATA_BYTES (space tokens)
	# Output: MSG_DESC_AND_DATA, NMSGS=1
	if [ "$ADDR_WIDTH" -eq 8 ]; then
		offtok="$(int_to_hex_byte "$CHUNK_OFFSET")"
		offset_bytes="$offtok"
	else
		off_hi=$(( (CHUNK_OFFSET >> 8) & 255 ))
		off_lo=$(( CHUNK_OFFSET & 255 ))
		offset_bytes="$(int_to_hex_byte "$off_hi") $(int_to_hex_byte "$off_lo")"
	fi
	set -- $offset_bytes $DATA_BYTES
	total_len=$#
	len_lo=$(( total_len & 255 ))
	len_hi=$(( (total_len >> 8) & 255 ))
	MSG_DESC_AND_DATA="$(int_to_hex_byte "$VIRT_ADDR") 0x00 $(int_to_hex_byte "$len_lo") $(int_to_hex_byte "$len_hi") $offset_bytes $DATA_BYTES"
	NMSGS=1
}

parse_result_and_output_read() {
	# Inputs: RESP_PAYLOAD, LENGTH
	set -- $RESP_PAYLOAD
	[ $# -ge 6 ] || { echo "short response" >&2; return 1; }
	st_lo="$1"; st_hi="$2"; nread="$3"
	# skip reserved 1 + 2 + 2 = 5 bytes after nread
	shift 8 # st_lo st_hi nread reserved + two u16 reserved (total 8 bytes)
	status_u16=$(( $(hex_to_int "$st_lo") | ($(hex_to_int "$st_hi") << 8) ))
	# convert to signed 16
	if [ $status_u16 -ge 32768 ]; then status=$(( status_u16 - 65536 )); else status=$status_u16; fi
	[ $status -eq 0 ] || { echo "transfer failed: $status" >&2; return 1; }
	# First block len
	[ $# -ge 2 ] || { echo "missing read block" >&2; return 1; }
	bl_lo="$1"; bl_hi="$2"; shift 2
	blen=$(( $(hex_to_int "$bl_lo") | ($(hex_to_int "$bl_hi") << 8) ))
	if [ $blen -ne "$LENGTH" ]; then
		# continue anyway
		:
	fi
	# Output raw bytes
	out=""
	i=0
	for b in "$@"; do
		[ $i -ge $blen ] && break
		out="$out\\x$(printf "%02x" "$(hex_to_int "$b")")"
		i=$(( i + 1 ))
	done
	if [ -n "$OUT_FILE" ]; then
		# shellcheck disable=SC2059
		if [ "${APPEND_OUTPUT:-0}" -eq 1 ]; then
			printf "$out" >> "$OUT_FILE"
		else
			printf "$out" > "$OUT_FILE"
		fi
		[ "${DEBUG:-0}" -eq 1 ] && printf 'vmux [%s]: wrote %d bytes to %s\n' "$(ts_now)" "$blen" "$OUT_FILE"
	else
		# shellcheck disable=SC2059
		printf "$out"
	fi
	return 0
}

parse_result_status_only() {
	set -- $RESP_PAYLOAD
	[ $# -ge 2 ] || { echo "short status" >&2; return 1; }
	st_lo="$1"; st_hi="$2"
	status_u16=$(( $(hex_to_int "$st_lo") | ($(hex_to_int "$st_hi") << 8) ))
	if [ $status_u16 -ge 32768 ]; then status=$(( status_u16 - 65536 )); else status=$status_u16; fi
	[ $status -eq 0 ] || { echo "transfer failed: $status" >&2; return 1; }
	return 0
}

# Parse args
LINK_BUS=""
CLIENT_ADDR=""
VIRT_ADDR=""
ADDR_WIDTH=""
USE_CRC=1
DEBUG=0
OUT_FILE=""
APPEND_OUTPUT=0
INPUT_BIN_FILE=""
PAGE_SIZE_OVERRIDE=""
WRITE_DELAY_MS_OVERRIDE=""

while [ $# -gt 0 ]; do
	case "$1" in
		--bus) LINK_BUS="$2"; shift 2;;
		--address) CLIENT_ADDR="$2"; shift 2;;
		--virt-address) VIRT_ADDR="$2"; shift 2;;
		--bits) ADDR_WIDTH="$2"; shift 2;;
		--input-bin-file) INPUT_BIN_FILE="$2"; shift 2;;
		--page-size) PAGE_SIZE_OVERRIDE="$2"; shift 2;;
		--write-delay-ms) WRITE_DELAY_MS_OVERRIDE="$2"; shift 2;;
		--out) OUT_FILE="$2"; shift 2;;
		--no-crc) USE_CRC=0; shift 1;;
		--debug) DEBUG=1; shift 1;;
		-h|--help)
			usage; exit 0;;
		*) echo "Unknown arg: $1"; usage; exit 1;;
	esac
done

require_cmd i2ctransfer
require_cmd usleep

[ -n "$LINK_BUS" ] && [ -n "$CLIENT_ADDR" ] && [ -n "$VIRT_ADDR" ] && [ -n "$ADDR_WIDTH" ] || { usage; exit 1; }
LINK_BUS="$(hex_to_int "$LINK_BUS")"
CLIENT_ADDR="$(printf "0x%02x" "$(hex_to_int "$CLIENT_ADDR")")"
VIRT_ADDR="$(hex_to_int "$VIRT_ADDR")"
ADDR_WIDTH="$(hex_to_int "$ADDR_WIDTH")"
[ "$ADDR_WIDTH" -eq 8 ] || [ "$ADDR_WIDTH" -eq 16 ] || { echo "--bits must be 8 or 16"; exit 1; }

# Initialize CRC table once
init_crc16_table

# Sequence (single-shot)
SEQ=$(( (RANDOM % 250) + 1 ))

# WRITE only when --input-bin-file is provided
if [ -n "$INPUT_BIN_FILE" ]; then
	[ -r "$INPUT_BIN_FILE" ] || { echo "Error: --input-bin-file not found or unreadable: $INPUT_BIN_FILE" >&2; exit 1; }
	# Page sizing and delay
	if [ -n "$PAGE_SIZE_OVERRIDE" ]; then
		PAGE_SIZE="$(hex_to_int "$PAGE_SIZE_OVERRIDE")"
	else
		if [ "$ADDR_WIDTH" -eq 8 ]; then PAGE_SIZE=8; else PAGE_SIZE=32; fi
	fi
	if [ -n "$WRITE_DELAY_MS_OVERRIDE" ]; then
		WRITE_DELAY_MS="$(hex_to_int "$WRITE_DELAY_MS_OVERRIDE")"
	else
		WRITE_DELAY_MS=10
	fi
	# Require dd for streaming
	command -v dd >/dev/null 2>&1 || { echo "Error: 'dd' not found"; exit 1; }
	FILE_SIZE=$(wc -c < "$INPUT_BIN_FILE" 2>/dev/null)
	[ -n "$FILE_SIZE" ] || FILE_SIZE=0
	printf 'vmux: write %s-bit FRU: page-size=%d bytes, delay=%d ms, total=%d bytes\n' "$ADDR_WIDTH" "$PAGE_SIZE" "$WRITE_DELAY_MS" "$FILE_SIZE"
	local_off=0
	# Progress (stderr) when DEBUG is off
	LAST_PCT=-1
	if [ "${DEBUG:-0}" -ne 1 ] && [ "$FILE_SIZE" -gt 0 ]; then
		printf 'Progress: %3d%%' 0 >&2
	fi
	while [ "$local_off" -lt "$FILE_SIZE" ]; do
		bytes_left=$(( FILE_SIZE - local_off ))
		page_off=$(( local_off % PAGE_SIZE ))
		head=$(( PAGE_SIZE - page_off ))
		[ $head -le 0 ] && head=$PAGE_SIZE
		chunk=$bytes_left
		[ $chunk -gt $head ] && chunk=$head
		# Stream this chunk and convert to hex tokens (BusyBox-compatible dd and hexdump)
		PAGE_HEX="$(dd if="$INPUT_BIN_FILE" bs=1 skip=$local_off count=$chunk 2>/dev/null | hexdump -v -e '1/1 "0x%02x "')" || PAGE_HEX=""
		# Trim trailing space
		DATA_BYTES="$(printf "%s" "$PAGE_HEX" | sed -e 's/[[:space:]]\+$//')"
		# Build and send
		CHUNK_OFFSET=$local_off
		build_write_msg_for_chunk
		build_submit_frame
		submit_and_fetch_response
		parse_result_status_only || exit 1
		local_off=$(( local_off + chunk ))
		# Update progress (stderr) if DEBUG is off
		if [ "${DEBUG:-0}" -ne 1 ] && [ "$FILE_SIZE" -gt 0 ]; then
			pct=$(( (local_off * 100) / FILE_SIZE ))
			if [ $pct -gt $LAST_PCT ]; then
				LAST_PCT=$pct
				printf '\rProgress: %3d%%' "$pct" >&2
			fi
		fi
		[ "$local_off" -lt "$FILE_SIZE" ] && sleep_ms "$WRITE_DELAY_MS"
		SEQ=$(( (SEQ + 1) & 255 )); [ $SEQ -eq 0 ] && SEQ=1
	done
	# Finish progress line
	if [ "${DEBUG:-0}" -ne 1 ] && [ "$FILE_SIZE" -gt 0 ]; then
		printf '\rProgress: 100%%\n' >&2
	fi
	exit 0
fi

# READ path
if [ "$ADDR_WIDTH" -eq 8 ]; then
	LENGTH=256
	OFFSET=0
	build_read_msgs
	build_submit_frame
	submit_and_fetch_response
	parse_result_and_output_read || exit 1
	exit 0
else
	# 16-bit FRU: read in chunks to avoid oversized transfers
	TOTAL=8192
	CHUNK=256
	OFFSET=0
	# If writing to a file, truncate first; then append per chunk
	if [ -n "$OUT_FILE" ]; then : > "$OUT_FILE"; fi
	APPEND_OUTPUT=1
	while [ "$OFFSET" -lt "$TOTAL" ]; do
		remain=$(( TOTAL - OFFSET ))
		LENGTH=$CHUNK
		[ "$LENGTH" -gt "$remain" ] && LENGTH=$remain
		build_read_msgs
		build_submit_frame
		submit_and_fetch_response
		parse_result_and_output_read || exit 1
		OFFSET=$(( OFFSET + LENGTH ))
		# bump seq for each chunk
		SEQ=$(( (SEQ + 1) & 255 )); [ $SEQ -eq 0 ] && SEQ=1
		# small delay helps some EEPROMs
		sleep_ms 4
	done
	APPEND_OUTPUT=0
	exit 0
fi


