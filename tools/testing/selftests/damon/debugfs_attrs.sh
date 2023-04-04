#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

source _debugfs_common.sh

# Test attrs file
# ===============

file="$DBGFS/attrs"
orig_content=$(cat "$file")

test_write_succ "$file" "1 2 3 4 5" "$orig_content" "valid input"
test_write_fail "$file" "1 2 3 4" "$orig_content" "no enough fields"
test_write_fail "$file" "1 2 3 5 4" "$orig_content" \
	"min_nr_regions > max_nr_regions"
test_content "$file" "$orig_content" "1 2 3 4 5" "successfully written"
echo "$orig_content" > "$file"
<<<<<<< HEAD

# Test target_ids file
# ====================

file="$DBGFS/target_ids"
orig_content=$(cat "$file")

test_write_succ "$file" "1 2 3 4" "$orig_content" "valid input"
test_write_succ "$file" "1 2 abc 4" "$orig_content" "still valid input"
test_content "$file" "$orig_content" "1 2" "non-integer was there"
test_write_succ "$file" "abc 2 3" "$orig_content" "the file allows wrong input"
test_content "$file" "$orig_content" "" "wrong input written"
test_write_succ "$file" "" "$orig_content" "empty input"
test_content "$file" "$orig_content" "" "empty input written"
echo "$orig_content" > "$file"

# Test huge count read write
# ==========================

dmesg -C

for file in "$DBGFS/"*
do
	./huge_count_read_write "$file"
done

if dmesg | grep -q WARNING
then
	dmesg
	exit 1
else
	exit 0
fi

echo "PASS"
=======
>>>>>>> origin/linux_6.1.15_upstream
