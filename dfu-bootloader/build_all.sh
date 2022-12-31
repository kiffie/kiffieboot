#!/bin/bash
#
# build all kb3k USB DFU bootloader variants
#

CONFIGS="mx2xx-40mhz mx2xx-48mhz mx2x4-48mhz mx2x4-72mhz mx470-96mhz"
OUT_DIR=../bin

if [ ! -d $OUT_DIR ]; then
	echo "$OUT_DIR does not exist or is not a directory."
	exit 1
fi

for config in $CONFIGS; do
	echo "Building $config"
	make BUILD_CONFIG=$config || exit
	cp kb3k-dfu-*{hex,bin,xxd} $OUT_DIR
	make BUILD_CONFIG=$config mrproper
	echo
done

chmod a-x $OUT_DIR/*



