#!/bin/sh
# Lay out what rpiboot serves (output*/images/boot/, for `rpiboot -d`): only
# the files the Pi asks for, and of the overlays only those config.txt names.
set -e
FW="${BINARIES_DIR}/rpi-firmware"
BOOT="${BINARIES_DIR}/boot"
rm -rf "${BOOT}"
mkdir -p "${BOOT}/overlays"
cp "${FW}/bootcode.bin" "${FW}/config.txt" "${FW}/cmdline.txt" "${BOOT}/"
for f in $(sed -n 's/^\(start\|fixup\)_file=//p' "${FW}/config.txt"); do
	cp "${FW}/${f}" "${BOOT}/"
done
cp "${BINARIES_DIR}/zImage" "${BINARIES_DIR}"/*.dtb "${BOOT}/"
# config.txt names it rootfs.img: gzip (debug) or xz (shipped), the kernel
# tells them apart itself
for z in xz gz; do
	if [ -e "${BINARIES_DIR}/rootfs.cpio.${z}" ]; then
		cp "${BINARIES_DIR}/rootfs.cpio.${z}" "${BOOT}/rootfs.img"
		break
	fi
done
for o in $(sed -n 's/^dtoverlay=\([^,]*\).*/\1/p' "${FW}/config.txt"); do
	cp "${FW}/overlays/${o}.dtbo" "${BOOT}/overlays/"
done
# which build this is: bt3dd reports the same in its status line
cp "${TARGET_DIR}/etc/bt3dd-build" "${BOOT}/build.txt"
du -sh "${BOOT}" | sed 's/^/boot files: /'
