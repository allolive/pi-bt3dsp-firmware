#!/bin/sh
# Keep only what the Zero W uses: its Bluetooth patch, as a real file under the
# name btbcm asks for first (in the package that name is a symlink).
set -e
FW="${TARGET_DIR}/lib/firmware"
NAME='BCM43430A1.raspberrypi,model-zero-w.hcd'
cp -L "${FW}/brcm/${NAME}" "${BUILD_DIR}/${NAME}"
rm -rf "${FW}"
mkdir -p "${FW}/brcm"
mv "${BUILD_DIR}/${NAME}" "${FW}/brcm/${NAME}"
rm -rf "${TARGET_DIR}/usr/share" "${TARGET_DIR}/etc/init.d"
