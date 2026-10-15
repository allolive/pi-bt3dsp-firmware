#!/usr/bin/env python3
"""Can this Bluetooth controller be a 3D display for Bluetooth shutter glasses?

The Bluetooth 3D Synchronization Profile broadcasts the glasses' timing with
Connectionless Slave Broadcast (CSB) and a Synchronization Train, both optional
Bluetooth 4.1 controller features. Asks the controller (read-only HCI queries,
nothing is changed) and prints a verdict. Uses the add-on's own HCI socket
(resources/lib/shutter3d/bluetooth.py), so it runs where the add-on is.

Run as root on the box (or any Linux machine with the adapter):
  sudo python3 check_csb.py [hci0]

The bit positions are the Core spec's; see NOTES.md.
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shutter3d import bluetooth  # noqa: E402

OP_READ_LOCAL_VERSION = bluetooth.op(0x04, 0x001)
OP_READ_SUPPORTED_COMMANDS = bluetooth.op(0x04, 0x002)

COMMAND_BITS = {
    "Read_Clock": (15, 7),
    "Set_Triggered_Clock_Capture": (30, 5),
    "Set_Connectionless_Slave_Broadcast": (31, 0),
    "Start_Synchronization_Train": (31, 2),
    "Set_Reserved_LT_ADDR": (31, 4),
    "Set_Connectionless_Slave_Broadcast_Data": (31, 6),
    "Write_Synchronization_Train_Parameters": (32, 0),
}
NEEDED = ("Set_Connectionless_Slave_Broadcast", "Start_Synchronization_Train",
          "Set_Reserved_LT_ADDR", "Set_Connectionless_Slave_Broadcast_Data",
          "Write_Synchronization_Train_Parameters")
FEATURE_BITS = {
    "CSB master operation": 0,
    "CSB slave operation": 1,
    "Synchronization train": 2,
    "Synchronization scan": 3,
}


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "hci0"
    hci = bluetooth.Hci(int(dev.replace("hci", "")))

    ver = hci.command(OP_READ_LOCAL_VERSION)
    hci_ver, _, _, manuf, lmp_sub = struct.unpack("<BHBHH", ver[:8])
    print("%s: HCI version %d, manufacturer 0x%04x, LMP subversion 0x%04x"
          % (dev, hci_ver, manuf, lmp_sub))

    cmds = hci.command(OP_READ_SUPPORTED_COMMANDS)
    print("\nsupported commands:")
    have = {}
    for name, (octet, bit) in COMMAND_BITS.items():
        have[name] = bool(cmds[octet] >> bit & 1)
        print("  %-42s %s" % (name, "yes" if have[name] else "no"))

    print("\nLMP features, page 2:")
    bits = 0
    try:
        feat = hci.command(bluetooth.OP_READ_LOCAL_EXT_FEATURES, b"\x02")
        if feat[0] == 2:
            bits = int.from_bytes(feat[2:10], "little")
        else:
            print("  controller has no feature page 2 (max page %d)" % feat[1])
    except bluetooth.BT_ERRORS as e:
        print("  not readable: %s" % e)
    for name, bit in FEATURE_BITS.items():
        print("  %-42s %s" % (name, "yes" if bits >> bit & 1 else "no"))

    ok = all(have[n] for n in NEEDED) and hci.can_broadcast_3d()
    print("\nverdict: %s" % (
        "this controller can broadcast 3D sync for Bluetooth glasses" if ok else
        "no Connectionless Slave Broadcast here: it cannot drive Bluetooth 3D glasses as is"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
