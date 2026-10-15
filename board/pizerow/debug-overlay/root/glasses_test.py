#!/usr/bin/env python3
"""Glasses test on the Pi: the add-on's Bluetooth 3D display (shutter3d/bluetooth.py,
unchanged) in its free-running test mode, with no Kodi and no fa_sync.

  [REVIVE=0] python3 glasses_test.py [hz] [pair_seconds] [run_seconds]

hz: the display rate the glasses alternate at (96 = 48 Hz per eye). The first
pair_seconds the Pi is discoverable for pairing (hold the glasses' button);
announcements and state are printed as they come.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shutter3d import bluetooth  # noqa: E402


class _NoFaSync:
    """bluetooth.py waits on the box's fa_sync; here it reads "off" every 100 ms."""

    def __getattr__(self, name):
        return getattr(os, name)

    def open(self, path, flags):
        return os.open("/dev/null", flags) if path == bluetooth.FA_SYNC else os.open(path, flags)

    def pread(self, fd, n, off):
        time.sleep(0.1)
        return b"off"


bluetooth.os = _NoFaSync()

# REVIVE=0: no periodic "broadcast on / start train" every 5 s. This chip
# accepts both while they already run, so the check cannot see "already on";
# with it off the train is restarted only on the controller's own events.
if os.environ.get("REVIVE", "1") == "0":
    _revive = bluetooth.Display.revive
    bluetooth.Display.revive = lambda self, why: None if why == "periodic check" else _revive(self, why)


class Settings:
    bt_adapter = "auto"
    delay_us = 0
    invert = False
    open_after_us = 0
    open_us = 0


class Store:
    PATH = "/tmp/bt3d-store.json"

    def __init__(self):
        try:
            with open(self.PATH) as f:
                self.data = json.load(f)
        except (OSError, ValueError):
            self.data = {}

    def _save(self):
        with open(self.PATH, "w") as f:
            json.dump(self.data, f)

    def saved_adapter(self, addr):
        return self.data.get(addr, {})

    def saved_adapters(self):
        return dict(self.data)

    def save_adapter(self, addr, d):
        self.data[addr] = d
        self._save()

    def glasses_seen(self, addr, associated, battery_request, battery):
        log("GLASSES %s associated=%s battery_request=%s battery=%s%%"
            % (addr, associated, battery_request, battery))

    def glasses_name(self, addr, name):
        log("GLASSES %s name %r" % (addr, name))


T0 = time.monotonic()


def log(msg):
    print("%7.2f %s" % (time.monotonic() - T0, msg), flush=True)


def main():
    hz = float(sys.argv[1]) if len(sys.argv) > 1 else 96.0
    pair_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    run_s = float(sys.argv[3]) if len(sys.argv) > 3 else 600.0
    # Named, not "auto": the CYW43438 runs the sync train but does not set its
    # LMP feature bit, which "auto" requires.
    mgmt = bluetooth.Mgmt()
    try:
        Settings.bt_adapter = mgmt.info(mgmt.indexes()[0])[0]
    finally:
        mgmt.close()
    link = bluetooth.BluetoothLink(Settings(), log, Store(), on_problem=lambda r: log("PROBLEM " + r))
    link.pairing = pair_s > 0
    link.test(hz)
    link.start()
    log("adapter %s" % Settings.bt_adapter)
    log("test at %.3f Hz (%.3f Hz per eye), pairing for %.0f s" % (hz, hz / 2, pair_s))
    last = None
    try:
        while time.monotonic() - T0 < run_s:
            time.sleep(0.5)
            if link.pairing and time.monotonic() - T0 > pair_s:
                link.pair(False)
            st = link.state()
            st.pop("on_since", None)
            if st != last:
                log("state %s" % st)
                last = st
    except KeyboardInterrupt:
        pass
    link.stop()
    log("stopped")


if __name__ == "__main__":
    main()
