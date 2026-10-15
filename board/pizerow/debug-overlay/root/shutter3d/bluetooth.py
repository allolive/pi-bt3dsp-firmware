"""Bluetooth shutter glasses: the box as a 3D display (3DD) of the Bluetooth 3D
Synchronization Profile, on any adapter whose controller has Connectionless
Slave Broadcast (CSB) and the Synchronization Train. No Kodi imports.

The glasses only listen: the display broadcasts, every 80 ms, when a left+right
pair starts on its own Bluetooth clock, the pair's period and when each lens
opens and closes; the glasses run on that until the next message. Glasses that
support it also connect for a moment to announce themselves (just paired, or
switched on) with their battery level; that is the only way the display learns
which glasses exist. See firmware/bt3dsp/NOTES.md for the profile.

The display is on only while it is needed: a 3D film, pairing, a glasses
test. In between the adapter is exactly as bluetoothd left it. It stays
shared: the kernel and bluetoothd keep running (the remote may use the same
dongle over LE). The class of device and the extended inquiry response are
rewritten for 3D and put back on stop; connectable and discoverable go through
the kernel's management interface, so the kernel accepts the glasses'
connections.
"""

import ctypes
import os
import select
import struct
import threading
import time

from . import core

FA_SYNC = "/sys/class/video/fa_sync"

AF_BLUETOOTH = 31
BTPROTO_HCI = 1
SOCK_RAW = 3
SOCK_CLOEXEC = 0o2000000
SOL_HCI = 0
HCI_FILTER = 2
HCI_CHANNEL_RAW = 0
HCI_CHANNEL_CONTROL = 3
HCI_DEV_NONE = 0xFFFF

HCI_COMMAND_PKT = 0x01
HCI_ACLDATA_PKT = 0x02
HCI_EVENT_PKT = 0x04

EVT_CONN_COMPLETE = 0x03
EVT_DISCONN_COMPLETE = 0x05
EVT_REMOTE_NAME = 0x07
EVT_CMD_COMPLETE = 0x0E
EVT_CMD_STATUS = 0x0F
EVT_SYNC_TRAIN_COMPLETE = 0x4F
EVT_SLAVE_PAGE_RESPONSE_TIMEOUT = 0x54
EVT_CSB_CHANNEL_MAP_CHANGE = 0x55
RESTART_SYNC_TRAIN = (EVT_SYNC_TRAIN_COMPLETE, EVT_SLAVE_PAGE_RESPONSE_TIMEOUT, EVT_CSB_CHANNEL_MAP_CHANGE)

LT_ADDR = 0x01
CSB_PACKET_TYPE = 0x330E         # DM1 only: legacy glasses drop anything else
CSB_INTERVAL = 0x0080            # 80 ms, what legacy glasses expect
CSB_TIMEOUT = 0x2000             # slots without fresh data; we refresh every 80 ms
SYNC_TRAIN_INTERVAL = 0x0080     # 80 ms: legacy glasses scan ~100 ms per channel
SYNC_TRAIN_TIMEOUT = 0x0002EE00  # 120 s, restarted whenever it ends
CLASS_3D_DISPLAY = bytes([0x3C, 0x04, 0x08])   # 0x08043C: A/V, display and loudspeaker
# all 79 channels usable (bit 79 reserved). The UB500 otherwise announces an
# empty channel map in its sync train, and receivers have nowhere to listen
ALL_CHANNELS = b"\xff" * 9 + b"\x7f"
# glasses pick a display whose path loss (its TX power less the RSSI) is
# under this; an antenna inside the box looks far, and a home has one display
PATH_LOSS_DB = 127
PSM_3DSP = 0x0021                # the 3D communications channel (connectionless L2CAP)
L2CAP_CID_CONNECTIONLESS = 0x0002
CLKN_US = 312.5                  # native clock tick
CLKN_WRAP = 1 << 28
UPDATE_S = 0.08                  # one message per CSB interval
KEEP_S = 5.0                     # re-assert class, EIR and connectable this often
RESEND_S = 4.0                   # an unchanged message is resent this often (CSB_TIMEOUT: 5.12 s)


def op(ogf, ocf):
    return (ogf << 10) | ocf


OP_WRITE_CLASS_OF_DEVICE = op(0x03, 0x024)
OP_READ_CLASS_OF_DEVICE = op(0x03, 0x023)
OP_SET_AFH_HOST_CLASSIFICATION = op(0x03, 0x03F)
# scan activity (interval, window) and type (standard/interlaced): glasses
# search in short bursts and miss a controller that scans 11 ms in 2.56 s
SCAN_SETTINGS = (  # (name, read opcode, write opcode, while displaying)
    ("inquiry_scan", op(0x03, 0x01D), op(0x03, 0x01E), struct.pack("<HH", 0x0400, 0x0200)),
    ("inquiry_scan_type", op(0x03, 0x042), op(0x03, 0x043), b"\x01"),
    ("page_scan", op(0x03, 0x01B), op(0x03, 0x01C), struct.pack("<HH", 0x0400, 0x0200)),
    ("page_scan_type", op(0x03, 0x046), op(0x03, 0x047), b"\x01"),
)
OP_READ_EIR = op(0x03, 0x051)
OP_WRITE_EIR = op(0x03, 0x052)
OP_READ_INQ_TX_POWER = op(0x03, 0x058)
OP_SET_RESERVED_LT_ADDR = op(0x03, 0x074)
OP_DELETE_RESERVED_LT_ADDR = op(0x03, 0x075)
OP_SET_CSB_DATA = op(0x03, 0x076)
OP_WRITE_SYNC_TRAIN_PARAMS = op(0x03, 0x078)
OP_SET_CSB = op(0x01, 0x041)
OP_START_SYNC_TRAIN = op(0x01, 0x043)
OP_REMOTE_NAME_REQUEST = op(0x01, 0x019)
OP_READ_LOCAL_EXT_FEATURES = op(0x04, 0x004)
OP_READ_CLOCK = op(0x05, 0x007)

MGMT_READ_INDEX_LIST = 0x0003
MGMT_READ_INFO = 0x0004
MGMT_SET_DISCOVERABLE = 0x0006
MGMT_SET_CONNECTABLE = 0x0007
MGMT_EV_CMD_COMPLETE = 0x0001
MGMT_EV_CMD_STATUS = 0x0002
MGMT_SETTING_POWERED = 1 << 0
MGMT_SETTING_CONNECTABLE = 1 << 1
MGMT_SETTING_DISCOVERABLE = 1 << 3
MGMT_SETTING_BREDR = 1 << 7


class HciError(RuntimeError):
    def __init__(self, opcode, status):
        super().__init__("0x%04x failed, status 0x%02x" % (opcode, status))
        self.status = status


# everything a command or socket can raise: HciError is a RuntimeError,
# TimeoutError an OSError
BT_ERRORS = (RuntimeError, OSError)


# ---- the broadcast message (3DSP Table 8.1) -------------------------------

def pack_message(fsi_us, period_us, offsets):
    """17 bytes: frame sync instant on the native clock (us), the L+R period
    (us, fractional), and (left open, left close, right open, right close) in
    us after the instant. period 0 (or None) asks for 2D: both lenses open."""
    fsi_us = int(round(fsi_us)) % int(CLKN_WRAP * CLKN_US)    # CLK[27:1] wraps with CLKN
    slots, phase = divmod(fsi_us, 625)
    word = slots & 0x07FFFFFF                    # bits 27-31: reserved, video mode 3D
    if not period_us or period_us > 40000:
        return struct.pack("<IH4HHB", word, phase, 0xFFFF, 0, 0, 0, 0, 0)
    whole = int(period_us)
    frac = int(round((period_us - whole) * 256))
    if frac == 256:
        whole, frac = whole + 1, 0
    lo, lc, ro, rc = (max(0, min(0xFFFE, int(round(v)))) for v in offsets)
    return struct.pack("<IH4HHB", word, phase, lo, lc, ro, rc, whole, frac)


MESSAGE_2D = pack_message(0, 0, (0, 0, 0, 0))     # both lenses open


def window_offsets(period_us, open_after_us, open_us=None):
    """Each lens opens open_after us after its eye's frame start and stays open
    open_us (by default until the other eye's frame starts)."""
    half = period_us / 2
    width = half - open_after_us if not open_us else min(open_us, half - open_after_us)
    width = max(width, 0)
    return (open_after_us, open_after_us + width,
            half + open_after_us, half + open_after_us + width)


def eir_data(tx_power):
    """3D Information (association notification, battery level reporting and a
    report at start-up: flags 0x07), the legacy Broadcom record (first
    manufacturer entry, "3D capable") and TX power, which legacy glasses read
    unsigned."""
    eir = bytes([0x03, 0x3D, 0x07, PATH_LOSS_DB])
    eir += bytes([0x06, 0xFF, 0x0F, 0x00, 0x00, 0x01, PATH_LOSS_DB])
    eir += bytes([0x02, 0x0A, max(0, min(127, tx_power))])
    return eir.ljust(240, b"\x00")


def format_addr(raw):
    """6 little-endian bytes -> AA:BB:CC:DD:EE:FF."""
    return ":".join("%02X" % b for b in reversed(raw))


def parse_addr(text):
    return bytes(reversed(bytes.fromhex(text.replace(":", ""))))


def parse_announcement(acl):
    """An ACL data packet (after the packet type byte) carrying a 3DG Connection
    Announcement: (handle, associated, battery request, battery % or None), or
    None for anything else."""
    if len(acl) < 4:
        return None
    hdr, length = struct.unpack("<HH", acl[:4])
    if (hdr >> 12) & 0x3 == 0x1:                 # a continuation fragment
        return None
    data = acl[4:4 + length]
    if len(data) < 9:
        return None
    l2len, cid, psm = struct.unpack("<HHH", data[:6])
    if cid != L2CAP_CID_CONNECTIONLESS or psm != PSM_3DSP or l2len < 5:
        return None
    opcode, flags, battery = data[6], data[7], data[8]
    if opcode != 0:
        return None
    return hdr & 0x0FFF, bool(flags & 1), bool(flags & 2), battery if battery <= 100 else None


# ---- the controller's clock against CLOCK_MONOTONIC -----------------------

class ClockMap:
    """CLOCK_MONOTONIC ns -> native Bluetooth clock us. Read_Clock is bracketed
    by two monotonic reads; the tightest brackets are the most trustworthy, and
    a line through the recent best ones gives offset and drift."""

    def __init__(self, keep=32):
        self.samples = []            # (mono_ns, clk_us_unwrapped, rtt_ns)
        self.keep = keep
        self.last_raw = None
        self.wraps = 0

    def add(self, before_ns, clkn, after_ns):
        if self.last_raw is not None and clkn < self.last_raw:
            self.wraps += 1
        self.last_raw = clkn
        clk_us = (clkn + self.wraps * CLKN_WRAP) * CLKN_US
        self.samples.append(((before_ns + after_ns) // 2, clk_us, after_ns - before_ns))
        self.samples = self.samples[-self.keep:]

    def ready(self):
        return len(self.samples) >= 4

    def to_clk_us(self, mono_ns):
        best = sorted(self.samples, key=lambda s: s[2])[:max(4, len(self.samples) // 2)]
        best.sort()
        if len(best) == 1 or best[-1][0] == best[0][0]:
            m0, c0, _ = best[0]
            return c0 + (mono_ns - m0) / 1000.0
        n = len(best)
        mx = sum(s[0] for s in best) / n
        cy = sum(s[1] for s in best) / n
        sxx = sum((s[0] - mx) ** 2 for s in best)
        sxy = sum((s[0] - mx) * (s[1] - cy) for s in best)
        slope = sxy / sxx if sxx else 1e-3          # us per ns
        return cy + slope * (mono_ns - mx)


# ---- the eye feed ----------------------------------------------------------

class VsyncTrack:
    """The latest left-eye vsync and the vsync period, from fa_sync samples."""

    def __init__(self):
        self.left_ns = None
        self.last = None             # (vsync, ns)
        self.period_ns = None

    def add(self, vsync, ns, eye):
        if self.last is not None:
            dv = (vsync - self.last[0]) & 0xFFFFFFFF
            if 0 < dv < 64:
                p = (ns - self.last[1]) / dv
                self.period_ns = p if self.period_ns is None else self.period_ns + (p - self.period_ns) / 32
            else:
                self.period_ns = None
        self.last = (vsync, ns)
        if eye == "L":
            self.left_ns = ns

    def ready(self):
        return self.left_ns is not None and self.period_ns is not None


# ---- sockets (ctypes: CoreELEC's Python has no AF_BLUETOOTH) ---------------

_libc = None


def _lib():
    global _libc
    if _libc is None:
        _libc = ctypes.CDLL(None, use_errno=True)
    return _libc


def _bt_socket(dev, channel):
    fd = _lib().socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI)
    if fd < 0:
        raise OSError(ctypes.get_errno(), "bluetooth socket")
    addr = struct.pack("<HHH", AF_BLUETOOTH, dev, channel)    # sockaddr_hci
    if _lib().bind(fd, addr, len(addr)) < 0:
        err = ctypes.get_errno()
        os.close(fd)
        raise OSError(err, "bind hci%s channel %d" % ("-" if dev == HCI_DEV_NONE else dev, channel))
    return fd


def _readable(fd, timeout):
    return bool(select.select([fd], [], [], max(0.0, timeout))[0])


class Mgmt:
    """The kernel's Bluetooth management interface (what bluetoothd uses)."""

    def __init__(self):
        self.fd = _bt_socket(HCI_DEV_NONE, HCI_CHANNEL_CONTROL)

    def close(self):
        os.close(self.fd)

    def command(self, opcode, index, params=b"", timeout=2.0):
        os.write(self.fd, struct.pack("<HHH", opcode, index, len(params)) + params)
        end = time.monotonic() + timeout
        while _readable(self.fd, end - time.monotonic()):
            pkt = os.read(self.fd, 1024)
            if len(pkt) < 9:
                continue
            event, _index, _length, ev_opcode, status = struct.unpack("<HHHHB", pkt[:9])
            if event not in (MGMT_EV_CMD_COMPLETE, MGMT_EV_CMD_STATUS) or ev_opcode != opcode:
                continue
            if status:
                raise RuntimeError("mgmt 0x%04x on hci%d: status 0x%02x" % (opcode, index, status))
            return pkt[9:]
        raise TimeoutError("mgmt 0x%04x: no answer" % opcode)

    def indexes(self):
        ret = self.command(MGMT_READ_INDEX_LIST, HCI_DEV_NONE)
        n = struct.unpack("<H", ret[:2])[0]
        return list(struct.unpack("<%dH" % n, ret[2:2 + 2 * n]))

    def info(self, index):
        """(address, current settings)"""
        return self.details(index)[:2]

    def details(self, index):
        """(address, current settings, name)"""
        ret = self.command(MGMT_READ_INFO, index)
        name = ret[20:269].split(b"\x00", 1)[0].decode("utf-8", "replace")
        return format_addr(ret[:6]), struct.unpack("<I", ret[13:17])[0], name

    def set_connectable(self, index, on):
        self.command(MGMT_SET_CONNECTABLE, index, bytes([1 if on else 0]))

    def set_discoverable(self, index, on):
        """No timeout: discoverable until switched off."""
        self.command(MGMT_SET_DISCOVERABLE, index, struct.pack("<BH", 1 if on else 0, 0))


class Hci:
    """A raw HCI socket beside the kernel's own use of the adapter: sees every
    event and the incoming ACL data, sends commands through the kernel's
    queue."""

    def __init__(self, dev):
        self.fd = _bt_socket(dev, HCI_CHANNEL_RAW)
        flt = struct.pack("<IIIH2x", (1 << HCI_EVENT_PKT) | (1 << HCI_ACLDATA_PKT),
                          0xFFFFFFFF, 0xFFFFFFFF, 0)
        if _lib().setsockopt(self.fd, SOL_HCI, HCI_FILTER, flt, len(flt)) < 0:
            err = ctypes.get_errno()
            os.close(self.fd)
            raise OSError(err, "setsockopt")
        self.pending = []            # (packet type, payload) not yet handed out

    def close(self):
        os.close(self.fd)

    def _read(self, timeout):
        if not _readable(self.fd, timeout):
            return None
        pkt = os.read(self.fd, 1100)
        if len(pkt) < 2:
            return None
        if pkt[0] == HCI_EVENT_PKT:
            if len(pkt) < 3:
                return None
            return HCI_EVENT_PKT, pkt[1], pkt[3:3 + pkt[2]]
        if pkt[0] == HCI_ACLDATA_PKT:
            return HCI_ACLDATA_PKT, None, pkt[1:]
        return None

    def command(self, opcode, params=b"", timeout=2.0):
        os.write(self.fd, struct.pack("<BHB", HCI_COMMAND_PKT, opcode, len(params)) + params)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pkt = self._read(end - time.monotonic())
            if pkt is None:
                continue
            kind, code, data = pkt
            if kind == HCI_EVENT_PKT and code == EVT_CMD_COMPLETE and len(data) >= 4 \
                    and struct.unpack("<H", data[1:3])[0] == opcode:
                if data[3]:
                    raise HciError(opcode, data[3])
                return data[4:]
            if kind == HCI_EVENT_PKT and code == EVT_CMD_STATUS and len(data) >= 4 \
                    and struct.unpack("<H", data[2:4])[0] == opcode:
                if data[0]:
                    raise HciError(opcode, data[0])
                return b""
            self.pending.append(pkt)
        raise TimeoutError("0x%04x: no answer" % opcode)

    def packets(self):
        """Everything that arrived outside a command, oldest first."""
        while True:
            pkt = self._read(0)
            if pkt is None:
                break
            self.pending.append(pkt)
        out, self.pending = self.pending, []
        return out

    def read_clock(self):
        before = time.monotonic_ns()
        ret = self.command(OP_READ_CLOCK, b"\x00\x00\x00")
        after = time.monotonic_ns()
        return before, struct.unpack("<I", ret[2:6])[0] & (CLKN_WRAP - 1), after

    def can_broadcast_3d(self):
        """LMP features page 2: CSB master (bit 0) and Synchronization Train (bit 2)."""
        try:
            ret = self.command(OP_READ_LOCAL_EXT_FEATURES, b"\x02")
        except BT_ERRORS:
            return False
        if len(ret) < 10 or ret[0] != 2:
            return False
        bits = int.from_bytes(ret[2:10], "little")
        return bits & 0b101 == 0b101


def can_broadcast_3d(index):
    try:
        hci = Hci(index)
    except OSError:
        return False
    try:
        return hci.can_broadcast_3d()
    finally:
        hci.close()


def list_adapters():
    """[(address, name, can drive 3D glasses)] for every adapter present.
    Read-only, so it is safe beside a running display."""
    mgmt = Mgmt()
    try:
        out = []
        for index in mgmt.indexes():
            try:
                addr, _settings, name = mgmt.details(index)
            except BT_ERRORS:
                continue
            out.append((addr, name, can_broadcast_3d(index)))
        return out
    finally:
        mgmt.close()


def find_adapter(mgmt, wanted="auto"):
    """(hci index, address) of the adapter to use: the one with this address,
    or with "auto" the first powered BR/EDR adapter that can broadcast 3D."""
    for index in mgmt.indexes():
        try:
            addr, settings = mgmt.info(index)
        except BT_ERRORS:
            continue
        if wanted and wanted != "auto":
            if addr == wanted.upper():
                return index, addr
            continue
        if settings & (MGMT_SETTING_POWERED | MGMT_SETTING_BREDR) != MGMT_SETTING_POWERED | MGMT_SETTING_BREDR:
            continue
        if can_broadcast_3d(index):
            return index, addr
    return None


# ---- the display -------------------------------------------------------------

def _csb_params(enable):
    return struct.pack("<BBBHHHH", 1 if enable else 0, LT_ADDR, 0, CSB_PACKET_TYPE,
                       CSB_INTERVAL, CSB_INTERVAL, CSB_TIMEOUT)


class Display:
    """One adapter as a 3D display. start() saves what it changes through
    save(saved) at once, so a crash cannot lose the originals, and stop() puts
    it back. keep() undoes what bluetoothd and the kernel change behind it."""

    def __init__(self, index, addr, log, saved, save):
        self.index = index
        self.addr = addr
        self.log = log
        self.saved = saved
        self.save = save
        self.hci = Hci(index)
        self.mgmt = Mgmt()
        self.eir = None
        self.discoverable = False    # pairing
        self.connections = {}        # ACL handle -> address

    # -- adapter state

    def _read_state(self):
        cls = self.hci.command(OP_READ_CLASS_OF_DEVICE)[:3]
        eir = self.hci.command(OP_READ_EIR)
        return cls, eir[:241]                       # FEC required + 240 bytes

    def _settings(self):
        return self.mgmt.info(self.index)[1]

    def start(self, message):
        self._clean_slate()
        if "class" not in self.saved:
            cls, eir = self._read_state()
            self.saved.update({"class": cls.hex(), "eir": eir.hex(),
                               "connectable": bool(self._settings() & MGMT_SETTING_CONNECTABLE)})
            for name, read, _write, _value in SCAN_SETTINGS:
                ret = self.hci.command(read)
                self.saved[name] = ret[:4 if name.endswith("scan") else 1].hex()
            self.save(self.saved)
        for _name, _read, write, value in SCAN_SETTINGS:
            self.hci.command(write, value)
        try:
            tx = struct.unpack("<b", self.hci.command(OP_READ_INQ_TX_POWER)[:1])[0]
        except BT_ERRORS:
            tx = 4
        self.eir = b"\x01" + eir_data(tx)
        self.hci.command(OP_SET_AFH_HOST_CLASSIFICATION, ALL_CHANNELS)
        self.hci.command(OP_SET_RESERVED_LT_ADDR, bytes([LT_ADDR]))
        self.set_message(message)
        self.hci.command(OP_SET_CSB, _csb_params(True))
        self.hci.command(OP_WRITE_SYNC_TRAIN_PARAMS, struct.pack(
            "<HHIB", SYNC_TRAIN_INTERVAL, SYNC_TRAIN_INTERVAL, SYNC_TRAIN_TIMEOUT, 0x00))
        self.start_sync_train()
        self.keep()
        self.log("bluetooth 3D display on hci%d (%s)" % (self.index, self.addr))

    def _clean_slate(self):
        """A previous run that died may have left the broadcast on."""
        for opcode, params in ((OP_SET_CSB, _csb_params(False)),
                               (OP_DELETE_RESERVED_LT_ADDR, bytes([LT_ADDR]))):
            try:
                self.hci.command(opcode, params)
            except BT_ERRORS:
                pass

    def _assert_identity(self):
        cls, eir = self._read_state()
        if cls != CLASS_3D_DISPLAY:
            self.hci.command(OP_WRITE_CLASS_OF_DEVICE, CLASS_3D_DISPLAY)
        if self.eir is not None and eir != self.eir:
            self.hci.command(OP_WRITE_EIR, self.eir)

    def keep(self):
        """Connectable (the kernel then accepts the glasses' announcements),
        discoverable while pairing, then class and EIR (last: the kernel
        rewrites the class when discoverable changes), and a sync train that
        never stays off."""
        settings = self._settings()
        if not settings & MGMT_SETTING_CONNECTABLE:
            self.mgmt.set_connectable(self.index, True)
        if self.discoverable and not settings & MGMT_SETTING_DISCOVERABLE:
            self.mgmt.set_discoverable(self.index, True)
        self._assert_identity()
        # glasses find the broadcast through the train, and lose it when the
        # controller drops either. The train's end event does not come on every
        # controller (the UB500 sends none), so check both every time: "already
        # on" costs one round trip each.
        self.revive("periodic check")

    def ensure_broadcast(self):
        """Set CSB on again. True if the controller took it (it had stopped),
        False if it answered "already on"."""
        try:
            self.hci.command(OP_SET_CSB, _csb_params(True))
        except HciError as e:
            if e.status != 0x0C:                     # Command Disallowed: already on
                raise
            return False
        return True

    def revive(self, why):
        """The broadcast and the train back on, logging what had stopped."""
        stopped = [name for name, step in (("broadcast", self.ensure_broadcast),
                                           ("sync train", self.start_sync_train)) if step()]
        if stopped:
            self.log("bluetooth %s had stopped (%s): started again" % (" and ".join(stopped), why))

    def start_sync_train(self):
        """True if it was stopped and now runs, False if it was running."""
        try:
            self.hci.command(OP_START_SYNC_TRAIN)
        except HciError as e:
            if e.status != 0x0C:                     # Command Disallowed: already running
                raise
            return False
        return True

    def set_message(self, message):
        self.hci.command(OP_SET_CSB_DATA, bytes([LT_ADDR, 0x03, len(message)]) + message)

    def pair(self, on):
        """Discoverable: glasses in pairing mode find the box by inquiry and
        pick the nearest 3D display. Until switched off."""
        self.discoverable = on
        self.mgmt.set_discoverable(self.index, on)
        self._assert_identity()

    def stop(self):
        """Also after a start that failed half way: whatever was saved is put back."""
        self._clean_slate()
        if "class" in self.saved:
            for step in (self._restore_discoverable, self._restore_connectable,
                         self._restore_identity):
                try:
                    step()
                except BT_ERRORS as e:
                    self.log("bluetooth restore: %s" % e)
        self._close()

    def _restore_discoverable(self):
        if self.discoverable or self._settings() & MGMT_SETTING_DISCOVERABLE:
            self.mgmt.set_discoverable(self.index, False)

    def _restore_connectable(self):
        if not self.saved.get("connectable", True):
            self.mgmt.set_connectable(self.index, False)

    def _restore_identity(self):
        self.hci.command(OP_WRITE_CLASS_OF_DEVICE, bytes.fromhex(self.saved["class"]))
        self.hci.command(OP_WRITE_EIR, bytes.fromhex(self.saved["eir"]))
        for name, _read, write, _value in SCAN_SETTINGS:
            if name in self.saved:
                self.hci.command(write, bytes.fromhex(self.saved[name]))
        self.saved.clear()
        self.save(self.saved)

    def _close(self):
        for sock in (self.hci, self.mgmt):
            try:
                sock.close()
            except OSError:
                pass

    # -- what arrives

    def poll(self):
        """Handle what arrived: sync train restarts, class or EIR written by
        someone else (their Command Complete reaches this socket too; ours are
        taken by command()), connections, announcements and names. Returns
        [(address, associated, battery request, battery %)] and {address: name}."""
        announced, names = [], {}
        rewritten = False
        for kind, code, data in self.hci.packets():
            if kind == HCI_ACLDATA_PKT:
                ann = parse_announcement(data)
                if ann and ann[0] in self.connections:
                    addr = self.connections[ann[0]]
                    announced.append((addr,) + ann[1:])
                    self._ask_name(addr)
            elif code in RESTART_SYNC_TRAIN:
                self.start_sync_train()
            elif code == EVT_CMD_COMPLETE and len(data) >= 3:
                rewritten |= struct.unpack("<H", data[1:3])[0] in (OP_WRITE_CLASS_OF_DEVICE, OP_WRITE_EIR)
            elif code == EVT_CONN_COMPLETE and len(data) >= 9 and data[0] == 0:
                self.connections[struct.unpack("<H", data[1:3])[0] & 0x0FFF] = format_addr(data[3:9])
            elif code == EVT_DISCONN_COMPLETE and len(data) >= 3 and data[0] == 0:
                gone = self.connections.pop(struct.unpack("<H", data[1:3])[0] & 0x0FFF, None)
                if gone:
                    # glasses connect just after locking on, to report; some
                    # controllers drop the broadcast or the train meanwhile
                    self.revive("after %s's connection" % gone)
            elif code == EVT_REMOTE_NAME and len(data) >= 7 and data[0] == 0:
                name = data[7:].split(b"\x00", 1)[0].decode("utf-8", "replace").strip()
                if name:
                    names[format_addr(data[1:7])] = name
        if rewritten:
            self._assert_identity()
        return announced, names

    def _ask_name(self, addr):
        """While the announcement's link is up the name costs one round trip."""
        try:
            self.hci.command(OP_REMOTE_NAME_REQUEST, parse_addr(addr) + b"\x01\x00\x00\x00")
        except BT_ERRORS:
            pass


# ---- the thread --------------------------------------------------------------

GRACE_S = 10.0                   # 3D gone this long (end of the film, not a seek): Bluetooth off
RETRY_S = 10.0                   # a start that failed is retried this often while wanted
PROBE_S = 30.0                   # no adapter yet: look again this often


def wanted(now, last_3d, pairing, test_hz):
    """The display is on only for a 3D film (and GRACE_S after it), pairing,
    or a glasses test; otherwise the adapter is left as it was."""
    return now - last_3d < GRACE_S or pairing or bool(test_hz)


class BluetoothLink(threading.Thread):
    """Runs for the life of the service. Watches fa_sync; while wanted() it
    holds the adapter as a 3D display, broadcasts the glasses' timing and
    reports announcements, and in between it gives the adapter back.

    settings: object with bt_adapter, delay_us, invert, open_after_us, open_us.
    store: object with saved_adapter(addr) -> dict, saved_adapters() -> {addr: dict},
    save_adapter(addr, dict), glasses_seen(addr, associated, battery_request,
    battery), glasses_name(addr, name).
    on_problem(reason): the display was wanted and could not start (once per
    failure, not per retry).
    pairing and test_hz are switches the Kodi thread sets directly.
    """

    def __init__(self, settings, log, store, on_problem=None):
        super().__init__(name="shutter3d-bluetooth", daemon=True)
        self.settings = settings
        self.log = log
        self.store = store
        self.on_problem = on_problem
        self.stopping = threading.Event()
        self.reopen = threading.Event()
        self.display = None
        self.adapter = None          # address of a usable adapter, None until found
        self.pairing = False
        self.test_hz = 0.0
        self.in_3d = False
        self.problem = ""            # why the display cannot start, while wanted
        self.on_since = None         # wall clock when the broadcast started, None while off
        self.clock = ClockMap()
        self.next_update = self.next_keep = self.last_sent_at = 0.0
        self.last_sent = None

    # -- from the Kodi thread

    def retune(self, settings):
        if settings.bt_adapter != self.settings.bt_adapter:
            self.reopen.set()
        self.settings = settings

    def pair(self, on):
        self.pairing = on

    def test(self, hz):
        self.test_hz = hz

    def stop(self):
        self.stopping.set()
        self.join(timeout=3.0)

    def state(self):
        """For the screen: what the link is doing, as plain values."""
        return {"link": "bluetooth", "adapter": self.adapter or "", "on_since": self.on_since,
                "in_3d": self.in_3d,
                "pairing": self.pairing, "test_hz": self.test_hz, "problem": self.problem}

    # -- the loop

    def run(self):
        try:
            fd = os.open(FA_SYNC, os.O_RDONLY)
        except OSError as e:
            self.log("no %s (kernel without the frame-sequential patches?): %s" % (FA_SYNC, e))
            self.problem = "no frame-sequential 3D in this build"
            return
        self._recover()
        track = VsyncTrack()
        last_3d = -GRACE_S
        next_probe = 0.0                 # idle, no adapter yet: the next search
        next_start = 0.0                 # wanted: the next start after one failed
        try:
            while not self.stopping.is_set():
                if self.reopen.is_set():
                    self.reopen.clear()
                    self._close()
                    self.adapter = None
                    next_probe = next_start = 0.0
                try:
                    # blocks until the next 3D vsync, or 100 ms and "off"
                    sample = core.parse_fa_sync(os.pread(fd, 64, 0).decode("ascii", "replace"))
                except InterruptedError:
                    continue
                now = time.monotonic()
                if sample is not None:
                    track.add(*sample)
                    last_3d = now
                    if self.test_hz:
                        self.test_hz = 0.0               # a 3D film takes over from a test
                        self.log("bluetooth glasses test off: a 3D film started")
                elif track.last is not None:
                    track = VsyncTrack()
                self.in_3d = track.ready()

                if not wanted(now, last_3d, self.pairing, self.test_hz):
                    self._set_problem("")
                    if self.display is not None:
                        self._close()
                        self.log("bluetooth off: no 3D, pairing or test")
                    if self.adapter is None and now >= next_probe:
                        next_probe = now + PROBE_S
                        self._probe()
                    continue
                if self.display is None:
                    if now < next_start:
                        continue
                    problem = self._open()
                    self._set_problem(problem)
                    if problem:
                        next_start = now + RETRY_S       # only a failed start waits
                        continue
                if now < self.next_update:
                    continue
                self.next_update = now + UPDATE_S
                try:
                    self._tick(track, now)
                except Exception as e:  # noqa: BLE001 - the thread must outlive any one update
                    self.log("bluetooth: %s" % e)
                    self._close()
                    self._set_problem("adapter lost, retrying")
                    next_start = now + RETRY_S
        finally:
            os.close(fd)
            self._close()

    def _tick(self, track, now):
        """One update: pairing, the message, the periodic keep, what arrived."""
        d = self.display
        if d is None:
            return
        # the Kodi thread flips these at any time: one value per update
        pairing, test_hz = self.pairing, self.test_hz
        if d.discoverable != pairing:
            d.pair(pairing)
            self.log("bluetooth pairing %s" % ("on" if pairing else "off"))
        if track.ready() or test_hz:
            self.clock.add(*d.hci.read_clock())
            message = self._message(track, test_hz) if self.clock.ready() else None
        else:
            self.clock = ClockMap()                  # timing starts afresh, the next film or test
            message = MESSAGE_2D
        if message is not None and (message != self.last_sent or now - self.last_sent_at >= RESEND_S):
            d.set_message(message)
            self.last_sent, self.last_sent_at = message, now
        if now >= self.next_keep:
            self.next_keep = now + KEEP_S
            d.keep()
        announced, names = d.poll()
        for addr, associated, battery_request, battery in announced:
            self.store.glasses_seen(addr, associated, battery_request, battery)
        for addr, name in names.items():
            self.store.glasses_name(addr, name)

    def _set_problem(self, reason):
        if reason and not self.problem and self.on_problem:
            self.on_problem(reason)
        self.problem = reason

    def _find(self):
        """((index, address) or None, why not)."""
        try:
            mgmt = Mgmt()
        except OSError as e:
            self.log("bluetooth management: %s" % e)
            return None, "no Bluetooth"
        try:
            found = find_adapter(mgmt, self.settings.bt_adapter)
        finally:
            mgmt.close()
        return found, "" if found else "no adapter that can broadcast 3D sync"

    def _probe(self):
        """Idle: only find the adapter, so the screen can say whether it will work."""
        found, _ = self._find()
        self.adapter = found[1] if found else None

    def _display(self, index, addr):
        return Display(index, addr, self.log, self.store.saved_adapter(addr),
                       lambda saved: self.store.save_adapter(addr, saved))

    def _recover(self):
        """Kodi died with the display on: give those adapters back now, not
        at the next film."""
        for addr in self.store.saved_adapters():
            try:
                mgmt = Mgmt()
                try:
                    found = find_adapter(mgmt, addr)
                finally:
                    mgmt.close()
                if found:
                    self._display(*found).stop()
                    self.log("bluetooth: put back %s after an unclean stop" % addr)
            except BT_ERRORS as e:
                self.log("bluetooth recover %s: %s" % (addr, e))

    def _open(self):
        """"" once the display is on, otherwise why not."""
        found, problem = self._find()
        if not found:
            self.adapter = None
            return problem
        index, addr = found
        self.adapter = addr
        try:
            display = self._display(index, addr)
        except OSError as e:
            self.log("bluetooth hci%d: %s" % (index, e))
            return "cannot open the adapter"
        try:
            display.start(MESSAGE_2D)
        except BT_ERRORS as e:
            self.log("bluetooth hci%d start: %s" % (index, e))
            display.stop()
            return "adapter refused the 3D broadcast"
        self.display = display
        self.on_since = time.time()
        self.clock = ClockMap()
        self.next_update = self.next_keep = 0.0
        self.last_sent = None
        return ""

    def _close(self):
        self.on_since = None
        if self.display is not None:
            self.display.stop()
            self.display = None

    def _message(self, track, test_hz):
        s = self.settings
        if track.ready():
            period_us = 2 * track.period_ns / 1000.0
            fsi = self.clock.to_clk_us(track.left_ns)
        else:
            # the test: one steady rhythm, whole periods from the clock's zero.
            # A new instant per message would move the phase every 80 ms, and
            # glasses open both lenses while it is off by more than 500 us.
            period_us = 2e6 / test_hz
            fsi = self.clock.to_clk_us(time.monotonic_ns()) // period_us * period_us
        fsi += s.delay_us + (period_us / 2 if s.invert else 0)
        return pack_message(fsi, period_us, window_offsets(period_us, s.open_after_us, s.open_us))
