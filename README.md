# pi-bt3dsp-firmware

**Bluetooth 3D glasses for [CoreELEC yacer builds](https://github.com/allolive/CoreELEC/tree/yacer),
with a Raspberry Pi Zero W (v1.1, not the Zero 2 W) as the emitter.** Plug the
Pi into the box running the yacer build with one USB cable: no SD card, no
setup on the Pi. The box boots it, sends it the timing of every frame, and the
Pi broadcasts it to Bluetooth active-shutter glasses, which then switch eyes
in step with the picture.

- **CoreELEC yacer builds:** the source is the
  [`yacer` branch](https://github.com/allolive/CoreELEC/tree/yacer), and each
  build is a [release](https://github.com/allolive/CoreELEC/releases) you can
  install as an update on a CoreELEC box.
- **The Kodi add-on** *Active 3D for 120 Hz VRR displays*
  (`service.shutter3d.allolive`) shows 3D films frame-sequentially on a 2D TV
  with game VRR (95.904 Hz for 23.976 films) and drives the glasses through
  this Pi.

## What you get

- **Glasses sync without an emitter box**: the Pi speaks the Bluetooth 3D
  Synchronization Profile (3DSP), the standard Bluetooth 3D glasses use.
- **Pairing during the film**: while 3D plays, hold a new pair's pairing
  button near the Pi; paired glasses come back on by themselves at the next
  3D film.
- **Glasses sleep when 3D stops**: the broadcast ends about ten seconds after
  the last 3D frame (a seek or a menu does not drop them).
- **Nothing to install on the Pi**: it starts from the box over USB at every
  power-up, from a 5 MB image the add-on carries, so updating the add-on
  updates the Pi.
- **Status LED**: off without a host, steady with one, blinking while 3D is
  broadcast.

Glasses tested: Hi-Shock RF Pro (three pairs at once). Also tried: XPAND
X105-RF-X1. Runs, rates and what each showed are in [RUNS.md](RUNS.md).

## Hardware

> **Raspberry Pi Zero W only (the first one, board v1.1).** Everything here
> was built and tested on it alone. The **Zero 2 W is not supported**: it has
> another processor (RP3A0) and another Bluetooth chip (CYW43436), which need
> their own kernel, device tree and Bluetooth firmware, and it has never been
> tried. Expect this image not to work on it.

- A Raspberry Pi Zero W, v1.1 (BCM2835; Bluetooth: CYW43438). No SD card.
- One micro-USB **data** cable into the Pi's inner port, labelled **USB**
  (not PWR). It carries power and data; leave PWR empty. Many micro-USB
  cables are charge-only: with one of those the Pi powers up but never shows
  up on the host.

## How it works

1. With no SD card, the Pi's boot ROM waits on USB (`0a5c:2763`). The host
   sends it the boot files with the rpiboot protocol (the add-on, or
   `rpiboot` on a PC).
2. Linux runs from RAM with one program, `bt3dd`. The Pi then shows the host
   two USB serial ports: the first carries the protocol, the second its log.
3. The host sends each vsync's number, time and eye
   (`S <vsync> <us> <L|R|->`).
   `bt3dd` maps those times onto the Bluetooth clock and broadcasts where each
   left-right frame pair starts and how long it lasts, about nine times a
   second. The glasses run their own timer from it. The frame times are
   fitted over the last ten seconds or so, so the Pi follows a host clock
   that NTP slews (stamping with a clock it does not slew is better still).

The full protocol is under [Serial protocol](#serial-protocol) below.

## Serial protocol

The Pi shows the host two USB serial ports (CDC ACM; on Linux usually
`/dev/ttyACM0` and `/dev/ttyACM1`, in that order):

| Port | Direction | Carries |
|---|---|---|
| first (`ttyACM0`) | both | the protocol below |
| second (`ttyACM1`) | Pi to host | the log: `bt3dd: ...` lines and kernel messages (on the debug image, also a root shell) |

The baud rate does not matter (it is USB). One command per line, ended by `\n`
or `\r`, at most 95 characters. Commands are a letter, a space, and decimal
numbers; anything unknown is ignored. Nothing is answered except `?`; replies
and events are lines ending in `\n`.

**The Pi keeps no settings.** It runs from RAM and starts from its defaults at
every boot: the host sends what it wants after each connection.

### Commands

| Command | Default | Meaning |
|---|---|---|
| `S <vsync> <us> <L\|R\|->` | | One vsync from the host, sent as it happens: its number (counting up, wraps at 2^32), its time in microseconds on any steady host clock (wraps at 2^32), and the eye shown from it (`-`: no 3D). Only used in mode 2. The Pi fits the host's clock to its own; the frame period is the host's time over the vsync count, so it is exact. |
| `M <0\|1\|2>` | `2` | Mode. `0` off (no broadcast, unless pairing). `1` free-run at `F`'s rate (testing without a host). `2` follow the host's `S` lines. Changing it restarts the clock fit. |
| `F <millihertz>` | `96000` | Free-run display rate: `96000` = 96 Hz, `95904` = 95.904 Hz (23.976 films). The glasses switch eyes at half of it. |
| `D <us>` | `0` | Delay from the vsync to the eye switch, may be negative. This is the 3D glasses delay the add-on tunes. |
| `I <0\|1>` | `0` | Swap the eyes (half a frame pair later). |
| `O <open_after_us> <open_us>` | `0 0` | Lens window: each lens opens `open_after_us` after its eye's frame starts and stays open `open_us` (`0`: until the other eye's frame starts). Shortening it cuts the ghosting from the TV drawing top to bottom. |
| `A <0\|1>` | `0` | Pairing window: the Pi is discoverable with a heavy scan and broadcasts even without 3D, for glasses held in pairing mode next to it. Turn it off again. |
| `V <0\|1>` | `1` | Discoverable during 3D with a light scan, so new glasses pair during a film. |
| `G <seconds>` | `10` | How long after the last 3D frame the broadcast stops (1 to 3600). Then the glasses go to sleep. |
| `B` | | Reboot to the boot ROM: the Pi leaves USB and comes back as `0a5c:2763`, for the host to boot it again (an update). Shipped image only. |
| `?` | | One status line (below). |

Between the last `S` with an eye and the broadcast stopping, the glasses
coast: for 2 s on the last timing, then with both lenses open until `G`
seconds have passed.

### Status line (`?`)

One line of `name value` pairs, starting with `pi-bt3dsp <version> build
<build>`:

```
pi-bt3dsp 0.1.1 build 6768d11 mode 2 delay_us 500 invert 0 free_mhz 96000 period_us 10427.134
rate_changes 0 skew_ppm 8.5 fit_windows 8 msgs 3026 resets 3 last_resid_us 176
running 1 bt 1 address B8:27:EB:7B:97:61 open_after_us 6500 open_us 0 pairing 0
discov_3d 1 grace_s 10 bt_ppm -15.2 clock_samples 370 train_restarts 2 glasses 0
host_off_us -914468871.9 floor_us -1925.9 bt_off_us -2010248.0 bt_rtt_min_us 617
bt_rtt_used_us 689 fsi_jump_last_us 2 fsi_jump_max_us -23 fsi_jumps 4 reset_why mode
reset_age_s 312 train_age_s 47
```

(one line on the wire; wrapped here)

| Field | Meaning |
|---|---|
| `build` | the git commit the firmware was built from; the release's `boot/build.txt` holds the same, so a host can tell whether the Pi runs the image it has |
| `mode`, `delay_us`, `invert`, `free_mhz`, `open_after_us`, `open_us`, `pairing`, `discov_3d`, `grace_s` | the settings above |
| `period_us` | the host's frame period, measured from its `S` lines |
| `rate_changes` | rate changes seen in the `S` lines (a VRR change between films) |
| `skew_ppm`, `fit_windows`, `last_resid_us` | the host clock against the Pi's: drift, seconds of fit (up to 8), and how late the last line arrived above the fitted floor |
| `msgs`, `resets` | `S` lines received, and how often the clock fit started over (a host gap of 2 s or a jump) |
| `running` | 1 while there is 3D to show (mode 2 with recent `S` eyes, or mode 1) |
| `bt` | 1 while the broadcast is on |
| `address` | the Pi's Bluetooth address, what glasses pair with |
| `bt_ppm`, `clock_samples` | the Bluetooth clock against the Pi's, and the readings it was fitted from |
| `train_restarts` | how often the sync train was restarted (it runs 120 s at a time) |
| `glasses` | announcements received from glasses |
| `host_off_us`, `floor_us` | the host clock's offset onto the Pi's, in us, and the USB floor it stands on; against a straight line over time they show the host clock wandering (NTP slewing it, for one) |
| `bt_off_us`, `bt_rtt_min_us`, `bt_rtt_used_us` | the Bluetooth clock's offset from the Pi's, and the bracket widths of the readings it is fitted from (the narrowest, and the widest of the half used) |
| `fsi_jump_last_us`, `fsi_jump_max_us`, `fsi_jumps` | how far the broadcast frame instant moved between two messages against where the previous one put it: the last, the largest since the last status line, and how many moved over 100 us |
| `reset_why`, `reset_age_s` | why the clock fit last started over (`mode`, `repeat`, `frames`, `time`, `silent`), and how long ago |
| `train_age_s` | seconds since the sync train last (re)started |

The second port logs `bt3dd: ...` lines when the clock fit starts over,
when the sync train restarts and when the frame instant moves over 100 us.

### Events (Pi to host, unasked)

| Line | When |
|---|---|
| `G <address> <associated> <battery_request> <battery>` | Glasses announced themselves after syncing: `associated` 1 when paired with this Pi, `battery` in % or `-1` when unknown. |
| `N <address> <name>` | The name of glasses that just announced themselves. |

Glasses do not have to announce: the Hi-Shock RF Pro sync without it.

### Examples

From a shell on the host:

```
stty -F /dev/ttyACM0 raw -echo
cat /dev/ttyACM0 &                 # replies and events
echo '?' > /dev/ttyACM0            # status
echo 'F 95904' > /dev/ttyACM0      # free-run at the 23.976 film rate...
echo 'M 1' > /dev/ttyACM0          # ...now: the glasses sync
echo 'M 0' > /dev/ttyACM0          # off: the glasses sleep
cat /dev/ttyACM1                   # the Pi's log
```

From Python, following a host's vsyncs:

```python
import os, termios
fd = os.open("/dev/ttyACM0", os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd); a[3] &= ~(termios.ECHO | termios.ICANON); termios.tcsetattr(fd, termios.TCSANOW, a)
os.write(fd, b"M 2\nD 0\n")

def vsync(n, t_ns, eye):          # once per vsync, as it happens; eye b"L", b"R" or b"-"
    os.write(fd, b"S %d %d %s\n" % (n & 0xFFFFFFFF, (t_ns // 1000) & 0xFFFFFFFF, eye))
```

## Build and try on a PC

```
make            # the shipped image, output-min/images/boot/ (Buildroot 2026.02.x)
make boot       # send it to the Pi (Ubuntu: apt install rpiboot; no sudo needed)
make test       # host simulation of the timing core
make debug      # the debug image: shell on the second port, Python, BlueZ tools
make boot-debug
```

The first build takes 15-30 minutes on a laptop, mostly the kernel. After `make boot`
the Pi appears as `/dev/ttyACM0` (protocol: try `?`) and `/dev/ttyACM1`
(its log).

## Images

- **Shipped** (`make`, 5 MB of boot files): `bt3dd` as the only process
  (PID 1, static), no shell, the cut-down GPU firmware, Wi-Fi, video and audio
  off, the CPU at its lowest clock. It feeds the hardware watchdog and reboots
  to the boot ROM if USB stays down, so the host can always boot it again.
- **Debug** (`make debug`): a root shell on the second serial port, Python
  and the BlueZ tools, with two bring-up scripts in `/root`:
  - `check_csb.py`: the 3DSP features the controller lists. It says "no" on
    the Pi: the CYW43438 runs the sync train but does not set its feature bit.
  - `glasses_test.py [hz] [pair_s] [run_s]`: the add-on's Python Bluetooth
    display, free-running. Run it with `REVIVE=0`: re-sending "broadcast on"
    while it runs ends the sync train on this controller.

## Builds and releases

GitHub Actions runs the timing simulation and builds the image on every push
and pull request. Every build of `main` is published as a
[release](https://github.com/allolive/pi-bt3dsp-firmware/releases), tagged
`v<version>-<date>-<commit>` and marked the latest; a `v*` tag matching
`bt3dd`'s version publishes a release under that name. Each release has
`pi-bt3dsp-<version>.tar.gz` (the boot files the add-on bundles), its SHA-256,
and the sources of everything in the image. Locally: `make dist`,
`make legal-info`.

## Notes

- Ubuntu 26.04's uutils `install` breaks Buildroot. The Makefile puts GNU
  `install` (`/usr/bin/gnuinstall`) first on the build's PATH.
- The serial ports declare no AT command set (a kernel patch), so
  ModemManager on a desktop host leaves them alone.

## License

Free for personal and other non-commercial use, not for sale: PolyForm
Noncommercial 1.0.0, see `LICENSE` (bt3dd and the build files). The image
also contains the Linux kernel (GPL-2.0), the Raspberry Pi firmware and the
Broadcom Bluetooth patch under their own licences; the release's sources
archive has them all.
