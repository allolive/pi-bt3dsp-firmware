# Glasses runs

Every claim about what the glasses do points to rows here. One variant per
run, Pi rebooted before each run, Bluetooth recorded (btmon) for the whole
run. Judged by eye until the light rig is set up. Glasses: Hi-Shock RF Pro.

Evidence: PROVEN = 3+ runs agree. SUPPORTED = fewer, or partly confounded.
ANECDOTE = one observation, or confounded. VOID = reboot, cable fault, or
unknown glasses state.

| Date | Start | Build | Variant | Rate | Pairs | Start state | Result | Train restarts |
|---|---|---|---|---|---|---|---|---|
| 2026-10-14 | 16:12:26 | c780e09 | baseline (direct control, bluetooth.py parity) | free 96.000 | 1 (2nd in pairing mode) | glasses off | held 6+ min | 3 |
| 2026-10-14 | 16:22:11 | c780e09 | baseline | free 96.000 | 2 | glasses still on from run 1: dropped while the Pi rebooted, re-acquired by themselves | held 6 min | 3 |
| 2026-10-14 | 16:31:18 | c780e09 | baseline | free 96.000 | 2 | glasses off, switched on at start | held 12 min | 6 |
| 2026-10-14 | 16:46:55 | c780e09 | baseline | free 95.904 | 2 | glasses off; 2nd pair needed a second button press | held 5+ min | 2+ |
| 2026-10-14 | 17:03:09 | c780e09 | baseline, following fake-host vsyncs (laptop) | sync 96.000 | 2 | glasses off; 2nd pair slow to start from the button | held 5+ min | 1+ |
| 2026-10-14 | 17:10:50 | c780e09 | baseline, following fake-host vsyncs | sync 95.904 | 2 | glasses left on, idle after the previous run: both resumed by themselves | held 4.5+ min | 2 |
| 2026-10-14 | 17:17:55 | d1707e8 | V 1: discoverable during 3D, light inquiry scan | sync 95.904 | 2 | glasses left on, idle: both resumed by themselves | held 6 min | 3 |
| 2026-10-14 | ~17:25 | d1707e8 | same run, then a third pair in pairing mode next to the Pi | sync 95.904 | 3 | pairing during 3D | the third pair paired and held | - |
| 2026-10-14 | 17:33:10 | f23011a | minimal shipped image (bt3dd as PID 1, static) | sync 95.904 | 3 | glasses left on, idle: resumed by themselves | held (5 min not confirmed) | - |
| 2026-10-14 | 17:42:01 | f23011a+LED | minimal image, cut-down GPU firmware (start_cd.elf) | sync 95.904 | 3 | glasses left on, idle: resumed by themselves | held 6.5 min; LED blinked during 3D | 3 |
| 2026-10-14 | 18:20:39 | cleanup (after a7df533) | minimal image, code simplified (same messages by design), booted with `B` | sync 95.904 | 3 | glasses left on, idle: resumed by themselves | worked | - |

PROVEN (3/3): the baseline holds both pairs through repeated 120 s sync-train
ends restarted with Start_Sync_Train alone.

SUPPORTED (1 run each): 95.904 Hz free-run; following vsyncs at 96.000 and
95.904 Hz; glasses left on resume by themselves when a broadcast starts;
discoverable during 3D at the light inquiry duty (V 1) holds, and a new pair
pairs during 3D with it.

Also seen: pairing a second pair with `A 1` (discoverable, scans at 50 %)
during a running broadcast worked, and the synced pair kept going (once:
ANECDOTE).

Earlier, before this ledger (bt3dd branch bt3dd-shared and uncommitted
builds): glasses started and dropped within ~2 min in every bt3dd run that
had any of: the 0x07FFFFFE train, a full CSB restart on 0x4F, ~84 ms updates,
discoverable during 3D. Which of these mattered is not known.

Other checks, 2026-10-14 17:48-17:52 (minimal image, start_cd):
- LED: off while booting, steady on with the host connected, blinking 1 Hz
  while 3D is broadcast, back to steady when it stops (seen by eye).
- `B` reboots the Pi to the boot ROM and the host boots it again: worked
  twice. Once, right after a `B`, the freshly booted Pi rebooted again 4 s
  after "ready", with nothing on its console (ANECDOTE, not reproduced).

On the box, 2026-10-14 18:47-19:40 (add-on 0.9.0, then 0.9.1 with firmware
245a61e), Pi booted by the add-on's Python booter:
- 0.9.0 (release a74f863): the band of the other eye grew and shrank at the
  top and bottom. The Pi's status showed `rate_changes` rising ~50/s: the box
  stamps its vsyncs with +-120 us of jitter (measured, 1st-99th percentile),
  and a single step 0.5 % off restarted the period. Fixed in 245a61e (a
  least-squares line through the span's stamps, rate changes judged on
  128-frame averages). The simulation now includes that jitter.
- 0.9.1 updated the Pi by itself (`B`, then booted 245a61e). `rate_changes`
  stayed 0. The viewer saw an occasional jump, under one a minute; the sync
  train restarts every 120 s and may be the cause (ANECDOTE, not confirmed).
  Several films afterwards showed no issue. Left as it is; to watch.

