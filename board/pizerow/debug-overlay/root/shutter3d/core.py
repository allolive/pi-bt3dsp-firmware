"""Frame-sequential 3D, free of Kodi and of the box. Kodi shows the eyes
alternately on every vsync; the kernel reports each one on
/sys/class/video/fa_sync as "<vsync> <ns> <L|R|->"."""


def parse_fa_sync(text):
    """/sys/class/video/fa_sync: (vsync, ns, eye) or None for "off"."""
    parts = text.split()
    if len(parts) != 3:
        return None
    try:
        return int(parts[0]), int(parts[1]), parts[2]
    except ValueError:
        return None


def sync_line(vsync, ns, eye):
    """The board's sync message for one vsync: its number and time in
    microseconds, both mod 2^32."""
    return "S %d %d %s\n" % (vsync & 0xFFFFFFFF, (ns // 1000) & 0xFFFFFFFF, eye)
