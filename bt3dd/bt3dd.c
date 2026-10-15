/* bt3dd: a Raspberry Pi Zero W as a Bluetooth 3D display (3DSP 3DD).
 *
 * The host sends, over USB serial (ttyGS0), every vsync's number, time and
 * eye. bt3dd maps those times onto the Bluetooth native clock and broadcasts
 * the shutter timing to the glasses (Connectionless Slave Broadcast and a
 * Synchronization Train). Kernel and
 * bt3dd messages go to the second serial port (ttyGS1).
 *
 * It runs as PID 1 in an initramfs with nothing else: it mounts what it needs,
 * holds the controller through an HCI user channel (nothing else talks to it;
 * it sets the controller up the way the kernel would, then as the add-on's
 * bluetooth.py does: that Python display held glasses for minutes),
 * feeds the hardware watchdog, and reboots when the USB link stays down: with
 * no SD card the Pi then waits in its boot ROM for the host to boot it again.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/reboot.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/watchdog.h>

#include "sync.h"

#define VERSION "0.1.1"
#ifndef BT3DD_BUILD
#define BT3DD_BUILD "unknown"                /* the build's git commit, from the package */
#endif

#define PROTO_TTY "/dev/ttyGS0"
#define LOG_TTY "/dev/ttyGS1"
#define UDC_STATE "/sys/class/udc/20980000.usb/state"

/* one update every ~108 ms on average, the pace bluetooth.py ran at when glasses
 * held (100 ms + 0-17 ms dither); 80 ms put every update next to a broadcast */
#define TICK_NS 100000000LL
#define TICK_DITHER_NS 17000000
#define IDLE_TICK_NS NS_PER_S            /* nothing to broadcast */
#define COAST_NS (2 * NS_PER_S)          /* host silent this long: lenses open (2D) */
#define GRACE_DEFAULT_S 10               /* no 3D this long: broadcast off, glasses sleep */
#define RESEND_NS NS_PER_S               /* an unchanged message is still refreshed */
#define USB_DOWN_REBOOT_S 15             /* link down this long: reboot to the boot ROM */
#define WATCHDOG_S 15

/* ---- HCI ------------------------------------------------------------------ */

/* The few BlueZ socket definitions bt3dd needs, here rather than from
 * <bluetooth/hci.h>, so the image needs no BlueZ at all. */
#ifndef AF_BLUETOOTH
#define AF_BLUETOOTH 31
#endif
#define BTPROTO_HCI 1
#define HCI_CHANNEL_USER 1
#define HCIDEVDOWN _IOW('H', 202, int)

struct sockaddr_hci {
	unsigned short hci_family;
	unsigned short hci_dev;
	unsigned short hci_channel;
};

#define OP(ogf, ocf) ((uint16_t)(((ogf) << 10) | (ocf)))
#define OP_ACCEPT_CONN OP(0x01, 0x009)
#define OP_REJECT_CONN OP(0x01, 0x00A)
#define OP_LINK_KEY_NEG OP(0x01, 0x00C)
#define OP_PIN_CODE_NEG OP(0x01, 0x00E)
#define OP_REMOTE_NAME OP(0x01, 0x019)
#define OP_IO_CAP_NEG OP(0x01, 0x034)
#define OP_RESET OP(0x03, 0x003)
#define OP_SET_EVENT_MASK OP(0x03, 0x001)
#define OP_WRITE_SSP_MODE OP(0x03, 0x056)
#define OP_WRITE_INQUIRY_MODE OP(0x03, 0x045)
#define OP_WRITE_CONN_ACCEPT_TIMEOUT OP(0x03, 0x016)
#define OP_WRITE_DEFAULT_LINK_POLICY OP(0x02, 0x00F)
#define OP_LE_SET_EVENT_MASK OP(0x08, 0x001)
#define OP_LE_CLEAR_ACCEPT_LIST OP(0x08, 0x010)
#define OP_WRITE_LE_HOST_SUPPORTED OP(0x03, 0x06D)
#define OP_DELETE_STORED_LINK_KEY OP(0x03, 0x012)
#define OP_WRITE_SC_HOST_SUPPORT OP(0x03, 0x07A)
#define OP_WRITE_LOCAL_NAME OP(0x03, 0x013)
#define OP_WRITE_SCAN_ENABLE OP(0x03, 0x01A)
#define OP_SET_EVENT_MASK_2 OP(0x03, 0x063)
#define OP_SET_CSB OP(0x01, 0x041)
#define OP_START_SYNC_TRAIN OP(0x01, 0x043)
#define OP_WRITE_PAGE_SCAN OP(0x03, 0x01C)
#define OP_WRITE_INQ_SCAN OP(0x03, 0x01E)
#define OP_WRITE_CLASS OP(0x03, 0x024)
#define OP_SET_AFH_CLASS OP(0x03, 0x03F)
#define OP_WRITE_INQ_SCAN_TYPE OP(0x03, 0x043)
#define OP_WRITE_PAGE_SCAN_TYPE OP(0x03, 0x047)
#define OP_WRITE_EIR OP(0x03, 0x052)
#define OP_READ_INQ_TX_POWER OP(0x03, 0x058)
#define OP_SET_RESERVED_LT OP(0x03, 0x074)
#define OP_DELETE_RESERVED_LT OP(0x03, 0x075)
#define OP_SET_CSB_DATA OP(0x03, 0x076)
#define OP_WRITE_SYNC_TRAIN OP(0x03, 0x078)
#define OP_READ_LOCAL_VERSION OP(0x04, 0x001)
#define OP_READ_BDADDR OP(0x04, 0x009)
#define OP_READ_CLOCK OP(0x05, 0x007)

#define EVT_CONN_COMPLETE 0x03
#define EVT_CONN_REQUEST 0x04
#define EVT_DISCONN_COMPLETE 0x05
#define EVT_REMOTE_NAME 0x07
#define EVT_CMD_COMPLETE 0x0E
#define EVT_CMD_STATUS 0x0F
#define EVT_PIN_CODE_REQ 0x16
#define EVT_LINK_KEY_REQ 0x17
#define EVT_IO_CAP_REQ 0x31
#define EVT_SYNC_TRAIN_COMPLETE 0x4F
#define EVT_SLAVE_PAGE_RESP_TIMEOUT 0x54
#define EVT_CSB_CHANNEL_MAP_CHANGE 0x55

#define LT_ADDR 0x01
#define CSB_PACKET_TYPE 0x330E           /* DM1 only: legacy glasses drop anything else */
#define CSB_INTERVAL 0x0080              /* 80 ms, what legacy glasses expect */
#define CSB_TIMEOUT 0x2000               /* slots without fresh data */
#define SYNC_TRAIN_INTERVAL 0x0080       /* 80 ms: legacy glasses scan ~100 ms per channel */
#define SYNC_TRAIN_TIMEOUT 0x0002EE00u   /* 120 s, as bluetooth.py, restarted when it ends */
#define PATH_LOSS_DB 127                 /* the glasses pick the nearest display under this */
#define PSM_3DSP 0x0021
#define L2CAP_CID_CONNECTIONLESS 0x0002
#define MAX_CONN 4

static int hci = -1;
static int log_fd = 2;
static int proto_fd = -1;
static int wd = -1;
static int pid1;                     /* the image's init: watchdog, reboots */

static int64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (int64_t)ts.tv_sec * NS_PER_S + ts.tv_nsec;
}

/* the watchdog, fed at most once a second */
static void kick(void)
{
	static int64_t last;
	int64_t now = now_ns();
	if (wd < 0 || now - last < NS_PER_S)
		return;
	last = now;
	if (write(wd, "k", 1) < 0) {
		/* a failing watchdog write reboots us, which is the point */
	}
}

static void logf_(const char *fmt, ...)
{
	char buf[256];
	va_list ap;
	int n = snprintf(buf, sizeof(buf), "bt3dd: ");
	va_start(ap, fmt);
	n += vsnprintf(buf + n, sizeof(buf) - n - 2, fmt, ap);
	va_end(ap);
	if (n > (int)sizeof(buf) - 3)
		n = sizeof(buf) - 3;
	buf[n++] = '\r';
	buf[n++] = '\n';
	if (write(log_fd, buf, n) < 0) {
		/* nowhere else to say it */
	}
}

/* lines to the host: status, glasses */
static void reply(const char *fmt, ...)
{
	char buf[1024];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
	va_end(ap);
	if (n > (int)sizeof(buf) - 2)
		n = sizeof(buf) - 2;
	buf[n++] = '\n';
	if (proto_fd >= 0 && write(proto_fd, buf, n) < 0) {
		/* the host is not listening; it asks again with '?' */
	}
}

/* ---- state ------------------------------------------------------------------ */

enum { MODE_OFF = 0, MODE_FREE = 1, MODE_SYNC = 2 };

static struct {
	int mode;
	uint32_t free_mhz;
	int32_t delay_us;
	int invert;
	int32_t open_after_us, open_us;
	int pairing;
	int grace_s;
	int discov_3d;                   /* V: discoverable during 3D, light inquiry scan */
} cfg = {
	.mode = MODE_SYNC,
	.free_mhz = 96000,
	.grace_s = GRACE_DEFAULT_S,
	.discov_3d = 1,
};

static struct hostclock host;
static struct btclock btc;

static struct {
	int valid;
	double vsync_mono, period_mono;  /* last vsync on our clock, one frame */
	char eye;                        /* 'L' or 'R' at that vsync */
} track;
static int64_t last_3d = -3600 * NS_PER_S;   /* when that vsync arrived */

static int bt_on;                    /* broadcast and train running */
static uint8_t last_msg[MSG_LEN];
static int64_t last_msg_at;
static int restart_train;            /* the controller ended the train */
static int scan_mode = -1, inq_light = -1;   /* what the controller has (-1: unknown) */
static uint8_t bdaddr[6];
static unsigned train_restarts, announcements;
static int64_t train_at;             /* when the train last (re)started */
/* how far the broadcast frame instant moved between two messages, against
 * where the previous one put it: what the glasses see of our corrections */
static struct {
	int have;
	double fsi, last, max;           /* fsi: the previous one, before the delay */
	unsigned big;                    /* moves over FSI_JUMP_LOG_US */
	int64_t logged_at;
} jump;
#define FSI_JUMP_LOG_US 100.0

static struct {
	int used;
	uint16_t handle;
	uint8_t addr[6];
	int want_name;
} conns[MAX_CONN];

/* ---- HCI transport ------------------------------------------------------------ */

static void handle_packet(const uint8_t *p, int len);

static int hci_send(uint16_t op, const void *params, uint8_t plen)
{
	uint8_t buf[260];
	buf[0] = 0x01;
	buf[1] = op & 0xff;
	buf[2] = op >> 8;
	buf[3] = plen;
	memcpy(buf + 4, params, plen);
	return write(hci, buf, 4 + plen) == 4 + plen ? 0 : -1;
}

/* Send a command and wait for its Command Complete / Status, handing every
 * other packet to handle_packet. Returns the HCI status (0 = success), or -1
 * when nothing came back; *ret gets the return parameters after the status. */
static int hci_cmd(uint16_t op, const void *params, uint8_t plen, uint8_t *ret, int *ret_len)
{
	if (hci_send(op, params, plen) < 0)
		return -1;
	int64_t deadline = now_ns() + 2 * NS_PER_S;
	for (;;) {
		int left = (int)((deadline - now_ns()) / 1000000);
		if (left <= 0)
			break;
		struct pollfd pfd = { hci, POLLIN, 0 };
		kick();
		if (poll(&pfd, 1, left) <= 0)
			continue;
		uint8_t b[1100];
		int n = read(hci, b, sizeof(b));
		if (n <= 0)
			return -1;
		if (b[0] == 0x04 && n >= 6 && b[1] == EVT_CMD_COMPLETE && get16(b + 4) == op) {
			int status = n > 6 ? b[6] : 0;
			if (ret && ret_len) {             /* callers' buffers hold 256 */
				*ret_len = n - 7 > 0 ? (n - 7 > 256 ? 256 : n - 7) : 0;
				memcpy(ret, b + 7, *ret_len);
			}
			return status;
		}
		if (b[0] == 0x04 && n >= 7 && b[1] == EVT_CMD_STATUS && get16(b + 5) == op)
			return b[3];
		handle_packet(b, n);
	}
	logf_("HCI 0x%04x: no answer", op);
	return -1;
}

static int cmd_ok(uint16_t op, const void *params, uint8_t plen)
{
	int st = hci_cmd(op, params, plen, NULL, NULL);
	if (st != 0)
		logf_("HCI 0x%04x: status 0x%02x", op, st & 0xff);
	return st;
}

static void addr_str(const uint8_t a[6], char out[18])
{
	snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", a[5], a[4], a[3], a[2], a[1], a[0]);
}

/* the commands that are an address and one byte: accept, reject, IO capability */
static void send_addr_byte(uint16_t op, const uint8_t addr[6], uint8_t byte)
{
	uint8_t p[7];
	memcpy(p, addr, 6);
	p[6] = byte;
	hci_send(op, p, 7);
}

/* ---- the controller, ours alone ------------------------------------------------ */

/* Bind the HCI user channel: exclusive, the kernel sends nothing itself. The
 * controller registers when hci_uart has loaded its patch; until then (or while
 * the kernel still has it up after setup) the bind fails and is retried. */
static int hci_open(void)
{
	for (;;) {
		int s = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
		if (s < 0) {
			logf_("no Bluetooth socket: %s", strerror(errno));
			kick();
			sleep(1);
			continue;
		}
		struct sockaddr_hci a = { AF_BLUETOOTH, 0, HCI_CHANNEL_USER };
		if (bind(s, (struct sockaddr *)&a, sizeof(a)) == 0)
			return s;
		int e = errno;
		if (e == EBUSY)
			ioctl(s, HCIDEVDOWN, 0);
		close(s);
		kick();
		usleep(300000);
	}
}

/* Inquiry scan duty: heavy (320 ms in 640 ms, interlaced: bluetooth.py's,
 * for glasses that search in short bursts) or light (11.25 ms in 1.28 s,
 * interlaced), so being discoverable during 3D leaves the radio to the
 * broadcast. */
static void set_inquiry_light(int light)
{
	if (light == inq_light)
		return;
	static const uint8_t heavy[4] = { 0x00, 0x04, 0x00, 0x02 }, lite[4] = { 0x00, 0x08, 0x12, 0x00 };
	if (cmd_ok(OP_WRITE_INQ_SCAN, light ? lite : heavy, 4) == 0)
		inq_light = light;
}

static void set_scan(int mode)
{
	if (mode == scan_mode)
		return;
	uint8_t m = (uint8_t)mode;
	if (cmd_ok(OP_WRITE_SCAN_ENABLE, &m, 1) == 0)
		scan_mode = mode;
}

static void csb_params(uint8_t p[11], int enable)
{
	p[0] = enable ? 1 : 0;
	p[1] = LT_ADDR;
	p[2] = 0;                                   /* no LPO: keep the precise clock */
	put16(p + 3, CSB_PACKET_TYPE);
	put16(p + 5, CSB_INTERVAL);
	put16(p + 7, CSB_INTERVAL);
	put16(p + 9, CSB_TIMEOUT);
}

/* The broadcast off and its LT_ADDR released; quiet at start-up, where
 * there is usually nothing to stop. */
static void csb_stop(int quiet)
{
	uint8_t p[11], lt = LT_ADDR;
	csb_params(p, 0);
	if (quiet) {
		hci_cmd(OP_SET_CSB, p, 11, NULL, NULL);
		hci_cmd(OP_DELETE_RESERVED_LT, &lt, 1, NULL, NULL);
	} else {
		cmd_ok(OP_SET_CSB, p, 11);
		cmd_ok(OP_DELETE_RESERVED_LT, &lt, 1);
	}
}

static int hci_init(void)
{
	uint8_t r[260];
	int rl = 0;
	if (cmd_ok(OP_RESET, NULL, 0))
		return -1;
	/* the kernel's own set-up (hci_init, 6.12, as recorded on this Pi), same
	 * order and values: then the controller is as bluetooth.py found it */
	uint8_t on = 1, eir_mode = 0x02;
	uint8_t accept[2] = { 0x00, 0x7d };          /* 20 s */
	uint8_t mask[8] = { 0xff, 0xff, 0xfb, 0xff, 0x07, 0xf8, 0xbf, 0x3d };
	uint8_t policy[2] = { 0x05, 0x00 };          /* role switch, sniff */
	uint8_t le_mask[8] = { 0x3f, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t le_host[2] = { 0x01, 0x00 };
	uint8_t del_keys[7] = { 0, 0, 0, 0, 0, 0, 0x01 };
	cmd_ok(OP_WRITE_SSP_MODE, &on, 1);
	cmd_ok(OP_WRITE_INQUIRY_MODE, &eir_mode, 1);
	cmd_ok(OP_WRITE_CONN_ACCEPT_TIMEOUT, accept, 2);
	cmd_ok(OP_SET_EVENT_MASK, mask, 8);
	cmd_ok(OP_WRITE_DEFAULT_LINK_POLICY, policy, 2);
	cmd_ok(OP_LE_SET_EVENT_MASK, le_mask, 8);
	cmd_ok(OP_LE_CLEAR_ACCEPT_LIST, NULL, 0);
	cmd_ok(OP_WRITE_LE_HOST_SUPPORTED, le_host, 2);
	hci_cmd(OP_DELETE_STORED_LINK_KEY, del_keys, 7, NULL, NULL);
	/* page 2: Synchronization Train Complete (0x4F, bit 15), Slave Page
	 * Response Timeout (0x54, bit 20), CSB Channel Map Change (0x55, bit 21) */
	uint8_t mask2[8] = { 0, 0x80, 0x30, 0, 0, 0, 0, 0 };
	cmd_ok(OP_SET_EVENT_MASK_2, mask2, 8);
	cmd_ok(OP_WRITE_SC_HOST_SUPPORT, &on, 1);

	if (hci_cmd(OP_READ_LOCAL_VERSION, NULL, 0, r, &rl) == 0 && rl >= 8) {
		unsigned rev = get16(r + 1), sub = get16(r + 6);
		/* btbcm's build number is the revision's low 12 bits: 0 = no patch */
		logf_("controller HCI %u, revision 0x%04x (build %u), LMP subversion 0x%04x%s",
		      r[0], rev, rev & 0xfff, sub, rev & 0xfff ? "" : " UNPATCHED");
	}
	if (hci_cmd(OP_READ_BDADDR, NULL, 0, r, &rl) == 0 && rl >= 6) {
		char s[18];
		memcpy(bdaddr, r, 6);
		addr_str(bdaddr, s);
		logf_("address %s", s);
	}
	uint8_t name[248] = "3D display";
	cmd_ok(OP_WRITE_LOCAL_NAME, name, sizeof(name));

	/* then bluetooth.py's Display.start(), in its order */
	csb_stop(1);                                 /* a run that died may have left it on */
	set_inquiry_light(0);
	uint8_t one = 1;
	cmd_ok(OP_WRITE_INQ_SCAN_TYPE, &one, 1);
	/* glasses page in short bursts: page scan half the time, interlaced */
	static const uint8_t page_scan[4] = { 0x00, 0x04, 0x00, 0x02 };
	cmd_ok(OP_WRITE_PAGE_SCAN, page_scan, 4);
	cmd_ok(OP_WRITE_PAGE_SCAN_TYPE, &one, 1);
	int tx = 4;
	if (hci_cmd(OP_READ_INQ_TX_POWER, NULL, 0, r, &rl) == 0 && rl >= 1)
		tx = (int8_t)r[0];
	tx = tx < 0 ? 0 : tx > 127 ? 127 : tx;     /* legacy glasses read it unsigned */
	/* 3D Information (association notification, battery reports, report at
	 * start-up), the legacy Broadcom record (first manufacturer entry, "3D
	 * capable"), TX power */
	static const uint8_t items[] = {
		0x03, 0x3D, 0x07, PATH_LOSS_DB,
		0x06, 0xFF, 0x0F, 0x00, 0x00, 0x01, PATH_LOSS_DB,
		0x02, 0x0A, 0x00,
	};
	uint8_t eir[241] = { 0x01 };                /* FEC required: DM packets */
	memcpy(eir + 1, items, sizeof(items));
	eir[1 + sizeof(items) - 1] = (uint8_t)tx;
	/* every channel usable, or the train can announce an empty channel map */
	uint8_t afh[10];
	memset(afh, 0xff, 9);
	afh[9] = 0x7f;
	cmd_ok(OP_SET_AFH_CLASS, afh, 10);
	uint8_t cls[3] = { 0x3C, 0x04, 0x08 };     /* 0x08043C: A/V, display and loudspeaker */
	cmd_ok(OP_WRITE_CLASS, cls, 3);
	cmd_ok(OP_WRITE_EIR, eir, sizeof(eir));
	set_scan(0x00);
	return 0;
}

static void send_message(const uint8_t m[MSG_LEN])
{
	uint8_t p[3 + MSG_LEN] = { LT_ADDR, 0x03, MSG_LEN };   /* 0x03: no fragmentation */
	memcpy(p + 3, m, MSG_LEN);
	if (cmd_ok(OP_SET_CSB_DATA, p, sizeof(p)) == 0) {
		memcpy(last_msg, m, MSG_LEN);
		last_msg_at = now_ns();
	}
}

/* The broadcast and its train on. Set_CSB is sent once: re-sending it while
 * it runs ends the sync train on this controller, and glasses then never
 * hold sync. The train is restarted only when the controller says it ended. */
static int broadcast_on(const uint8_t first[MSG_LEN])
{
	uint8_t lt = LT_ADDR, p[11], st[9];
	cmd_ok(OP_SET_RESERVED_LT, &lt, 1);
	send_message(first);
	csb_params(p, 1);
	if (cmd_ok(OP_SET_CSB, p, 11))
		return -1;
	put16(st, SYNC_TRAIN_INTERVAL);
	put16(st + 2, SYNC_TRAIN_INTERVAL);
	put32(st + 4, SYNC_TRAIN_TIMEOUT);
	st[8] = 0x00;                               /* service data = display id: home */
	cmd_ok(OP_WRITE_SYNC_TRAIN, st, 9);
	cmd_ok(OP_START_SYNC_TRAIN, NULL, 0);
	train_at = now_ns();
	restart_train = 0;                          /* any older end event is moot */
	bt_on = 1;
	logf_("broadcast on");
	return 0;
}

static void broadcast_off(void)
{
	csb_stop(0);
	restart_train = 0;
	bt_on = 0;
	memset(last_msg, 0, sizeof(last_msg));
	logf_("broadcast off");
}

/* ---- what arrives from the controller ------------------------------------------ */

static int conn_find(uint16_t handle)
{
	for (int i = 0; i < MAX_CONN; i++)
		if (conns[i].used && conns[i].handle == handle)
			return i;
	return -1;
}

static void on_acl(const uint8_t *p, int len)
{
	/* a 3DG Connection Announcement: connectionless L2CAP on PSM 0x0021,
	 * opcode 0, flags (bit 0 associated, bit 1 battery request), battery % */
	if (len < 4)
		return;
	uint16_t hdr = get16(p), dlen = get16(p + 2);
	if (((hdr >> 12) & 0x3) == 0x1 || dlen < 9 || len < 4 + 9)
		return;
	const uint8_t *d = p + 4;
	uint16_t l2len = get16(d), cid = get16(d + 2), psm = get16(d + 4);
	if (cid != L2CAP_CID_CONNECTIONLESS || psm != PSM_3DSP || l2len < 5 || d[6] != 0)
		return;
	int i = conn_find(hdr & 0x0fff);
	if (i < 0)
		return;
	char a[18];
	addr_str(conns[i].addr, a);
	int batt = d[8] <= 100 ? d[8] : -1;
	announcements++;
	reply("G %s %d %d %d", a, d[7] & 1, (d[7] >> 1) & 1, batt);
	logf_("glasses %s associated %d battery %d%%", a, d[7] & 1, batt);
	conns[i].want_name = 1;
}

static void handle_packet(const uint8_t *b, int n)
{
	if (b[0] == 0x02) {
		on_acl(b + 1, n - 1);
		return;
	}
	if (b[0] != 0x04 || n < 3)
		return;
	uint8_t code = b[1];
	const uint8_t *e = b + 3;
	int elen = n - 3;
	switch (code) {
	case EVT_SYNC_TRAIN_COMPLETE:
	case EVT_SLAVE_PAGE_RESP_TIMEOUT:
	case EVT_CSB_CHANNEL_MAP_CHANGE:
		restart_train = 1;
		break;
	case EVT_CONN_REQUEST:
		if (elen >= 10) {
			if (e[9] == 0x01)               /* ACL: the glasses' announcement */
				send_addr_byte(OP_ACCEPT_CONN, e, 0x01);     /* stay peripheral */
			else
				send_addr_byte(OP_REJECT_CONN, e, 0x0F);     /* unacceptable BD_ADDR */
		}
		break;
	case EVT_CONN_COMPLETE:
		if (elen >= 9 && e[0] == 0) {
			for (int i = 0; i < MAX_CONN; i++)
				if (!conns[i].used) {
					conns[i].used = 1;
					conns[i].handle = get16(e + 1) & 0x0fff;
					memcpy(conns[i].addr, e + 3, 6);
					conns[i].want_name = 0;
					break;
				}
		}
		break;
	case EVT_DISCONN_COMPLETE:
		if (elen >= 3 && e[0] == 0) {
			int i = conn_find(get16(e + 1) & 0x0fff);
			if (i >= 0)
				conns[i].used = 0;
		}
		break;
	case EVT_REMOTE_NAME:
		if (elen >= 7 && e[0] == 0) {
			char a[18], name[64];
			addr_str(e + 1, a);
			int k = 0;
			for (int i = 7; i < elen && e[i] && k < (int)sizeof(name) - 1; i++)
				name[k++] = (e[i] >= 0x20 && e[i] < 0x7f) ? e[i] : '?';
			name[k] = 0;
			reply("N %s %s", a, name);
		}
		break;
	case EVT_LINK_KEY_REQ:
		if (elen >= 6)
			hci_send(OP_LINK_KEY_NEG, e, 6);
		break;
	case EVT_PIN_CODE_REQ:
		if (elen >= 6)
			hci_send(OP_PIN_CODE_NEG, e, 6);
		break;
	case EVT_IO_CAP_REQ:
		if (elen >= 6)
			send_addr_byte(OP_IO_CAP_NEG, e, 0x18);          /* pairing not allowed */
		break;
	default:
		break;
	}
}

static void ask_names(void)
{
	for (int i = 0; i < MAX_CONN; i++) {
		if (!conns[i].used || !conns[i].want_name)
			continue;
		conns[i].want_name = 0;
		uint8_t p[10] = { 0 };
		memcpy(p, conns[i].addr, 6);
		p[6] = 0x01;                            /* page scan repetition R1 */
		hci_send(OP_REMOTE_NAME, p, 10);
	}
}

static void sample_clock(void)
{
	uint8_t p[3] = { 0, 0, 0 }, r[260];     /* handle 0, native clock */
	int rl = 0;
	int64_t before = now_ns();
	int st = hci_cmd(OP_READ_CLOCK, p, 3, r, &rl);
	int64_t after = now_ns();
	if (st == 0 && rl >= 6)
		bt_add(&btc, before, get32(r + 2), after);
}

/* ---- the message ---------------------------------------------------------------- */

static int in_3d(int64_t now)
{
	if (cfg.mode == MODE_FREE)
		return 1;
	return cfg.mode == MODE_SYNC && track.valid && now - last_3d < COAST_NS;
}

static int wanted(int64_t now)
{
	if (cfg.mode == MODE_OFF)
		return cfg.pairing;
	if (cfg.mode == MODE_FREE || cfg.pairing)
		return 1;
	return now - last_3d < cfg.grace_s * NS_PER_S;
}

static void make_message(uint8_t m[MSG_LEN], int64_t now, int show_3d)
{
	if (!show_3d || !bt_ready(&btc)) {
		jump.have = 0;
		pack_2d(m);
		return;
	}
	struct btline line = bt_line(&btc);
	double pair_us, fsi;
	if (cfg.mode == MODE_FREE) {
		/* one steady rhythm on whole periods of the clock: a new instant
		 * every message would move the phase */
		pair_us = 2e6 * 1000.0 / cfg.free_mhz;
		fsi = floor(bt_at(&line, (double)now) / pair_us) * pair_us;
	} else {
		double left = track.vsync_mono;
		if (track.eye == 'R')
			left -= track.period_mono;      /* the left frame just before it */
		pair_us = 2.0 * track.period_mono * line.rate;
		fsi = bt_at(&line, left);
		if (jump.have) {
			double j = remainder(fsi - jump.fsi, pair_us);
			jump.last = j;
			if (fabs(j) > fabs(jump.max))
				jump.max = j;
			if (fabs(j) > FSI_JUMP_LOG_US) {
				jump.big++;
				if (now - jump.logged_at >= NS_PER_S) {
					logf_("frame instant moved %+.0f us", j);
					jump.logged_at = now;
				}
			}
		}
		jump.fsi = fsi;
		jump.have = 1;
	}
	fsi += cfg.delay_us + (cfg.invert ? pair_us / 2 : 0);
	double off[4];
	window_offsets(pair_us, cfg.open_after_us, cfg.open_us, off);
	pack_message(m, fsi, pair_us, off);
}

/* ---- the host protocol --------------------------------------------------------- */

static int cmp_double(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return x < y ? -1 : x > y;
}

static void status(void)
{
	char a[18];
	double rate = 0, mx = 0, cy = 0;
	int64_t now = now_ns();
	if (bt_ready(&btc))
		rate = bt_fit(&btc, &mx, &cy);
	addr_str(bdaddr, a);
	/* the offsets the timing stands on: host us -> our ns (the USB floor
	 * included), our us -> the Bluetooth clock, and the bracket widths the
	 * clock fit uses (its tightest half) */
	double host_off_us = ((double)host.d_ref + host.floor_ns) / 1000.0;
	double bt_off_us = rate ? cy + rate * ((double)now - mx) - (double)now / 1000.0 : 0;
	double rtt[BT_KEEP];
	int n = btc.n;
	memcpy(rtt, btc.rtt, sizeof(double) * n);
	qsort(rtt, n, sizeof(double), cmp_double);
	int used = n / 2 < BT_READY ? n : n / 2;
	reply("pi-bt3dsp " VERSION " build " BT3DD_BUILD " mode %d delay_us %ld invert %d free_mhz %lu"
	      " period_us %.3f rate_changes %u skew_ppm %.1f fit_windows %d msgs %u resets %u"
	      " last_resid_us %.0f running %d bt %d address %s open_after_us %ld open_us %ld"
	      " pairing %d discov_3d %d grace_s %d bt_ppm %.1f clock_samples %u train_restarts %u glasses %u"
	      " host_off_us %.1f floor_us %.1f bt_off_us %.1f bt_rtt_min_us %.0f bt_rtt_used_us %.0f"
	      " fsi_jump_last_us %.0f fsi_jump_max_us %.0f fsi_jumps %u reset_why %s reset_age_s %.0f"
	      " train_age_s %.0f",
	      cfg.mode, (long)cfg.delay_us, cfg.invert, (unsigned long)cfg.free_mhz,
	      host.period_us, host.rate_changes, host.skew * 1e6, host.wn, host.msgs, host.resets,
	      host.resid_ns / 1000.0, in_3d(now_ns()), bt_on, a, (long)cfg.open_after_us,
	      (long)cfg.open_us, cfg.pairing, cfg.discov_3d, cfg.grace_s,
	      rate ? (rate * 1000.0 - 1.0) * 1e6 : 0.0, btc.samples, train_restarts, announcements,
	      host_off_us, host.floor_ns / 1000.0, bt_off_us, n ? rtt[0] / 1000.0 : 0.0,
	      n ? rtt[used - 1] / 1000.0 : 0.0, jump.last, jump.max, jump.big,
	      host_reset_names[host.reset_why],
	      host.reset_at ? (double)(now - host.reset_at) / NS_PER_S : -1.0,
	      train_at ? (double)(now - train_at) / NS_PER_S : -1.0);
	jump.max = 0;                    /* the largest since the last status */
}

static void on_sync(char *p, int64_t arrival)
{
	char *e;
	uint32_t vsync = strtoul(p, &e, 10);
	uint32_t us = strtoul(e, &e, 10);
	while (*e == ' ')
		e++;
	char eye = *e;
	if (eye != 'L' && eye != 'R') {
		track.valid = 0;                        /* '-': no 3D at this vsync */
		return;
	}
	double vm, pm;
	unsigned resets = host.resets;
	int ok = host_on_sync(&host, vsync, us, arrival, &vm, &pm);
	if (host.resets != resets)
		logf_("clock fit started over: %s", host_reset_names[host.reset_why]);
	if (ok) {
		track.valid = 1;
		track.vsync_mono = vm;
		track.period_mono = pm;
		track.eye = eye;
		last_3d = arrival;
	}
}

static void handle_line(char *line, int64_t arrival)
{
	char *p = line + 1;
	switch (line[0]) {
	case 'S':
		if (cfg.mode == MODE_SYNC)
			on_sync(p, arrival);
		break;
	case 'M': {
		int m = atoi(p);
		cfg.mode = m < 0 ? 0 : m > 2 ? 2 : m;
		host_reset(&host, HOST_RESET_MODE, arrival);
		track.valid = 0;
		break;
	}
	case 'F': {
		unsigned long f = strtoul(p, NULL, 10);
		cfg.free_mhz = f < 1000 ? 1000 : f;
		break;
	}
	case 'D':
		cfg.delay_us = strtol(p, NULL, 10);
		break;
	case 'I':
		cfg.invert = atoi(p) ? 1 : 0;
		break;
	case 'O': {                                 /* lens window: open after, open for */
		char *e;
		cfg.open_after_us = strtol(p, &e, 10);
		cfg.open_us = strtol(e, NULL, 10);
		if (cfg.open_after_us < 0)
			cfg.open_after_us = 0;
		if (cfg.open_us < 0)
			cfg.open_us = 0;
		break;
	}
	case 'A':
		cfg.pairing = atoi(p) ? 1 : 0;
		break;
	case 'V':
		cfg.discov_3d = atoi(p) ? 1 : 0;
		break;
	case 'G': {
		int g = atoi(p);
		cfg.grace_s = g < 1 ? 1 : g > 3600 ? 3600 : g;
		break;
	}
	case 'B':                                   /* back to the boot ROM: the host boots the Pi again */
		if (pid1) {
			logf_("reboot asked by the host");
			sync();
			reboot(RB_AUTOBOOT);
		}
		break;
	case '?':
		status();
		break;
	default:
		break;
	}
}

/* ---- PID 1 ---------------------------------------------------------------------- */

static void setup_system(void)
{
	mount("devtmpfs", "/dev", "devtmpfs", 0, NULL);
	mount("proc", "/proc", "proc", 0, NULL);
	mount("sysfs", "/sys", "sysfs", 0, NULL);
	signal(SIGPIPE, SIG_IGN);
	signal(SIGHUP, SIG_IGN);
}

static int open_tty(const char *path)
{
	int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0)
		return -1;
	struct termios t;
	if (tcgetattr(fd, &t) == 0) {
		cfmakeraw(&t);
		t.c_cflag |= CLOCAL;
		tcsetattr(fd, TCSANOW, &t);
	}
	return fd;
}

/* ---- the green LED: off without a host, on with one, blinking during 3D ---- */

static const char *led_dir;
static int led_state = -1;

static void led_write(const char *file, const char *value)
{
	char path[96];
	snprintf(path, sizeof(path), "%s/%s", led_dir, file);
	int fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return;
	if (write(fd, value, strlen(value)) < 0) {
		/* no LED control: the LED stays as the firmware left it */
	}
	close(fd);
}

static void led_init(void)
{
	static const char *dirs[] = { "/sys/class/leds/ACT", "/sys/class/leds/led0" };
	for (unsigned i = 0; i < sizeof(dirs) / sizeof(dirs[0]); i++)
		if (access(dirs[i], F_OK) == 0) {
			led_dir = dirs[i];
			break;
		}
	if (led_dir)
		led_write("trigger", "none");
}

/* 0 off, 1 on, 2 blink (the kernel's timer trigger, 500 ms on and off) */
static void led_set(int state)
{
	if (!led_dir || state == led_state)
		return;
	if (state == 2) {
		led_write("trigger", "timer");
		led_write("delay_on", "500");
		led_write("delay_off", "500");
	} else {
		led_write("trigger", "none");
		led_write("brightness", state ? "1" : "0");
	}
	led_state = state;
}

/* The host is there: the gadget configured, or suspended (the host idling the
 * port with autosuspend; counting that as gone rebooted the Pi once). */
static int usb_link_up(void)
{
	char s[32] = "";
	int fd = open(UDC_STATE, O_RDONLY | O_CLOEXEC);
	if (fd < 0)
		return 1;                               /* no state to go by: never reboot on it */
	int n = read(fd, s, sizeof(s) - 1);
	close(fd);
	return n > 0 && (strncmp(s, "configured", 10) == 0 || strncmp(s, "suspended", 9) == 0);
}

int main(void)
{
	pid1 = getpid() == 1;
	if (pid1)
		setup_system();
	srand((unsigned)now_ns());

	/* only as PID 1: run by hand (the debug image), nothing restarts it, and
	 * stopping it would leave the watchdog to reboot the Pi */
	wd = pid1 ? open("/dev/watchdog", O_WRONLY | O_CLOEXEC) : -1;
	if (wd >= 0) {
		int t = WATCHDOG_S;
		ioctl(wd, WDIOC_SETTIMEOUT, &t);
	}
	for (int i = 0; i < 50 && proto_fd < 0; i++) {
		proto_fd = open_tty(PROTO_TTY);
		if (proto_fd < 0)
			usleep(100000);
	}
	int lf = open_tty(LOG_TTY);
	if (lf >= 0)
		log_fd = lf;
	logf_("pi-bt3dsp " VERSION " build " BT3DD_BUILD " starting");
	led_init();

	hci = hci_open();
	if (hci_init())
		logf_("controller set-up incomplete");
	logf_("ready");

	char line[96];
	int line_len = 0;
	int64_t line_t = 0;
	int64_t next_tick = now_ns(), next_check = 0, usb_down_since = 0;

	for (;;) {
		int64_t now = now_ns();
		int timeout = (int)((next_tick - now) / 1000000);
		if (timeout < 0)
			timeout = 0;
		struct pollfd pf[2] = { { proto_fd, POLLIN, 0 }, { hci, POLLIN, 0 } };
		poll(pf, 2, timeout);

		if (pf[0].revents & POLLIN) {
			char b[256];
			int n = read(proto_fd, b, sizeof(b));
			if (n <= 0)
				pf[0].revents |= POLLHUP;
			int64_t t = now_ns();
			for (int i = 0; i < n; i++) {
				if (line_len == 0)
					line_t = t;                 /* arrival of the line's first byte */
				if (b[i] == '\n' || b[i] == '\r') {
					if (line_len) {
						line[line_len] = 0;
						handle_line(line, line_t);
					}
					line_len = 0;
				} else if (line_len < (int)sizeof(line) - 1) {
					line[line_len++] = b[i];
				}
			}
		}
		if (proto_fd >= 0 && pf[0].revents & (POLLHUP | POLLERR)) {
			/* USB went away: reopened on the next tick, so a dead port
			 * never spins this loop */
			close(proto_fd);
			proto_fd = -1;
		}
		if (pf[1].revents & POLLIN) {
			uint8_t b[1100];
			int n = read(hci, b, sizeof(b));
			if (n > 0)
				handle_packet(b, n);
		}

		now = now_ns();
		if (now < next_tick)
			continue;
		int want = wanted(now), show_3d = in_3d(now);
		/* dithered, so Read_Clock samples land at every phase of CLKN's tick;
		 * with the radio off, once a second is enough (less power) */
		next_tick = now + (bt_on || want ? TICK_NS + rand() % TICK_DITHER_NS : IDLE_TICK_NS);

		kick();
		if (proto_fd < 0)
			proto_fd = open_tty(PROTO_TTY);
		int link = usb_link_up();
		led_set(!link ? 0 : bt_on && show_3d ? 2 : 1);
		if (now >= next_check) {
			next_check = now + NS_PER_S;
			if (link) {
				usb_down_since = 0;
			} else if (!usb_down_since) {
				usb_down_since = now;
			} else if (pid1 && now - usb_down_since > USB_DOWN_REBOOT_S * NS_PER_S) {
				logf_("USB link down %d s: rebooting to the boot ROM", USB_DOWN_REBOOT_S);
				sync();
				reboot(RB_AUTOBOOT);
			}
		}

		if (bt_on && restart_train) {
			/* the train ended (or a pair paged us, or the channel map
			 * changed): the train alone again, as bluetooth.py does. Never
			 * Set_CSB while it runs: that ends the train and re-anchors the
			 * broadcast, and every synced pair loses it. */
			restart_train = 0;
			train_restarts++;
			cmd_ok(OP_START_SYNC_TRAIN, NULL, 0);
			train_at = now;
			logf_("sync train restarted (%u)", train_restarts);
		}
		if (want && !bt_on) {
			uint8_t m[MSG_LEN];
			pack_2d(m);
			broadcast_on(m);
		} else if (!want && bt_on) {
			broadcast_off();
		}
		/* page scan while on, for glasses that page to find the train, as
		 * bluetooth.py; discoverable while pairing (A 1), and with V 1 during
		 * 3D too, at the light inquiry duty */
		set_inquiry_light(cfg.discov_3d && !cfg.pairing);
		set_scan(bt_on ? (cfg.pairing || (cfg.discov_3d && show_3d) ? 0x03 : 0x02) : 0x00);
		if (!bt_on)
			continue;
		sample_clock();
		now = now_ns();                         /* the message is for now, after Read_Clock */
		uint8_t m[MSG_LEN];
		make_message(m, now, in_3d(now));
		if (memcmp(m, last_msg, MSG_LEN) || now - last_msg_at >= RESEND_NS)
			send_message(m);
		ask_names();
	}
}
