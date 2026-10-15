/* bt3dd timing core: the host's vsync times onto the Pi's clock, the Pi's
 * clock onto the Bluetooth native clock, and the 3DSP broadcast message.
 * No I/O, so the host test (test_sync.c) runs exactly this code.
 *
 * Clocks: host_us is the host's microsecond clock (uint32, wraps), mono is
 * CLOCK_MONOTONIC_RAW on the Pi in ns, clk is the controller's native clock
 * in us (CLKN ticks are 312.5 us and wrap at 2^28). */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#define NS_PER_S 1000000000LL

/* ---- host clock -> mono ------------------------------------------------- */

#define HOST_WINDOWS 8                 /* skew fit over the last 8 one-second minima */
/* Another rate (VRR between films) shows as the last HOST_CHECK_FRAMES
 * averaging HOST_RATE_JUMP away from the span's period. Single steps never
 * decide it: the box stamps its vsyncs with +-120 us of jitter, a third of
 * single steps are 0.5 % off. Averaged over 128 frames the jitter is ~200 ppm;
 * 23.976 against 24 fps is 1000 ppm. */
#define HOST_CHECK_FRAMES 128
#define HOST_RATE_JUMP 0.0005
/* The vsyncs' line is fitted with this memory, in frames (about 10 s): long
 * enough to average the stamps' jitter out, short enough to follow the box's
 * clock while NTP slews it (+-1.5 ms over minutes, measured), which a line over
 * the whole film cannot. The picture runs on the video clock, the stamps on the
 * slewed one. */
#define HOST_SPAN_TAU_FRAMES 960.0
#define HOST_SPAN_RECENTRE 16384       /* frames: the line's origin moves up to here */
#define HOST_MAX_COAST_FRAMES 512      /* a silent 2 s at up to 240 Hz */
#define HOST_MAX_COAST_US 2100000u
#define HOST_LOST_NS (2 * NS_PER_S)          /* silent this long: the fit starts over */

/* d = arrival - host time. Its floor is the fixed part of the USB path; the
 * skew between the two clocks makes the floor drift, so it is fitted over the
 * last HOST_WINDOWS one-second minima. The vsync times and the frame period
 * come from the host and are exact. */
struct hostclock {
	int have_last;
	uint32_t last_vsync, last_us;
	int64_t vsync, us;             /* unwrapped */
	int64_t last_arrival;
	int64_t d_ref;                 /* d at the first message since a reset */
	int win_has;
	int64_t win_start, win_min, win_min_t;
	int64_t wt[HOST_WINDOWS], wm[HOST_WINDOWS];
	int wn, wh;                    /* windows fitted (the fit exists once wn > 0) */
	double skew, fit_m;            /* floor drift (ns per ns), floor at fit_t */
	int64_t fit_t;
	int64_t span_vsync, span_us;   /* the span starts here (once period_us > 0) */
	double rn, rsx, rsy, rsxx, rsxy; /* least squares of the span's stamps against vsync */
	double phase_us;               /* the line's value at span_vsync, relative to span_us */
	int64_t check_vsync, check_us; /* the window checked against the span starts here */
	double period_us;              /* exact frame period, host time: the line's slope */
	unsigned msgs, resets, rate_changes;
	double resid_ns;
	double floor_ns;               /* the floor in use at the last message, relative to d_ref */
	int reset_why;                 /* why the fit last started over (HOST_RESET_*) */
	int64_t reset_at;              /* and when, mono ns */
};

enum { HOST_RESET_NONE, HOST_RESET_MODE, HOST_RESET_REPEAT, HOST_RESET_FRAMES, HOST_RESET_TIME,
       HOST_RESET_SILENT };
static const char *const host_reset_names[] = { "none", "mode", "repeat", "frames", "time", "silent" };

static inline void host_reset(struct hostclock *h, int why, int64_t at)
{
	unsigned msgs = h->msgs, resets = h->resets + 1, rc = h->rate_changes;
	memset(h, 0, sizeof(*h));
	h->msgs = msgs;
	h->resets = resets;
	h->rate_changes = rc;
	h->reset_why = why;
	h->reset_at = at;
}

static inline void host_fit(struct hostclock *h)
{
	int first = (h->wh + HOST_WINDOWS - h->wn) % HOST_WINDOWS;
	int last = (h->wh + HOST_WINDOWS - 1) % HOST_WINDOWS;
	if (h->wn < 2) {
		h->fit_t = h->wt[last];
		h->fit_m = (double)h->wm[last];
		h->skew = 0;
		return;
	}
	int64_t t0 = h->wt[first];
	double sx = 0, sy = 0, sxx = 0, sxy = 0, n = h->wn;
	for (int k = 0; k < h->wn; k++) {
		int i = (first + k) % HOST_WINDOWS;
		double x = (double)(h->wt[i] - t0), y = (double)h->wm[i];
		sx += x; sy += y; sxx += x * x; sxy += x * y;
	}
	double den = n * sxx - sx * sx;
	h->skew = den != 0 ? (n * sxy - sx * sy) / den : 0;
	h->fit_t = h->wt[last];
	h->fit_m = (sy - h->skew * sx) / n + h->skew * (double)(h->wt[last] - t0);
}

/* Where the span's line puts a vsync, in host us: every stamp's jitter
 * averaged out, in its slope (the period) and its phase. */
static inline double host_model_us(const struct hostclock *h, int64_t vsync)
{
	return (double)h->span_us + h->phase_us + (double)(vsync - h->span_vsync) * h->period_us;
}

static inline void host_span_start(struct hostclock *h, int64_t vsync, int64_t us)
{
	h->span_vsync = h->check_vsync = vsync;
	h->span_us = h->check_us = us;
	h->rn = h->rsx = h->rsy = h->rsxx = h->rsxy = 0;
	h->phase_us = 0;
}

static inline void host_span_add_point(struct hostclock *h, double x, double y)
{
	h->rn += 1;
	h->rsx += x;
	h->rsy += y;
	h->rsxx += x * x;
	h->rsxy += x * y;
}

/* Move the line's origin to the current vsync, the sums with it, so its
 * numbers stay small however long the film. */
static inline void host_span_recentre(struct hostclock *h)
{
	double dx = (double)(h->vsync - h->span_vsync), dy = (double)(h->us - h->span_us);
	double sx = h->rsx, sy = h->rsy;
	h->rsx = sx - h->rn * dx;
	h->rsy = sy - h->rn * dy;
	h->rsxx += -2 * dx * sx + h->rn * dx * dx;
	h->rsxy += -dx * sy - dy * sx + h->rn * dx * dy;
	h->span_vsync = h->vsync;
	h->span_us = h->us;
}

static inline void host_span_add(struct hostclock *h)
{
	if (h->vsync - h->span_vsync >= HOST_SPAN_RECENTRE)
		host_span_recentre(h);
	double x = (double)(h->vsync - h->span_vsync), y = (double)(h->us - h->span_us);
	const double keep = 1.0 - 1.0 / HOST_SPAN_TAU_FRAMES;
	h->rn *= keep;
	h->rsx *= keep;
	h->rsy *= keep;
	h->rsxx *= keep;
	h->rsxy *= keep;
	h->rn += 1;
	h->rsx += x;
	h->rsy += y;
	h->rsxx += x * x;
	h->rsxy += x * y;
	double den = h->rn * h->rsxx - h->rsx * h->rsx;
	if (h->rn >= 2 && den > 0) {
		h->period_us = (h->rn * h->rsxy - h->rsx * h->rsy) / den;
		h->phase_us = (h->rsy - h->period_us * h->rsx) / h->rn;
	}
}

static inline void host_period(struct hostclock *h, uint32_t dv, uint32_t du)
{
	if (h->period_us <= 0 && h->rn == 0) {   /* the first span starts at the last message */
		host_span_start(h, h->vsync - dv, h->us - du);
		host_span_add_point(h, 0, 0);
	}
	int64_t cdv = h->vsync - h->check_vsync;
	if (h->period_us > 0 && cdv >= HOST_CHECK_FRAMES) {
		double recent = (double)(h->us - h->check_us) / (double)cdv;
		h->check_vsync = h->vsync;
		h->check_us = h->us;
		if (fabs(recent - h->period_us) > h->period_us * HOST_RATE_JUMP) {
			/* another rate: a new line from this vsync on (the window may
			 * hold both rates), the window's rate meanwhile */
			host_span_start(h, h->vsync, h->us);
			host_span_add_point(h, 0, 0);
			h->period_us = recent;
			h->rate_changes++;
			return;
		}
	}
	host_span_add(h);
}

/* One sync message that arrived at mono 'arrival'. True once *vsync_mono (the
 * vsync on the Pi's clock) and *period_mono (one frame, Pi ns) are known. */
static inline int host_on_sync(struct hostclock *h, uint32_t vsync, uint32_t us,
                               int64_t arrival, double *vsync_mono, double *period_mono)
{
	h->msgs++;
	int fresh = !h->have_last;                  /* the first message since a reset */
	if (!fresh) {
		uint32_t dv = vsync - h->last_vsync, du = us - h->last_us;
		int why = dv == 0 ? HOST_RESET_REPEAT
		        : dv > HOST_MAX_COAST_FRAMES ? HOST_RESET_FRAMES
		        : du > HOST_MAX_COAST_US ? HOST_RESET_TIME
		        : arrival - h->last_arrival > HOST_LOST_NS ? HOST_RESET_SILENT : HOST_RESET_NONE;
		if (why != HOST_RESET_NONE) {
			host_reset(h, why, arrival); /* host restarted, clock jumped, long gap */
			fresh = 1;
		} else {
			h->vsync += dv;
			h->us += du;
			host_period(h, dv, du);
		}
	}
	if (fresh) {
		h->vsync = vsync;
		h->us = us;
	}
	h->last_vsync = vsync;
	h->last_us = us;
	h->last_arrival = arrival;
	h->have_last = 1;

	/* the clock fit takes the raw stamps (its floor hardly moves with their
	 * symmetric jitter, and it must not depend on a period that may just
	 * have changed); the frame instants take the span's line */
	int64_t d = arrival - h->us * 1000;
	if (fresh) {
		h->d_ref = d;
		h->win_start = arrival;
	}
	int64_t rel = d - h->d_ref;
	if (!h->win_has || rel < h->win_min) {
		h->win_min = rel;
		h->win_min_t = arrival;   /* stamped where it was seen: the floor moves in a window */
	}
	h->win_has = 1;
	if (arrival - h->win_start >= NS_PER_S) {
		h->wt[h->wh] = h->win_min_t;
		h->wm[h->wh] = h->win_min;
		h->wh = (h->wh + 1) % HOST_WINDOWS;
		if (h->wn < HOST_WINDOWS)
			h->wn++;
		h->win_has = 0;
		h->win_start = arrival;
		host_fit(h);
	}
	double floor_now = h->wn > 0 ? h->fit_m + h->skew * (double)(arrival - h->fit_t)
	                               : (double)h->win_min;
	h->resid_ns = (double)rel - floor_now;
	h->floor_ns = floor_now;
	if (h->period_us <= 0)
		return 0;
	double vsync_us = h->rn >= 2 ? host_model_us(h, h->vsync) : (double)h->us;
	*vsync_mono = (double)h->d_ref + floor_now + vsync_us * 1000.0;
	*period_mono = h->period_us * 1000.0 * (1.0 + h->skew);
	return 1;
}

/* ---- mono -> Bluetooth native clock ------------------------------------ */

#define CLKN_US 312.5
#define CLKN_WRAP (1u << 28)
#define CLK_WRAP_US ((double)CLKN_WRAP * CLKN_US)
#define BT_KEEP 256                     /* samples kept; the tightest half is fitted */
#define BT_READY 8

/* Read_Clock bracketed by two mono reads. CLKN only counts whole 312.5 us
 * ticks, so a sample says the clock was in [v, v + 1) ticks somewhere in the
 * bracket; samples taken at random phases (the caller dithers them) average
 * that out in the fit. A constant bias (UART latency asymmetry) is harmless:
 * the delay setting absorbs it. */
struct btclock {
	int n, head;
	double mono[BT_KEEP], clk[BT_KEEP], rtt[BT_KEEP];
	uint32_t last_raw;
	int64_t wraps;
	unsigned samples;
};

static inline void bt_reset(struct btclock *c)
{
	memset(c, 0, sizeof(*c));
}

static inline void bt_add(struct btclock *c, int64_t before, uint32_t clkn, int64_t after)
{
	clkn &= CLKN_WRAP - 1;
	if (c->samples && clkn < c->last_raw)
		c->wraps++;
	c->last_raw = clkn;
	c->mono[c->head] = (double)(before + after) / 2.0;
	c->clk[c->head] = ((double)clkn + (double)c->wraps * CLKN_WRAP) * CLKN_US + CLKN_US / 2;
	c->rtt[c->head] = (double)(after - before);
	c->head = (c->head + 1) % BT_KEEP;
	if (c->n < BT_KEEP)
		c->n++;
	c->samples++;
}

static inline int bt_ready(const struct btclock *c)
{
	return c->n >= BT_READY;
}

/* Least squares clk = a + b * mono over the tightest half of the samples.
 * Returns b (clock us per mono ns, about 1e-3) and the line's centre. */
static inline double bt_fit(const struct btclock *c, double *mx_out, double *cy_out)
{
	int idx[BT_KEEP], n = c->n, use = n / 2 < BT_READY ? n : n / 2;
	for (int i = 0; i < n; i++)
		idx[i] = i;
	for (int i = 1; i < n; i++) {          /* insertion sort by bracket width */
		int k = idx[i], j = i - 1;
		while (j >= 0 && c->rtt[idx[j]] > c->rtt[k]) {
			idx[j + 1] = idx[j];
			j--;
		}
		idx[j + 1] = k;
	}
	double mx = 0, cy = 0;
	for (int i = 0; i < use; i++) {
		mx += c->mono[idx[i]];
		cy += c->clk[idx[i]];
	}
	mx /= use;
	cy /= use;
	double sxx = 0, sxy = 0;
	for (int i = 0; i < use; i++) {
		double dx = c->mono[idx[i]] - mx;
		sxx += dx * dx;
		sxy += dx * (c->clk[idx[i]] - cy);
	}
	*mx_out = mx;
	*cy_out = cy;
	return sxx > 0 ? sxy / sxx : 1e-3;
}

/* The fitted line, to evaluate at several points from one fit. */
struct btline {
	double mx, cy, rate;           /* rate: clock us per mono ns */
};

static inline struct btline bt_line(const struct btclock *c)
{
	struct btline l;
	l.rate = bt_fit(c, &l.mx, &l.cy);
	return l;
}

static inline double bt_at(const struct btline *l, double mono)
{
	return l->cy + l->rate * (mono - l->mx);
}

/* ---- the 3DSP broadcast message (Table 8.1) ----------------------------- */

#define MSG_LEN 17

static inline void put16(uint8_t *p, unsigned v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
}

static inline void put32(uint8_t *p, uint32_t v)
{
	put16(p, v & 0xffff);
	put16(p + 2, v >> 16);
}

static inline unsigned get16(const uint8_t *p)
{
	return p[0] | p[1] << 8;
}

static inline uint32_t get32(const uint8_t *p)
{
	return get16(p) | (uint32_t)get16(p + 2) << 16;
}

static inline unsigned clamp_off(double v)
{
	long r = lround(v);
	return r < 0 ? 0 : r > 0xFFFE ? 0xFFFE : (unsigned)r;
}

/* fsi_us on the native clock, period_us the L+R pair (fractional), off[4] =
 * left open, left close, right open, right close in us after the instant.
 * period 0 (or > 40 ms) asks for 2D: both lenses open. */
static inline void pack_message(uint8_t m[MSG_LEN], double fsi_us, double period_us,
                                const double off[4])
{
	double w = fmod(round(fsi_us), CLK_WRAP_US);
	if (w < 0)
		w += CLK_WRAP_US;
	uint64_t fsi = (uint64_t)w;
	uint32_t slots = (uint32_t)(fsi / 625) & 0x07FFFFFF;   /* bits 27-31: reserved, 3D */
	unsigned phase = (unsigned)(fsi % 625);
	memset(m, 0, MSG_LEN);
	put32(m, slots);
	put16(m + 4, phase);
	if (period_us <= 0 || period_us > 40000) {
		put16(m + 6, 0xFFFF);
		return;
	}
	unsigned whole = (unsigned)period_us;
	long frac = lround((period_us - whole) * 256);
	if (frac == 256) {
		whole++;
		frac = 0;
	}
	for (int i = 0; i < 4; i++)
		put16(m + 6 + 2 * i, clamp_off(off[i]));
	put16(m + 14, whole);
	m[16] = (uint8_t)frac;
}

/* 2D: both lenses open (no timing), the message while there is no 3D. */
static inline void pack_2d(uint8_t m[MSG_LEN])
{
	static const double none[4];
	pack_message(m, 0, 0, none);
}

/* Each lens opens open_after us after its eye's frame starts and stays open
 * open_us (0: until the other eye's frame starts); the right window ends on
 * the period, as the add-on's bluetooth.py sends it. */
static inline void window_offsets(double period_us, double open_after, double open_us,
                                  double off[4])
{
	double half = period_us / 2;
	double width = half - open_after;
	if (open_us > 0 && open_us < width)
		width = open_us;
	if (width < 0)
		width = 0;
	off[0] = open_after;
	off[1] = open_after + width;
	off[2] = half + open_after;
	off[3] = half + open_after + width;
}
