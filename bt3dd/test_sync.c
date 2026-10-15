/* Host test of sync.h: three free-running clocks (host, Pi, Bluetooth) with
 * their own skews and offsets, USB delays with jitter and stalls, Read_Clock
 * brackets over a UART, clocks wrapping, a rate change, a host gap. The frame
 * sync instant bt3dd would broadcast is compared with the true one.
 *
 *   cc -O2 -o test_sync test_sync.c -lm && ./test_sync
 */
#include <stdio.h>

#include "sync.h"

static int failures;

#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL: " __VA_ARGS__); printf("\n"); failures++; } } while (0)

/* deterministic uniform [0, 1) */
static uint64_t rng = 88172645463325252ULL;
static double urand(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 7;
	rng ^= rng << 17;
	return (rng >> 11) * (1.0 / 9007199254740992.0);
}

struct world {
	double host_ppm, pi_ppm, bt_ppm;
	double host_off_us, pi_off_ns, bt_off_us;
	/* NTP slewing the box's clock: its stamps wander from the video clock's
	 * vsyncs by +-slew_us over slew_period_s (0: no slewing) */
	double slew_us, slew_period_s;
};

/* true time t in ns -> each clock */
static double host_us(const struct world *w, double t) { return w->host_off_us + t / 1000.0 * (1 + w->host_ppm * 1e-6); }
static double pi_ns(const struct world *w, double t) { return w->pi_off_ns + t * (1 + w->pi_ppm * 1e-6); }
static double bt_us(const struct world *w, double t) { return w->bt_off_us + t / 1000.0 * (1 + w->bt_ppm * 1e-6); }
static double t_of_host(const struct world *w, double hu) { return (hu - w->host_off_us) * 1000.0 / (1 + w->host_ppm * 1e-6); }

static double wrap_diff(double a, double b, double wrap)
{
	double d = fmod(a - b, wrap);
	if (d > wrap / 2) d -= wrap;
	if (d < -wrap / 2) d += wrap;
	return d;
}

/* The box stamps its vsyncs with jitter: measured on it, steps between
 * stamps spread over +-120 us (1st to 99th percentile). The stamp is off,
 * the vsync itself is not. */
static double stamp_jitter_us(void)
{
	return (urand() + urand() + urand() - 1.5) * 80.0;   /* within +-120, sigma ~40 */
}

static unsigned last_rate_changes;

/* One run: 'seconds' of vsyncs at 'hz' (host clock), optional rate change and
 * gap. Returns the spread (max - min) of the FSI error after warm-up, in us. */
static double run(const char *name, struct world w, double hz, double seconds,
                  double change_at, double hz2, double gap_at, double gap_s,
                  double *bias_out, double *period_err_out)
{
	struct hostclock h;
	struct btclock c;
	memset(&h, 0, sizeof(h));
	bt_reset(&c);
	double period_host_us = 1e6 / hz;
	double next_vsync_host = host_us(&w, 1e9);      /* first vsync at t = 1 s */
	uint32_t vsync = 4000000000u;                   /* vsync counter wraps too */
	double t = 1e9, next_tick = 1e9, end = 1e9 + seconds * 1e9;
	double emin = 1e18, emax = -1e18, esum = 0, perr_max = 0;
	int en = 0, changed = 0, gapped = 0;
	struct { int valid; double vm, pm; int left; double true_bt; } track = { 0 };

	while (t < end) {
		double tv = t_of_host(&w, next_vsync_host);
		if (tv < next_tick) {
			t = tv;
			if (!changed && change_at > 0 && t > 1e9 + change_at * 1e9) {
				changed = 1;
				period_host_us = 1e6 / hz2;
			}
			int in_gap = gap_at > 0 && t > 1e9 + gap_at * 1e9 && t < 1e9 + (gap_at + gap_s) * 1e9;
			if (in_gap)
				gapped = 1;
			int left = (vsync & 1) == 0;
			/* USB: 250 us floor, exponential jitter, the odd 5 ms stall */
			double lat = 250e3 + -log(1 - urand()) * 400e3 + (urand() < 0.01 ? 5e6 : 0);
			if (!in_gap) {
				double stamp = next_vsync_host + stamp_jitter_us();
				if (w.slew_period_s > 0)
					stamp += w.slew_us * sin(2 * M_PI * t / (w.slew_period_s * 1e9));
				uint32_t hus = (uint32_t)(uint64_t)llround(fmod(stamp + 4294967296.0, 4294967296.0));
				double vm, pm;
				int64_t arrival = (int64_t)pi_ns(&w, t + lat);
				if (host_on_sync(&h, vsync, hus, arrival, &vm, &pm)) {
					track.valid = 1;
					track.vm = vm;
					track.pm = pm;
					track.left = left;
					track.true_bt = bt_us(&w, t);
				}
			}
			vsync++;
			next_vsync_host += period_host_us;
			continue;
		}
		t = next_tick;
		next_tick += 100e6 + urand() * 17e6;          /* bt3dd's tick: 100 ms + 0-17 ms */
		/* Read_Clock over the UART: ~180 us each way, jittered */
		double before = pi_ns(&w, t);
		double t_sample = t + 150e3 + urand() * 60e3;
		double t_after = t_sample + 150e3 + urand() * 60e3 + (urand() < 0.02 ? 2e6 : 0);
		double bt = bt_us(&w, t_sample);
		uint32_t clkn = (uint32_t)((uint64_t)floor(bt / CLKN_US) & (CLKN_WRAP - 1));
		bt_add(&c, (int64_t)before, clkn, (int64_t)pi_ns(&w, t_after));
		if (!track.valid || !bt_ready(&c) || t < 1e9 + 10e9)
			continue;
		if (gapped && t < 1e9 + (gap_at + gap_s + 10) * 1e9)
			continue;                               /* re-converging after the gap */
		if (changed && t < 1e9 + (change_at + 6) * 1e9)
			continue;                               /* detected over 128 frames, then averaged */
		double left_m = track.vm - (track.left ? 0 : track.pm);
		double true_left = track.true_bt - (track.left ? 0 : 1e6 / hz * (changed ? hz / hz2 : 1) *
		                   (1 + w.bt_ppm * 1e-6) / (1 + w.host_ppm * 1e-6));
		struct btline line = bt_line(&c);
		double fsi = bt_at(&line, left_m);
		double e = wrap_diff(fsi, true_left, CLK_WRAP_US);
		double pair = 2 * track.pm * line.rate;
		double true_pair = 2 * period_host_us * (1 + w.bt_ppm * 1e-6) / (1 + w.host_ppm * 1e-6);
		double pe = fabs(pair - true_pair);
		if (pe > perr_max) perr_max = pe;
		if (e < emin) emin = e;
		if (e > emax) emax = e;
		esum += e;
		en++;
	}
	double spread = emax - emin;
	*bias_out = en ? esum / en : 0;
	*period_err_out = perr_max;
	last_rate_changes = h.rate_changes;
	printf("%-34s FSI bias %7.1f us, spread %5.1f us, period error max %.3f us (%d checks, %u resets, %u rate changes)\n",
	       name, *bias_out, spread, perr_max, en, h.resets, h.rate_changes);
	return en ? spread : 1e9;
}

static void test_pack(void)
{
	uint8_t m[MSG_LEN];
	pack_2d(m);
	CHECK(m[6] == 0xff && m[7] == 0xff && m[14] == 0 && m[15] == 0, "2D message: left open 0xFFFF, period 0");
	/* 120 Hz pair, FSI 1 s in: slots 1600, phase 0 */
	double off[4];
	window_offsets(16666.6667, 0, 0, off);
	pack_message(m, 1e6, 16666.6667, off);
	CHECK(m[0] == 0x40 && m[1] == 0x06 && m[2] == 0 && m[3] == 0, "FSI slots 1600 (got %02x%02x)", m[1], m[0]);
	CHECK(m[4] == 0 && m[5] == 0, "phase 0");
	CHECK((m[14] | m[15] << 8) == 16666 && m[16] == 171, "period 16666 + 171/256 (got %u + %u)", m[14] | m[15] << 8, m[16]);
	unsigned lo = m[6] | m[7] << 8, lc = m[8] | m[9] << 8, ro = m[10] | m[11] << 8, rc = m[12] | m[13] << 8;
	CHECK(lo == 0 && lc == ro && rc == 16667, "windows end to end, the right one ending on the period (%u-%u, %u-%u)", lo, lc, ro, rc);
	/* wrap: the instant is taken modulo 2^28 ticks */
	pack_message(m, CLK_WRAP_US + 625 * 3 + 7, 16666.6667, off);
	CHECK(m[0] == 3 && m[1] == 0 && m[4] == 7, "FSI wraps with the clock");
	/* lens window shortened: open 2000 us after each eye starts, for 3000 us */
	window_offsets(20833.3333, 2000, 3000, off);
	CHECK(fabs(off[0] - 2000) < 1e-6 && fabs(off[1] - 5000) < 1e-6 &&
	      fabs(off[2] - 12416.667) < 1e-3 && fabs(off[3] - 15416.667) < 1e-3, "shortened window");
}

int main(void)
{
	test_pack();

	struct world w = { 87.0, -42.0, 15.0, 4294967296.0 - 3e6, 5e12, CLK_WRAP_US - 4e6, 0, 0 };
	double bias, perr, bias0;
	double s = run("95.904 Hz, all clocks wrap", w, 95.904, 120, 0, 0, 0, 0, &bias0, &perr);
	CHECK(s < 150, "FSI spread %.1f us (glasses allow 500)", s);
	CHECK(perr < 1.0, "period error %.3f us (profile: 1)", perr);
	CHECK(last_rate_changes == 0, "the stamps' jitter is no rate change (%u)", last_rate_changes);

	struct world w2 = { -150.0, 150.0, -20.0, 1e6, 1e9, 1e6, 0, 0 };
	s = run("120 Hz, opposite skews", w2, 120.0, 90, 0, 0, 0, 0, &bias, &perr);
	CHECK(s < 150, "FSI spread %.1f us", s);
	CHECK(perr < 1.0, "period error %.3f us", perr);
	CHECK(fabs(bias - bias0) < 60, "the bias stays put across setups (%.1f vs %.1f): the delay setting holds", bias, bias0);

	s = run("VRR 95.904 -> 100 Hz at 40 s", w, 95.904, 90, 40, 100.0, 0, 0, &bias, &perr);
	CHECK(s < 150, "FSI spread across a rate change %.1f us", s);
	CHECK(perr < 1.0, "period error after a rate change %.3f us", perr);

	CHECK(last_rate_changes >= 1, "the change to 100 Hz is seen");

	s = run("23.976 -> 24 fps (0.1 %) at 40 s", w, 95.904, 90, 40, 96.0, 0, 0, &bias, &perr);
	CHECK(last_rate_changes >= 1, "a 0.1 %% change is seen (%u)", last_rate_changes);
	CHECK(perr < 1.0, "period error after a 0.1 %% change %.3f us", perr);
	CHECK(s < 150, "FSI spread across a 0.1 %% change %.1f us", s);

	s = run("host silent 3 s at 30 s", w, 95.904, 90, 0, 0, 30, 3, &bias, &perr);
	CHECK(s < 150, "FSI spread after a host gap %.1f us", s);

	if (failures) {
		printf("%d FAILED\n", failures);
		return 1;
	}
	/* the box's clock slewed by NTP: measured on the box, +-1.5 ms over
	 * about 12 minutes; the picture follows the video clock, not that */
	struct world ws = w;
	ws.slew_us = 1500;
	ws.slew_period_s = 720;
	double still_bias, still = run("box clock steady, 30 min", w, 95.904, 1800, 0, 0, 0, 0, &still_bias, &perr);
	s = run("box clock slewed +-1.5 ms / 12 min", ws, 95.904, 1800, 0, 0, 0, 0, &bias, &perr);
	CHECK(s < still + 50, "FSI spread %.1f us with the box's clock slewed, %.1f without", s, still);
	CHECK(fabs(bias - still_bias) < 30, "bias %.1f us, %.1f without slewing", bias, still_bias);

	printf("all passed\n");
	return 0;
}
