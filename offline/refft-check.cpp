// refft-check: the listening guide's predictions, measured. Each check
// prints what it measured next to what it expects, so a pass shows its
// numbers too; the exit status is the number of failures. "observe" lines
// measure behaviour nobody has pinned down yet: no verdict, just the number.

#include <algorithm>
#include <complex>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "engine.h"
#include "signals.h"

static constexpr float kFs = 44100;
static unsigned int gChecks = 0, gFailed = 0;

static void check(const char* name, bool ok, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
static void check(const char* name, bool ok, const char* fmt, ...)
{
	char detail[256];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(detail, sizeof(detail), fmt, ap);
	va_end(ap);
	++gChecks;
	if(!ok)
		++gFailed;
	printf("  %-4s %-46s %s\n", ok ? "ok" : "FAIL", name, detail);
}

static void observe(const char* name, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static void observe(const char* name, const char* fmt, ...)
{
	char detail[256];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(detail, sizeof(detail), fmt, ap);
	va_end(ap);
	printf("  %-4s %-46s %s\n", "obs", name, detail);
}

// Knobs for a clean look at one stage: full transfer, contrast 1, no
// smoothing, no floor-only, fully wet, unity gains
static Knobs pure()
{
	Knobs k;
	k.a = {{ 0.5f, 0.5f, 1.f, 1.f / 3.f, 0.f, 0.f, 0.f, 1.f }};
	k.b[Knobs::kOutGain] = 0.5f;
	k.b[Knobs::kAttack] = 0.f;
	k.b[Knobs::kDrySource] = 0.f;
	return k;
}

struct Run {
	Signal wet, out;
};

static Run render(const Signal& drone, const Signal& field, const std::function<Knobs(size_t)>& knobsAt)
{
	Engine e;
	e.setup(kFs);
	const size_t n = std::min(drone.size(), field.size());
	Run r;
	r.wet.resize(n);
	r.out.resize(n);
	for(size_t i = 0; i < n; i += Engine::kBlock)
	{
		e.setKnobs(knobsAt(i));
		for(size_t j = i; j < std::min(n, i + Engine::kBlock); ++j)
		{
			const Taps t = e.process(drone[j], field[j]);
			r.wet[j] = t.wet;
			r.out[j] = t.outR;
		}
	}
	return r;
}

static Run render(const Signal& drone, const Signal& field, const Knobs& k)
{
	return render(drone, field, [&](size_t) { return k; });
}

static size_t sec(double s) { return (size_t)(s * kFs); }
static float rmsDb(const Signal& x, double from, double to) { return ampToDb(rmsOf(x, sec(from), sec(to))); }

// Amplitude of the component at `hz` over [from, to) seconds, dB re a
// full-scale sine (Hann-windowed DFT at one frequency)
static float toneDb(const Signal& x, double hz, double from, double to)
{
	const size_t a = sec(from), b = std::min(x.size(), sec(to));
	std::complex<double> acc = 0;
	double wsum = 0;
	for(size_t n = a; n < b; ++n)
	{
		const double w = 0.5 - 0.5 * cos(2 * M_PI * (n - a) / (b - a));
		acc += w * x[n] * std::polar(1., -2 * M_PI * hz * n / kFs);
		wsum += w;
	}
	return ampToDb((float)(2 * std::abs(acc) / wsum));
}

// Power spectrum of [from, to) seconds, averaged over Hann frames of N with
// half overlap (Welch). Through the stamp a partial takes the noise's
// phase, so it is a narrow band of noise, not a line: a single frame or a
// single frequency of it jitters by several dB, the average doesn't.
static std::vector<double> psd(const Signal& x, double from, double to, unsigned int N)
{
	Fft f;
	f.setup(N);
	std::vector<double> p(N / 2 + 1, 0.);
	double wsum = 0;
	for(unsigned int n = 0; n < N; ++n)
		wsum += 0.5 - 0.5 * cos(2 * M_PI * n / N);
	unsigned int frames = 0;
	for(size_t a = sec(from); a + N <= std::min(x.size(), sec(to)); a += N / 2, ++frames)
	{
		for(unsigned int n = 0; n < N; ++n)
			f.td(n) = x[a + n] * (0.5f - 0.5f * cosf(2.f * (float)M_PI * n / N));
		f.fft();
		for(unsigned int k = 0; k <= N / 2; ++k)
		{
			const double m = 2 * f.fda(k) / wsum; // a full-scale sine's bin reads 1
			p[k] += m * m;
		}
	}
	for(auto& v : p)
		v /= std::max(1u, frames);
	return p;
}

// Power between loHz and hiHz over [from, to) seconds, dB
static float bandDb(const Signal& x, double from, double to, double loHz, double hiHz)
{
	const unsigned int N = 4096;
	const std::vector<double> p = psd(x, from, to, N);
	double s = 0;
	for(unsigned int k = (unsigned int)ceil(loHz * N / kFs); k <= std::min<unsigned int>(N / 2, (unsigned int)(hiHz * N / kFs)); ++k)
		s += p[k];
	return 10.f * log10f((float)s + 1e-24f);
}

// Power-weighted centre and RMS width (Hz) of the spectrum between loHz and hiHz
static void bandShape(const std::vector<double>& p, unsigned int N, double loHz, double hiHz, float& centre, float& width)
{
	double s = 0, m1 = 0, m2 = 0;
	for(unsigned int k = (unsigned int)ceil(loHz * N / kFs); k <= (unsigned int)(hiHz * N / kFs); ++k)
	{
		const double hz = k * kFs / N;
		s += p[k];
		m1 += p[k] * hz;
		m2 += p[k] * hz * hz;
	}
	centre = (float)(m1 / s);
	width = (float)sqrt(std::max(0., m2 / s - (m1 / s) * (m1 / s)));
}

static void level()
{
	printf("\n0. Levels: a sine drone through white noise, full transfer\n");
	const Signal drone = sine(sec(12), kFs, 1000, -6), field = white(sec(12), -30);
	const float droneDb = rmsDb(drone, 4, 12);
	Knobs k = pure();
	const Run noisePhase = render(drone, field, k);
	const float wetDb = rmsDb(noisePhase.wet, 4, 12);
	// Not the 6 dB that four overlapping frames with unrelated phases would
	// lose: consecutive frames share 3/4 of their noise, so their phases
	// mostly agree and the overlap-add stays nearly coherent.
	check("noise phase: wet a little under the drone", wetDb - droneDb < 0.f && wetDb - droneDb > -3.f,
		"drone %.1f, wet %.1f dBFS (%+.1f dB; expect 0 to -3)", droneDb, wetDb, wetDb - droneDb);
	k.a[Knobs::kTonality] = 1;
	const Run tonal = render(drone, field, k);
	const float tonalDb = rmsDb(tonal.wet, 4, 12);
	check("tonality 1: wet at the drone's level", fabsf(tonalDb - droneDb) < 0.5f,
		"wet %.1f dBFS (%+.1f dB; expect 0 ±0.5)", tonalDb, tonalDb - droneDb);
	const float outDb = rmsDb(tonal.out, 4, 12);
	check("out gain 0 dB, wet 100%: out R = wet", fabsf(outDb - tonalDb) < 0.3f,
		"out R %.1f dBFS (expect %.1f ±0.3)", outDb, tonalDb);
}

static void pitch()
{
	printf("\n1. Pitch and FFT size: one partial (a narrow noise band) where the sine is\n");
	const Signal drone = sine(sec(20), kFs, 1000, -6), field = white(sec(20), -30);
	float widths[4];
	for(unsigned int s = 0; s < 4; ++s)
	{
		Knobs k = pure();
		k.sizeIndex = s;
		const Run r = render(drone, field, k);
		const unsigned int N = 16384;
		float centre;
		bandShape(psd(r.wet, 3, 20, N), N, 500, 1500, centre, widths[s]);
		const float binHz = kFs / Stamp::kSizes[s];
		char name[64];
		snprintf(name, sizeof(name), "FFT %u: partial centred on 1 kHz", Stamp::kSizes[s]);
		check(name, fabsf(centre - 1000.f) < 0.25f * binHz, "centre %.1f Hz (±%.1f), RMS width %.1f Hz (bin %.1f Hz)",
			centre, 0.25f * binHz, widths[s], binHz);
	}
	check("bigger FFT, narrower partial", widths[0] > widths[1] && widths[1] > widths[2] && widths[2] > widths[3],
		"%.1f > %.1f > %.1f > %.1f Hz", widths[0], widths[1], widths[2], widths[3]);
}

static void whiten()
{
	printf("\n2. Whiten and the field's level\n");
	const Signal drone = sine(sec(60), kFs, 1000, -6);
	float wet[2][3];
	const float levels[3] = { -20, -40, -60 };
	for(int w = 0; w < 2; ++w)
	{
		for(int l = 0; l < 3; ++l)
		{
			Knobs k = pure();
			k.a[Knobs::kWhiten] = w ? 1.f : 0.f;
			const Run r = render(drone, white(sec(10), levels[l]), k);
			wet[w][l] = rmsDb(r.wet, 2, 10);
		}
	}
	check("whiten 1: wet ignores the field's level", fabsf(wet[1][0] - wet[1][2]) < 1.f,
		"field -20/-40/-60: wet %.1f / %.1f / %.1f dBFS", wet[1][0], wet[1][1], wet[1][2]);
	// below the threshold the stamp is drone x field: it follows the field dB for dB
	check("whiten 0: wet follows the field's level", fabsf(wet[0][0] - wet[0][1] - 20.f) < 1.5f && fabsf(wet[0][1] - wet[0][2] - 20.f) < 1.5f,
		"field -20/-40/-60: wet %.1f / %.1f / %.1f dBFS (expect 20 dB steps)", wet[0][0], wet[0][1], wet[0][2]);

	// a 20 dB drop in the field, held: whiten 0 follows it and stays down,
	// whiten 1 doesn't show it
	{
		Signal field = white(sec(30), -20);
		for(size_t n = sec(10); n < field.size(); ++n)
			field[n] *= 0.1f;
		float before[2], after[2], later[2];
		for(int w = 0; w < 2; ++w)
		{
			Knobs k = pure();
			k.a[Knobs::kWhiten] = w ? 1.f : 0.f;
			const Run r = render(drone, field, k);
			before[w] = rmsDb(r.wet, 5, 10);
			after[w] = rmsDb(r.wet, 10.3, 11.3);
			later[w] = rmsDb(r.wet, 25, 30);
		}
		check("whiten 0: a 20 dB field drop comes through", fabsf(after[0] - before[0] + 20.f) < 1.5f,
			"%+.1f dB right after (expect -20 ±1.5)", after[0] - before[0]);
		check("whiten 0: ...and stays", fabsf(later[0] - after[0]) < 1.f,
			"%+.1f dB 15 s later (expect 0 ±1)", later[0] - after[0]);
		check("whiten 1: the drop doesn't show", fabsf(after[1] - before[1]) < 1.f,
			"%+.1f dB right after (expect 0 ±1)", after[1] - before[1]);
	}

	// The field stops for 30 s (only the codec's hiss) and comes back: it
	// returns at its own level, as it does from the start
	for(float wv : { 0.f, 0.5f, 1.f })
	{
		Knobs k = pure();
		k.a[Knobs::kWhiten] = wv;
		Signal field = white(sec(60), -72, 8); // the board's in R with the sampler stopped
		const Signal noise = white(sec(60), -40, 7);
		for(size_t n = sec(30); n < field.size(); ++n)
			field[n] += noise[n];
		const Run r = render(drone, field, k);
		const float settled = rmsDb(r.wet, 50, 60);
		char name[64];
		snprintf(name, sizeof(name), "whiten %.1f: the field back after silence", wv);
		check(name, fabsf(rmsDb(r.wet, 30.2, 32) - settled) < 1.5f && fabsf(rmsDb(r.wet, 0.2, 2) - rmsDb(r.wet, 20, 30)) < 1.5f,
			"first 2 s %+.1f dB vs settled; at start %+.1f dB (expect 0 ±1.5)",
			rmsDb(r.wet, 30.2, 32) - settled, rmsDb(r.wet, 0.2, 2) - rmsDb(r.wet, 20, 30));
	}

	// the field stops: digital silence, then the codec's hiss
	{
		Knobs k = pure();
		const Run r = render(drone, Signal(sec(20), 0.f), k);
		check("field exactly silent: no wet", rmsDb(r.wet, 5, 20) < -120.f, "wet %.1f dBFS", rmsDb(r.wet, 5, 20));
	}
	for(float hiss : { -72.f, -95.f })
	{
		for(float wv : { 1.f, 0.7f, 0.f })
		{
			Knobs k = pure();
			k.a[Knobs::kWhiten] = wv;
			const Run r = render(drone, white(sec(10), hiss, 9), k);
			char name[64];
			snprintf(name, sizeof(name), "field = %.0f dBFS hiss only, whiten %.1f", hiss, wv);
			observe(name, "wet %.1f dBFS (drone %.1f)", rmsDb(r.wet, 2, 10), rmsDb(drone, 2, 10));
		}
	}
}

static void contrast()
{
	printf("\n3. Contrast on a saw\n");
	const Signal drone = saw(sec(10), kFs, 110, -6), field = white(sec(10), -30);
	float wet[3];
	int count[3];
	const float pos[3] = { 0.f, 1.f / 3.f, 1.f };
	for(int c = 0; c < 3; ++c)
	{
		Knobs k = pure();
		k.a[Knobs::kContrast] = pos[c];
		const Run r = render(drone, field, k);
		wet[c] = rmsDb(r.wet, 3, 10);
		// harmonics within 30 dB of the strongest
		std::vector<float> h;
		for(int m = 1; m * 110 < 10000; ++m)
			h.push_back(toneDb(r.wet, 110.0 * m, 3, 10));
		const float top = *std::max_element(h.begin(), h.end());
		count[c] = (int)std::count_if(h.begin(), h.end(), [&](float v) { return v > top - 30.f; });
	}
	check("contrast keeps the level", std::max({ wet[0], wet[1], wet[2] }) - std::min({ wet[0], wet[1], wet[2] }) < 2.f,
		"^0.5 / ^1 / ^4: wet %.1f / %.1f / %.1f dBFS", wet[0], wet[1], wet[2]);
	check("contrast thins out the partials", count[2] < count[1] && count[1] <= count[0],
		"harmonics within 30 dB of the top: %d / %d / %d", count[0], count[1], count[2]);
}

static void smoothing()
{
	printf("\n4. Smear, attack and freeze: 440 Hz, then 660 Hz from 3 s\n");
	const Signal drone = steps(sec(12), kFs, { 440.f, 660.f, 660.f, 660.f }, 3.f, -6), field = white(sec(12), -30);
	const double lat = (2048 + 512) / kFs; // N + hop
	const double step = 3.0 + lat;
	auto at440 = [](const Signal& x, double from, double to) { return bandDb(x, from, to, 400, 480); };
	auto at660 = [](const Signal& x, double from, double to) { return bandDb(x, from, to, 620, 700); };
	{
		Knobs k = pure();
		k.a[Knobs::kSmear] = 0.6f; // tau = 5 * 0.36 = 1.8 s
		const Run r = render(drone, field, k);
		const float a = at440(r.wet, step + 0.25, step + 1.25), b = at440(r.wet, step + 2.25, step + 3.25);
		check("smear 0.6 (1.8 s): 440 Hz rings on", fabsf((a - b) - 9.7f) < 2.f,
			"%.1f dB down over 2 s (expect 2 x 8.7/1.8 = 9.7 ±2)", a - b);
	}
	{
		Knobs k = pure();
		const Run r = render(drone, field, k);
		const float a = at440(r.wet, step - 1.0, step - 0.2), b = at440(r.wet, step + 0.2, step + 1.2);
		check("smear 0: 440 Hz stops with the note", a - b > 30.f, "%.1f dB down", a - b);
	}
	{
		// with the drone's phase the partial is a clean sine, so its level
		// at one moment can be read exactly
		Knobs k = pure();
		k.b[Knobs::kAttack] = 0.6f; // tau = 2 * 0.36 = 0.72 s
		k.a[Knobs::kTonality] = 1;
		const Run r = render(drone, field, k);
		const float atTau = toneDb(r.wet, 660, step + 0.72 - 0.1, step + 0.72 + 0.1), full = toneDb(r.wet, 660, 10, 12);
		check("attack 0.6 (0.72 s): 660 Hz at 63% after tau", fabsf((atTau - full) + 4.f) < 1.f,
			"%+.1f dB vs settled (expect 20 log 0.63 = -4 ±1)", atTau - full);
	}
	{
		const Run r = render(drone, field, [&](size_t n) { Knobs k = pure(); k.freeze = n >= sec(2); return k; });
		const float held = at440(r.wet, 8, 12), before = at440(r.wet, 0.5, 2), newNote = at660(r.wet, 8, 12);
		check("freeze at 2 s: 440 Hz holds", fabsf(held - before) < 1.f, "%+.1f dB at 8-12 s vs 0.5-2 s", held - before);
		check("freeze: the new note doesn't come through", newNote < held - 40.f, "660 Hz %.1f dB under 440", held - newNote);
	}
}

static void tonality()
{
	printf("\n5. Tonality 1 is the drone itself, delayed by N + hop\n");
	const Signal drone = saw(sec(6), kFs, 110, -6), field = white(sec(6), -30);
	for(unsigned int s : { 0u, 1u, 3u })
	{
		Knobs k = pure();
		k.a[Knobs::kTonality] = 1;
		k.sizeIndex = s;
		const Run r = render(drone, field, k);
		const unsigned int N = Stamp::kSizes[s], expect = N + N / 4;
		// the lag at which the wet matches the drone best
		size_t best = 0;
		double bestC = -1e30;
		const size_t a = sec(2), b = sec(4);
		for(size_t lag = 0; lag < 12000; ++lag)
		{
			double c = 0;
			for(size_t n = a; n < b; n += 4)
				c += r.wet[n] * drone[n - lag];
			if(c > bestC)
			{
				bestC = c;
				best = lag;
			}
		}
		Signal resid(b - a);
		for(size_t n = a; n < b; ++n)
			resid[n - a] = r.wet[n] - drone[n - expect];
		const float residDb = ampToDb(rmsOf(resid)) - rmsDb(drone, 2, 4);
		char name[64];
		snprintf(name, sizeof(name), "FFT %u: latency N + hop = %u", N, expect);
		check(name, best == expect, "measured %zu samples (%.1f ms)", best, 1000.0 * best / kFs);
		snprintf(name, sizeof(name), "FFT %u: wet - delayed drone", N);
		check(name, residDb < (0 == s ? -45.f : -60.f), "residual %.1f dB re the drone (expect < %d)", residDb, 0 == s ? -45 : -60);
	}
}

static void floorOnly()
{
	printf("\n6. Floor only: a 4 kHz whistle (1-4 s) over a pink bed, sine 220 Hz drone\n");
	const Signal drone = sine(sec(8), kFs, 220, -6), field = whistle(sec(8), kFs, -40, 4000, -25, 3, 8);
	const double lat = (2048 + 512) / kFs;
	const float fieldTone = toneDb(field, 4000, 1.5, 3.5);
	auto run = [&](float floorOnly, float speed, float lo, float hi) {
		Knobs k = pure();
		k.a[Knobs::kFloorOnly] = floorOnly;
		k.b[Knobs::kFloorSpeed] = speed;
		k.b[Knobs::kEventLow] = lo;
		k.b[Knobs::kEventHigh] = hi;
		return render(drone, field, k);
	};
	const float lo = 0.6174f, hi = 0.9491f; // 2-10 kHz, the defaults
	const float speed = 0.3f;               // 3.2 dB/s, the default
	// the bed alone (whistle off), 3-5 kHz: the drone has nothing there, so
	// whatever the wet has comes from bins passed through as events
	const float bedField = bandDb(field, 5, 7.5, 3000, 5000);
	{
		const Run r = run(1, speed, lo, hi);
		const float t = toneDb(r.wet, 4000, 1.5 + lat, 3.5 + lat);
		check("floor only 1: the whistle passes", fabsf(t - fieldTone) < 2.f,
			"wet %.1f vs field %.1f dB at 4 kHz", t, fieldTone);
		// Not all of it: noise peaks often sit 6 dB or more over the tracked
		// floor, so at the default threshold part of the bed counts as events
		const float bedWet = bandDb(r.wet, 5 + lat, 7.5, 3000, 5000);
		check("floor only 1: the bed is mostly stamped away", bedWet < bedField - 10.f,
			"bed 3-5 kHz: wet %.1f vs field %.1f dB (expect > 10 dB under)", bedWet, bedField);
	}
	for(float e : { 0.6f, 1.f })
	{
		Knobs k = pure();
		k.a[Knobs::kFloorOnly] = 1;
		k.b[Knobs::kFloorSpeed] = speed;
		k.b[Knobs::kEventThresh] = e;
		const Run r = render(drone, field, k);
		char name[64];
		snprintf(name, sizeof(name), "floor only 1, event threshold %.0f dB: bed", 3 + 27 * e);
		observe(name, "3-5 kHz: wet %.1f vs field %.1f dB; whistle %.1f vs %.1f dB",
			bandDb(r.wet, 5 + lat, 7.5, 3000, 5000), bedField, toneDb(r.wet, 4000, 1.5 + lat, 3.5 + lat), fieldTone);
	}
	{
		// at the slowest floor speed the floor sinks to the noise's deepest
		// dips, and ordinary noise starts to count as events
		const Run r = run(1, 0, lo, hi);
		observe("floor only 1, slowest floor (0.5 dB/s): bed", "3-5 kHz: wet %.1f vs field %.1f dB",
			bandDb(r.wet, 5 + lat, 7.5, 3000, 5000), bedField);
	}
	{
		const Run r = run(0, 0, lo, hi);
		const float t = toneDb(r.wet, 4000, 1.5 + lat, 3.5 + lat);
		check("floor only 0: the whistle is stamped too", t < fieldTone - 30.f, "wet %.1f vs field %.1f dB", t, fieldTone);
	}
	{
		const Run r = run(1, 0, 0.f, 0.4745f); // event band 100 Hz - 1 kHz
		const float t = toneDb(r.wet, 4000, 1.5 + lat, 3.5 + lat);
		check("event band 100 Hz-1 kHz: the whistle isn't an event", t < fieldTone - 30.f, "wet %.1f vs field %.1f dB", t, fieldTone);
	}
	{
		// floor speed 30.5 dB/s: the floor climbs into the whistle and it
		// stops counting as an event part-way through
		const Run r = run(1, 1, lo, hi);
		double gone = -1;
		for(double t = 1.1; t < 3.9; t += 0.05)
			if(toneDb(r.wet, 4000, t + lat, t + lat + 0.1) < toneDb(field, 4000, t, t + 0.1) - 10.f)
			{
				gone = t - 1.0;
				break;
			}
		check("fast floor (30.5 dB/s): the whistle fades into the stamp", gone > 0.5 && gone < 2.0,
			"%.2f s after its onset it is 10 dB under (expect 0.5-2)", gone);
	}
}

int main()
{
	printf("REFFT self-checks: cpp/Stamp.cpp through the render.cpp signal path, at 44.1 kHz\n");
	level();
	pitch();
	whiten();
	contrast();
	smoothing();
	tonality();
	floorOnly();
	printf("\n%u checks, %u failed\n", gChecks, gFailed);
	return (int)gFailed;
}
