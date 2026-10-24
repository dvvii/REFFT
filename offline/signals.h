#pragma once
// Test signals with known content, shared by refft-gen (files for the
// sampler and for offline drones) and refft-check (the same signals in
// memory). Deterministic: the same seed gives the same noise.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using Signal = std::vector<float>;

static inline float dbToAmp(float db) { return powf(10.f, db / 20.f); }

static inline float rmsOf(const Signal& x, size_t from = 0, size_t to = SIZE_MAX)
{
	to = std::min(to, x.size());
	double s = 0;
	for(size_t n = from; n < to; ++n)
		s += (double)x[n] * x[n];
	return to > from ? (float)sqrt(s / (to - from)) : 0.f;
}

static inline float ampToDb(float a) { return 20.f * log10f(a + 1e-12f); }

// xorshift: uniform in [-1, 1)
struct Rng {
	uint64_t s;
	explicit Rng(uint64_t seed) : s(seed * 2654435761u + 1) {}
	float next()
	{
		s ^= s << 13;
		s ^= s >> 7;
		s ^= s << 17;
		return (float)((s >> 11) * (1.0 / 9007199254740992.0)) * 2.f - 1.f;
	}
};

static inline void scaleToRms(Signal& x, float rmsDb)
{
	const float r = rmsOf(x);
	const float g = r > 0 ? dbToAmp(rmsDb) / r : 0.f;
	for(auto& v : x)
		v *= g;
}

static inline Signal white(size_t n, float rmsDb, uint64_t seed = 1)
{
	Signal x(n);
	Rng r(seed);
	for(auto& v : x)
		v = r.next();
	scaleToRms(x, rmsDb);
	return x;
}

// Pink (−3 dB/octave), Paul Kellet's refined filter
static inline Signal pink(size_t n, float rmsDb, uint64_t seed = 2)
{
	Signal x(n);
	Rng r(seed);
	float b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
	for(auto& v : x)
	{
		const float w = r.next();
		b0 = 0.99886f * b0 + w * 0.0555179f;
		b1 = 0.99332f * b1 + w * 0.0750759f;
		b2 = 0.96900f * b2 + w * 0.1538520f;
		b3 = 0.86650f * b3 + w * 0.3104856f;
		b4 = 0.55000f * b4 - w * 0.5329522f;
		b5 = -0.7616f * b5 - w * 0.0168980f;
		v = b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362f;
		b6 = w * 0.115926f;
	}
	scaleToRms(x, rmsDb);
	return x;
}

static inline Signal sine(size_t n, float fs, float hz, float peakDb)
{
	Signal x(n);
	const float a = dbToAmp(peakDb);
	for(size_t i = 0; i < n; ++i)
		x[i] = a * sinf((float)(2. * M_PI * fmod(hz * (double)i / fs, 1.)));
	return x;
}

// Band-limited saw (polyBLEP), falling ramp like most modular saws
static inline Signal saw(size_t n, float fs, float hz, float peakDb)
{
	Signal x(n);
	const float a = dbToAmp(peakDb);
	const double dt = hz / fs;
	double ph = 0;
	for(size_t i = 0; i < n; ++i)
	{
		double v = 1. - 2. * ph;
		double t = ph;
		if(t < dt) { t /= dt; v += t + t - t * t - 1.; }
		else if(t > 1. - dt) { t = (t - 1.) / dt; v += t * t + t + t + 1.; }
		x[i] = a * (float)v;
		ph += dt;
		if(ph >= 1.)
			ph -= 1.;
	}
	return x;
}

// Sine through a sequence of pitches, each held `holdSec`, with 5 ms
// crossfades so the steps don't click
static inline Signal steps(size_t n, float fs, const std::vector<float>& hz, float holdSec, float peakDb)
{
	Signal x(n);
	const float a = dbToAmp(peakDb);
	const size_t hold = (size_t)(holdSec * fs);
	const size_t fade = (size_t)(0.005f * fs);
	double ph = 0;
	for(size_t i = 0; i < n; ++i)
	{
		const size_t step = i / hold;
		const size_t inStep = i % hold;
		float f = hz[step % hz.size()];
		if(inStep < fade && i >= hold)
		{
			const float prev = hz[(step + hz.size() - 1) % hz.size()];
			f = prev + (f - prev) * inStep / (float)fade;
		}
		ph += f / fs;
		ph -= floor(ph);
		x[i] = a * sinf((float)(2. * M_PI * ph));
	}
	return x;
}

// Raised-cosine gate: 1 inside [on, off) seconds with `rampSec` edges
static inline float gate(double t, double on, double off, double rampSec)
{
	if(t <= on - rampSec || t >= off + rampSec)
		return 0.f;
	if(t < on)
		return 0.5f - 0.5f * cosf((float)(M_PI * (t - (on - rampSec)) / rampSec));
	if(t > off)
		return 0.5f + 0.5f * cosf((float)(M_PI * (t - off) / rampSec));
	return 1.f;
}

// Pink noise switched on and off: `onSec` of noise, `offSec` of digital silence
static inline Signal gaps(size_t n, float fs, float rmsDb, float onSec, float offSec)
{
	Signal x = pink(n, rmsDb, 3);
	const double period = onSec + offSec;
	for(size_t i = 0; i < n; ++i)
	{
		const double t = fmod(i / (double)fs, period);
		x[i] *= gate(t, 0.01, onSec - 0.01, 0.01);
	}
	return x;
}

// A steady tone over a pink bed, `toneSec` long every `periodSec`: one long
// event, to hear the floor tracker catch up with it
static inline Signal whistle(size_t n, float fs, float bedDb, float hz, float toneDb, float toneSec, float periodSec)
{
	Signal x = pink(n, bedDb, 4);
	const float a = dbToAmp(toneDb);
	for(size_t i = 0; i < n; ++i)
	{
		const double t = fmod(i / (double)fs, periodSec);
		const float g = gate(t, 1.0, 1.0 + toneSec, 0.01);
		if(g > 0)
			x[i] += g * a * sinf((float)(2. * M_PI * fmod(hz * (double)i / fs, 1.)));
	}
	return x;
}

// A synthetic field recording: a pink bed, "birds" (short rising and
// falling chirps between 2.5 and 6 kHz, inside the default event band) and
// "frogs" (trains of 400–500 Hz pulses, below it). Events land at random
// times, so the floor tracker sees them as it would in a recording.
static inline Signal birds(size_t n, float fs, float bedDb, float birdDb, float frogDb)
{
	Signal x = pink(n, bedDb, 5);
	Rng r(6);
	const float birdA = dbToAmp(birdDb), frogA = dbToAmp(frogDb);
	// birds: a call every 0.8–2.2 s, 3–6 notes of 60–140 ms
	for(double t = 0.5; t < n / (double)fs - 1.0; t += 1.5 + 0.7 * r.next())
	{
		const int notes = 4 + (int)(1.5f * r.next());
		const float base = 3500.f + 1500.f * r.next();
		double nt = t;
		for(int k = 0; k < notes; ++k)
		{
			const float dur = 0.1f + 0.04f * r.next();
			const float f0 = base * (1.f + 0.25f * r.next()), f1 = f0 * (1.f + 0.3f * r.next());
			const size_t s0 = (size_t)(nt * fs), len = (size_t)(dur * fs);
			double ph = 0;
			for(size_t i = 0; i < len && s0 + i < n; ++i)
			{
				const float u = i / (float)len;
				ph += (f0 + (f1 - f0) * u) / fs;
				x[s0 + i] += birdA * sinf((float)(M_PI * u)) * sinf((float)(2. * M_PI * ph));
			}
			nt += dur + 0.03f + 0.03f * fabsf(r.next());
		}
	}
	// frogs: a train of 8–14 pulses (20 ms, 450 Hz ± 50) every 2–4 s
	for(double t = 1.0; t < n / (double)fs - 1.5; t += 3.0 + r.next())
	{
		const int pulses = 11 + (int)(3.f * r.next());
		const float f = 450.f + 50.f * r.next();
		for(int k = 0; k < pulses; ++k)
		{
			const size_t s0 = (size_t)((t + k * 0.06) * fs), len = (size_t)(0.02 * fs);
			for(size_t i = 0; i < len && s0 + i < n; ++i)
			{
				const float u = i / (float)len;
				x[s0 + i] += frogA * sinf((float)(M_PI * u)) * sinf((float)(2. * M_PI * f * i / fs));
			}
		}
	}
	return x;
}
