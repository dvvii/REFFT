#pragma once
// The audio path of cpp/render.cpp, sample for sample, without the board:
// input gains, the circular buffers and hop scheduling, the stamp itself
// (cpp/Stamp.cpp, unchanged), overlap-add, the size-change fade, the dry/wet
// and dry-source mix, bypass, out R gain and limiter (cpp/Limiter.cpp).
//
// The knob mappings and the mix are copied from render(), not shared with
// it: a change there has to be made here too. Each copied block names the
// lines it mirrors. Two things differ on purpose:
// - the stamp task runs at once instead of on another core. On the board
//   it has one hop to finish, and its output lands one hop later either
//   way, so the result is the same unless the board skips frames;
// - knobs are set directly. The panel's 5 ms smoothing and pickup are left
//   out, so a stepped sweep changes values at the step, inside one block.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>
#include "Limiter.h"
#include "Stamp.h"

// Knob positions 0–1, as on the panel and in the GUI
struct Knobs {
	enum A { kDroneGain, kFieldGain, kWhiten, kContrast, kFloorOnly, kSmear, kTonality, kWet };
	enum B { kOutGain, kAttack, kFloorSpeed, kEventThresh, kDrySource, kCvSmooth, kEventLow, kEventHigh };
	// render.cpp setup(): the boot defaults
	std::array<float, 8> a {{ 0.5f, 0.5f, 0.7f, 0.33f, 0.f, 0.f, 0.f, 0.5f }};
	std::array<float, 8> b {{ 0.5f, 0.f, 0.3f, 0.33f, 0.f, 0.3f, 0.6174f, 0.9491f }};
	bool freeze = false;
	bool bypass = false;
	unsigned int sizeIndex = 1; // into Stamp::kSizes
};

// Signals a render can write, all per sample at the board's rate
struct Taps {
	float outR = 0; // after gain and limiter, as at the jack
	float wet = 0;  // the stamp's output alone, before the mix
	float dry = 0;  // field/drone blend before the mix
};

class Engine {
public:
	static constexpr unsigned int kBufSize = 32768; // render.cpp: power of two, > 2 * max FFT size
	static constexpr unsigned int kBufMask = kBufSize - 1;
	static constexpr unsigned int kBlock = 16;      // settings.json blockSize

	bool setup(float sampleRate)
	{
		fs = sampleRate;
		if(stamp.setup(fs))
			return false;
		const unsigned int maxN = stamp.getMaxSize();
		droneFrame.assign(maxN, 0.f);
		fieldFrame.assign(maxN, 0.f);
		outFrame.assign(maxN, 0.f);
		in[0].assign(kBufSize, 0.f);
		in[1].assign(kBufSize, 0.f);
		out.assign(kBufSize, 0.f);
		limiter.setup(fs, -1.f, 0.1f);
		pendingSize = stamp.getSizeIndex();
		return true;
	}

	// Call before every block (kBlock samples), like render() reading the panel
	void setKnobs(const Knobs& k)
	{
		knobs = k;
		pendingSize = std::min<unsigned int>(k.sizeIndex, Stamp::kSizes.size() - 1);
		// render.cpp render(), "parameters"
		const float hopSec = stamp.getHop() / fs;
		auto A = [&](unsigned int i) { return k.a[i]; };
		auto B = [&](unsigned int i) { return k.b[i]; };
		droneGain = 4.f * A(Knobs::kDroneGain) * A(Knobs::kDroneGain);
		fieldGain = 4.f * A(Knobs::kFieldGain) * A(Knobs::kFieldGain);
		params.whiten = A(Knobs::kWhiten);
		params.contrast = 0.5f * powf(8.f, A(Knobs::kContrast));
		params.floorOnly = A(Knobs::kFloorOnly);
		params.release = onePoleCoef(5.f * A(Knobs::kSmear) * A(Knobs::kSmear), 1.f / hopSec);
		params.tonality = A(Knobs::kTonality);
		const float wetDry = A(Knobs::kWet);
		const float outKnob = B(Knobs::kOutGain);
		outGain = outKnob < 0.02f ? 0.f : powf(10.f, (-24.f + 48.f * outKnob) / 20.f);
		params.attack = onePoleCoef(2.f * B(Knobs::kAttack) * B(Knobs::kAttack), 1.f / hopSec);
		params.floorRiseDb = 0.5f + 30.f * B(Knobs::kFloorSpeed) * B(Knobs::kFloorSpeed);
		params.eventDb = 3.f + 27.f * B(Knobs::kEventThresh);
		const float dryBalance = B(Knobs::kDrySource);
		const float lo = knobToHz(B(Knobs::kEventLow)), hi = knobToHz(B(Knobs::kEventHigh));
		params.eventLoHz = std::min(lo, hi);
		params.eventHiHz = std::max(lo, hi);
		params.freeze = k.freeze;
		wetGain = sinf(wetDry * (float)M_PI_2);
		dryGain = cosf(wetDry * (float)M_PI_2);
		dryField = cosf(dryBalance * (float)M_PI_2);
		dryDrone = sinf(dryBalance * (float)M_PI_2);
		fadeStep = 1.f / (0.02f * fs);
		bypassCoef = onePoleCoef(0.01f, fs);
	}

	// One sample in (raw jack values, before the gain knobs), one out.
	// render.cpp render(), the per-frame loop.
	Taps process(float rawDrone, float rawField)
	{
		const float drone = rawDrone * droneGain;
		const float field = rawField * fieldGain;
		in[0][write] = drone;
		in[1][write] = field;
		const float wet = out[write] * wetFade;
		out[write] = 0;
		write = (write + 1) & kBufMask;

		if(pendingSize != stamp.getSizeIndex())
		{
			wetFade = std::max(0.f, wetFade - fadeStep);
			if(0 == wetFade)
			{
				stamp.setSizeIndex(pendingSize);
				std::fill(out.begin(), out.end(), 0.f);
				hopCount = 0;
				setKnobs(knobs); // the hop length, so the smoothing coefficients, changed
			}
		}
		else
		{
			wetFade = std::min(1.f, wetFade + fadeStep);
			if(++hopCount >= stamp.getHop())
			{
				hopCount = 0;
				runFrame(write);
			}
		}

		bypassGain += bypassCoef * ((knobs.bypass ? 1.f : 0.f) - bypassGain);
		const float dry = field * dryField + drone * dryDrone;
		float mix = dry * dryGain + wet * wetGain;
		mix = mix * (1.f - bypassGain) + dry * bypassGain;
		Taps t;
		t.outR = limiter.process(mix * outGain);
		t.wet = wet;
		t.dry = dry;
		limitDb = std::min(limitDb, limiter.reductionDb());
		return t;
	}

	const StampFeatures& features() const { return lastFeatures; }
	unsigned int framesDone() const { return frames; }
	unsigned int size() const { return stamp.getSize(); }
	unsigned int hop() const { return stamp.getHop(); }
	// most gain reduction on out R since the last call, dB ≤ 0
	float takeLimitDb() { float d = limitDb; limitDb = 0; return d; }

private:
	static float onePoleCoef(float tauSec, float rate)
	{
		return tauSec <= 0 ? 1.f : 1.f - expf(-1.f / (tauSec * rate));
	}
	static float knobToHz(float v) { return 100.f * powf(128.f, v); }

	// render.cpp stampTask()
	void runFrame(unsigned int end)
	{
		const unsigned int N = stamp.getSize();
		const unsigned int hopLen = stamp.getHop();
		for(unsigned int n = 0; n < N; ++n)
		{
			const unsigned int idx = (end - N + n) & kBufMask;
			droneFrame[n] = in[0][idx];
			fieldFrame[n] = in[1][idx];
		}
		stamp.processFrame(droneFrame.data(), fieldFrame.data(), outFrame.data(), params, lastFeatures);
		const unsigned int start = end + hopLen;
		for(unsigned int n = 0; n < N; ++n)
			out[(start + n) & kBufMask] += outFrame[n];
		++frames;
	}

	float fs = 44100;
	Stamp stamp;
	Limiter limiter;
	StampParams params;
	StampFeatures lastFeatures;
	Knobs knobs;
	std::vector<float> in[2], out, droneFrame, fieldFrame, outFrame;
	unsigned int write = 0, hopCount = 0, pendingSize = 1, frames = 0;
	float wetFade = 1, bypassGain = 0, limitDb = 0;
	float droneGain = 1, fieldGain = 1, outGain = 1;
	float wetGain = 0, dryGain = 1, dryField = 1, dryDrone = 0, fadeStep = 0, bypassCoef = 1;
};
