#include "Stamp.h"
#include <cmath>
#include <algorithm>

constexpr unsigned int Stamp::kOverlap;
constexpr std::array<unsigned int, 4> Stamp::kSizes;

static inline float dbToLin(float db) { return powf(10.f, db / 20.f); }

int Stamp::setup(float fs)
{
	sampleRate = fs;
	for(unsigned int s = 0; s < kSizes.size(); ++s)
	{
		const unsigned int N = kSizes[s];
		SizeData& d = sizes[s];
		if(d.drone.setup(N) || d.field.setup(N))
			return 1;
		d.window.resize(N);
		float sum = 0;
		float sumSq = 0;
		for(unsigned int n = 0; n < N; ++n)
		{
			// periodic Hann, so that overlapping windows sum to a constant
			float w = 0.5f - 0.5f * cosf(2.f * (float)M_PI * n / N);
			d.window[n] = w;
			sum += w;
			sumSq += w * w;
		}
		// a full-scale sine lands in its bin with magnitude sum / 2
		d.magNorm = 2.f / sum;
		// measure the backend's ifft(fft(x)) gain instead of assuming it
		for(unsigned int n = 0; n < N; ++n)
			d.field.td(n) = 0;
		d.field.td(1) = 1;
		d.field.fft();
		d.field.ifft();
		float ifftGain = d.field.td(1);
		// analysis and synthesis Hann at overlap 4 sum to sumSq / hop
		float olaGain = (N / (float)kOverlap) / sumSq;
		d.outScale = 1.f / (d.magNorm * ifftGain) * olaGain;
	}
	const unsigned int maxBins = kSizes.back() / 2 + 1;
	mask.resize(maxBins);
	floorEst.resize(maxBins);
	prevMag.resize(maxBins);
	magDrone.resize(maxBins);
	magField.resize(maxBins);
	setSizeIndex(sizeIdx);
	return 0;
}

void Stamp::setSizeIndex(unsigned int idx)
{
	sizeIdx = std::min<unsigned int>(idx, kSizes.size() - 1);
	std::fill(mask.begin(), mask.end(), 0.f);
	std::fill(floorEst.begin(), floorEst.end(), 0.f);
	std::fill(prevMag.begin(), prevMag.end(), 0.f);
}

void Stamp::processFrame(const float* drone, const float* field, float* out,
		const StampParams& p, StampFeatures& f)
{
	SizeData& d = sizes[sizeIdx];
	const unsigned int N = kSizes[sizeIdx];
	const unsigned int K = N / 2 + 1;
	const float binHz = sampleRate / N;
	const float frameSec = getHop() / sampleRate;

	float droneSq = 0;
	float fieldSq = 0;
	for(unsigned int n = 0; n < N; ++n)
	{
		droneSq += drone[n] * drone[n];
		fieldSq += field[n] * field[n];
		d.drone.td(n) = drone[n] * d.window[n];
		d.field.td(n) = field[n] * d.window[n];
	}
	f.droneRms = sqrtf(droneSq / N);
	f.fieldRms = sqrtf(fieldSq / N);
	d.drone.fft();
	d.field.fft();

	float peak = 0;
	for(unsigned int k = 0; k < K; ++k)
	{
		magDrone[k] = d.drone.fda(k) * d.magNorm;
		magField[k] = d.field.fda(k) * d.magNorm;
		peak = std::max(peak, magDrone[k]);
	}

	// Below this carrier magnitude the whitening gain is capped, so the
	// result fades from magnitude transfer towards a spectral product.
	const float thresh = dbToLin(-6.f - p.whiten * 94.f);
	const float floorRise = dbToLin(p.floorRiseDb * frameSec);
	const float eventLo = p.eventDb - 6.f;
	const float tiny = 1e-12f;
	const unsigned int kLo = std::max(1u, (unsigned int)(100.f / binHz));
	const unsigned int kHi = std::min(K - 1, (unsigned int)(10000.f / binHz));

	float logPowSum = 0, powSum = 0;
	float centroidNum = 0, magSum = 0;
	float droneCentroidNum = 0, droneMagSum = 0;
	float flux = 0, eventNum = 0;

	for(unsigned int k = 0; k < K; ++k)
	{
		const float mc = magField[k];
		float md = magDrone[k];

		// drone mask: contrast relative to the frame peak, then smoothing
		if(p.contrast != 1.f && peak > tiny)
			md = peak * powf(md / peak, p.contrast);
		if(!p.freeze)
		{
			float coef = md > mask[k] ? p.attack : p.release;
			mask[k] += coef * (md - mask[k]);
		}

		// carrier noise floor: follow drops at once, rise slowly
		float& fl = floorEst[k];
		if(mc < fl || fl <= 0)
			fl = mc;
		else
			fl *= floorRise;
		float aboveDb = 20.f * log10f((mc + tiny) / (fl + tiny));
		float event = std::min(1.f, std::max(0.f, (aboveDb - eventLo) / 12.f));
		float stampW = 1.f - p.floorOnly * event;

		// carrier unit phasor, optionally pulled towards the drone's phase
		float ur = 1, ui = 0;
		if(mc > tiny)
		{
			ur = d.field.fdr(k) * d.magNorm / mc;
			ui = d.field.fdi(k) * d.magNorm / mc;
		}
		if(p.tonality > 0 && magDrone[k] > tiny)
		{
			float dr = d.drone.fdr(k) * d.magNorm / magDrone[k];
			float di = d.drone.fdi(k) * d.magNorm / magDrone[k];
			float br = (1.f - p.tonality) * ur + p.tonality * dr;
			float bi = (1.f - p.tonality) * ui + p.tonality * di;
			float bm = sqrtf(br * br + bi * bi);
			if(bm > 1e-6f)
			{
				ur = br / bm;
				ui = bi / bm;
			}
		}

		float stamped = mask[k] * (mc > thresh ? 1.f : mc / thresh);
		float mag = stamped * stampW + mc * (1.f - stampW);
		if(0 == k)
			mag = 0; // no DC
		// DC and Nyquist bins are real
		d.field.fdr(k) = ur * mag * d.outScale;
		d.field.fdi(k) = (0 == k || K - 1 == k) ? 0 : ui * mag * d.outScale;

		if(k >= kLo && k <= kHi)
		{
			float pw = mc * mc + tiny;
			logPowSum += logf(pw);
			powSum += pw;
			centroidNum += k * binHz * mc;
			magSum += mc;
			droneCentroidNum += k * binHz * magDrone[k];
			droneMagSum += magDrone[k];
			flux += std::max(0.f, mc - prevMag[k]);
			eventNum += event * mc;
		}
		prevMag[k] = mc;
	}

	const unsigned int bands = kHi - kLo + 1;
	f.fieldFlatness = powSum > 0 ? expf(logPowSum / bands) / (powSum / bands) : 0;
	f.fieldCentroid = magSum > tiny ? centroidNum / magSum : 0;
	f.droneCentroid = droneMagSum > tiny ? droneCentroidNum / droneMagSum : 0;
	f.fieldFlux = magSum > tiny ? flux / magSum : 0;
	f.eventAmount = magSum > tiny ? eventNum / magSum : 0;

	d.field.ifft();
	for(unsigned int n = 0; n < N; ++n)
		out[n] = d.field.td(n) * d.window[n];
}
