#pragma once
// Spectral timbre stamp: the field recording (carrier) keeps its phase and
// noise, the drone (modulator) supplies the magnitude of every bin.
// One call to processFrame() handles one analysis frame; overlap-add and
// threading live in render.cpp.

#include <libraries/Fft/Fft.h>
#include <array>
#include <vector>

struct StampParams {
	float whiten = 0.7f;      // 0: spectral product (carrier x drone), 1: full magnitude transfer
	float contrast = 1.f;     // exponent on the drone mask, relative to its peak (>1 sharpens partials)
	float floorOnly = 0.f;    // 0: stamp every bin, 1: stamp only the carrier's noise floor
	float eventDb = 12.f;     // how far above the tracked floor a bin counts as an event
	float floorRiseDb = 6.f;  // floor tracker rise speed, dB per second
	float eventLoHz = 2000.f; // only bins in this band count as events (half-octave
	float eventHiHz = 10000.f; // tapers outside): birds above frogs and wind
	float attack = 1.f;       // per-frame one-pole coefficients of the drone mask (1 = instant)
	float release = 1.f;
	float tonality = 0.f;     // 0: carrier phase, 1: drone phase
	bool freeze = false;      // hold the drone mask
};

// Log-spaced display bands, 100 Hz…12.8 kHz (seven octaves)
static constexpr unsigned int kDisplayBands = 42;
static constexpr float kDisplayLoHz = 100.f;
static constexpr float kDisplayOctaves = 7.f;

struct StampFeatures {
	float droneRms = 0;
	float fieldRms = 0;
	float fieldFlatness = 0; // 0 tonal .. 1 white
	float fieldCentroid = 0; // Hz, power-weighted on a log-frequency scale
	float fieldFlux = 0;     // positive spectral change in the event band, normalised
	float eventAmount = 0;   // energy-weighted share of the event band above the floor
	float droneCentroid = 0; // Hz
	std::array<float, kDisplayBands> bandDb {};    // carrier power per display band
	std::array<float, kDisplayBands> bandEvent {}; // its event share (band-weighted)
};

class Stamp {
public:
	static constexpr unsigned int kOverlap = 4;
	static constexpr std::array<unsigned int, 4> kSizes = {{ 1024, 2048, 4096, 8192 }};

	int setup(float sampleRate);
	// Selects one of kSizes and clears the per-bin state.
	// Not thread-safe against processFrame().
	void setSizeIndex(unsigned int idx);
	unsigned int getSizeIndex() const { return sizeIdx; }
	unsigned int getSize() const { return kSizes[sizeIdx]; }
	unsigned int getHop() const { return getSize() / kOverlap; }
	unsigned int getMaxSize() const { return kSizes.back(); }

	// drone, field: the last getSize() input samples, oldest first.
	// out: getSize() windowed samples ready to be overlap-added.
	void processFrame(const float* drone, const float* field, float* out,
			const StampParams& p, StampFeatures& f);

private:
	struct SizeData {
		Fft drone;
		Fft field;
		std::vector<float> window;
		float magNorm;  // bin magnitude -> amplitude of a full-scale sine
		float outScale; // inverse of magNorm, ifft gain, window overlap
	};
	std::array<SizeData, kSizes.size()> sizes;
	unsigned int sizeIdx = 1;
	float sampleRate = 44100;
	std::vector<float> mask;     // smoothed drone magnitudes
	std::vector<float> levelSm;  // carrier magnitudes, smoothed over ~50 ms (power)
	std::vector<float> floorEst; // tracked carrier noise floor, of levelSm
	std::vector<float> prevMag;  // carrier magnitudes of the previous frame
	std::vector<float> magDrone;
	std::vector<float> shaped;   // drone magnitudes after contrast
	std::vector<float> magField;
};
