/*
REFFT: real-time timbre stamp for Pepper on a classic Bela cape.

Audio in L: drone (supplies the magnitudes)
Audio in R: field recording (supplies phase and noise, gets stamped)
Audio out L: dry/wet mix, out R: wet only
CV out 0-7: analysis features, see README.md

The spectral work runs in an auxiliary task, on another core than the audio
thread. Every hop the audio thread marks the end of a frame and schedules it;
the task reads the last N input samples and overlap-adds its output one hop
ahead of the read position, so it has one hop to finish.
*/

#include <Bela.h>
#include <libraries/Scope/Scope.h>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>
#include "Panel.h"
#include "Stamp.h"

constexpr std::array<unsigned int, Panel::kButtons> Panel::kButtonPins;
constexpr std::array<unsigned int, Panel::kLeds> Panel::kLedPins;

enum Knob { kKnobDroneGain, kKnobFieldGain, kKnobWhiten, kKnobContrast, kKnobFloorOnly, kKnobRelease, kKnobTonality, kKnobWetDry };
enum KnobB { kKnobOutGain, kKnobAttack, kKnobFloorSpeed, kKnobEventDb, kKnobDryBalance, kKnobCvSmooth };
enum Button { kButtonShift, kButtonFreeze, kButtonSize, kButtonBypass };
enum Feature { kFeatDrone, kFeatField, kFeatFlatness, kFeatCentroid, kFeatFlux, kFeatEvents, kFeatDroneCentroid, kNumFeatures };

static constexpr unsigned int kBufSize = 32768; // power of two, > 2 * max FFT size
static constexpr unsigned int kBufMask = kBufSize - 1;

static Stamp gStamp;
static Panel gPanel;
static Scope gScope; // in L, in R, out L, out R
static AuxiliaryTask gStampTask;

// written by the audio thread, read by the task
static float gIn[2][kBufSize];
static std::atomic<unsigned int> gFrameEnd {0};
static std::atomic<bool> gBusy {false};
static std::atomic<bool> gFreeze {false};
static std::atomic<float> gWhiten {0.7f}, gContrast {1.f}, gFloorOnly {0.f}, gEventDb {12.f},
	gFloorRiseDb {6.f}, gAttackCoef {0.f}, gReleaseCoef {0.f}, gTonality {0.f};
// written by the task, read by the audio thread
static float gOut[kBufSize];
static std::atomic<float> gFeatures[kNumFeatures];
static std::atomic<unsigned int> gFramesDone {0};
static std::atomic<unsigned int> gOverruns {0};

// task scratch
static std::vector<float> gDroneFrame, gFieldFrame, gOutFrame;

// audio thread state
static unsigned int gWrite = 0;
static unsigned int gHopCount = 0;
static unsigned int gPendingSize = 1;
static float gWetFade = 1;
static bool gBypass = false;
static float gBypassGain = 0;
static float gWetEnv = 0;
static float gCv[8] = {};
static unsigned int gLastFrames = 0;
static float gFluxAvg = 0;
static unsigned int gGateBlocks = 0;
static unsigned int gBlockCount = 0;
static unsigned int gSizeShowBlocks = 0;
// LED meters: peak envelopes of in L, in R, out L, out R
enum Meter { kMeterInL, kMeterInR, kMeterOutL, kMeterOutR, kNumMeters };
static float gMeter[kNumMeters] = {};
static constexpr float kMeterFloorDb = -48.f;

static inline float onePoleCoef(float tauSec, float rate)
{
	return tauSec <= 0 ? 1.f : 1.f - expf(-1.f / (tauSec * rate));
}

static inline float levelToCv(float rms)
{
	float db = 20.f * log10f(rms + 1e-9f);
	return std::min(1.f, std::max(0.f, (db + 60.f) / 60.f));
}

static inline float hzToCv(float hz)
{
	if(hz <= 50.f)
		return 0;
	return std::min(1.f, log2f(hz / 50.f) / log2f(10000.f / 50.f));
}

static void stampTask(void*)
{
	const unsigned int end = gFrameEnd.load();
	const unsigned int N = gStamp.getSize();
	const unsigned int hop = gStamp.getHop();
	for(unsigned int n = 0; n < N; ++n)
	{
		unsigned int idx = (end - N + n) & kBufMask;
		gDroneFrame[n] = gIn[0][idx];
		gFieldFrame[n] = gIn[1][idx];
	}
	StampParams p;
	p.whiten = gWhiten;
	p.contrast = gContrast;
	p.floorOnly = gFloorOnly;
	p.eventDb = gEventDb;
	p.floorRiseDb = gFloorRiseDb;
	p.attack = gAttackCoef;
	p.release = gReleaseCoef;
	p.tonality = gTonality;
	p.freeze = gFreeze;
	StampFeatures f;
	gStamp.processFrame(gDroneFrame.data(), gFieldFrame.data(), gOutFrame.data(), p, f);
	const unsigned int start = end + hop;
	for(unsigned int n = 0; n < N; ++n)
		gOut[(start + n) & kBufMask] += gOutFrame[n];
	gFeatures[kFeatDrone] = f.droneRms;
	gFeatures[kFeatField] = f.fieldRms;
	gFeatures[kFeatFlatness] = f.fieldFlatness;
	gFeatures[kFeatCentroid] = f.fieldCentroid;
	gFeatures[kFeatFlux] = f.fieldFlux;
	gFeatures[kFeatEvents] = f.eventAmount;
	gFeatures[kFeatDroneCentroid] = f.droneCentroid;
	++gFramesDone;
	gBusy = false;
}

bool setup(BelaContext* context, void* userData)
{
	if(context->audioInChannels < 2 || context->audioOutChannels < 2)
	{
		fprintf(stderr, "REFFT needs stereo audio I/O\n");
		return false;
	}
	if(context->analogFrames != context->audioFrames)
	{
		fprintf(stderr, "REFFT expects uniform analog sample rate (see settings.json)\n");
		return false;
	}
	if(gStamp.setup(context->audioSampleRate))
	{
		fprintf(stderr, "Stamp setup failed\n");
		return false;
	}
	gPendingSize = gStamp.getSizeIndex();
	const unsigned int maxN = gStamp.getMaxSize();
	gDroneFrame.resize(maxN);
	gFieldFrame.resize(maxN);
	gOutFrame.resize(maxN);
	for(auto& f : gFeatures)
		f = 0;

	std::array<std::array<float, Panel::kKnobs>, Panel::kLayers> defaults {};
	defaults[0] = {{ 0.5f, 0.5f, 0.7f, 0.33f, 0.f, 0.f, 0.f, 0.5f }};
	defaults[1] = {{ 0.5f, 0.f, 0.3f, 0.33f, 0.f, 0.3f, 0.f, 0.f }};
	gPanel.setup(context, defaults);
	gScope.setup(4, context->audioSampleRate);

	if((gStampTask = Bela_createAuxiliaryTask(stampTask, 90, "refft-stamp")) == 0)
		return false;
	rt_printf("REFFT: FFT %u, hop %u, latency %.1f ms\n", gStamp.getSize(), gStamp.getHop(),
			1000.f * (gStamp.getSize() + gStamp.getHop()) / context->audioSampleRate);
	return true;
}

void render(BelaContext* context, void* userData)
{
	const float fs = context->audioSampleRate;
	gPanel.process(context);

	// buttons
	if(gPanel.pressed(kButtonShift) || gPanel.released(kButtonShift))
		gPanel.setLayer(gPanel.isHeld(kButtonShift) ? 1 : 0);
	if(gPanel.pressed(kButtonFreeze))
		gFreeze = !gFreeze;
	if(gPanel.pressed(kButtonSize))
	{
		gPendingSize = (gPendingSize + 1) % Stamp::kSizes.size();
		gSizeShowBlocks = 0;
	}
	if(gPanel.pressed(kButtonBypass))
		gBypass = !gBypass;
	for(unsigned int b = 0; b < Panel::kButtons; ++b)
		if(gPanel.pressed(b))
			rt_printf("button %u pressed\n", b + 1);

	// parameters
	const float hopSec = gStamp.getHop() / fs;
	auto A = [](unsigned int k) { return gPanel.get(0, k); };
	auto B = [](unsigned int k) { return gPanel.get(1, k); };
	float droneGain = 4.f * A(kKnobDroneGain) * A(kKnobDroneGain);
	float fieldGain = 4.f * A(kKnobFieldGain) * A(kKnobFieldGain);
	gWhiten = A(kKnobWhiten);
	gContrast = 0.5f * powf(8.f, A(kKnobContrast));
	gFloorOnly = A(kKnobFloorOnly);
	gReleaseCoef = onePoleCoef(5.f * A(kKnobRelease) * A(kKnobRelease), 1.f / hopSec);
	gTonality = A(kKnobTonality);
	float wetDry = A(kKnobWetDry);
	float outGain = 4.f * B(kKnobOutGain) * B(kKnobOutGain);
	gAttackCoef = onePoleCoef(2.f * B(kKnobAttack) * B(kKnobAttack), 1.f / hopSec);
	gFloorRiseDb = 0.5f + 30.f * B(kKnobFloorSpeed) * B(kKnobFloorSpeed);
	gEventDb = 3.f + 27.f * B(kKnobEventDb);
	float dryBalance = B(kKnobDryBalance);
	float cvCoef = onePoleCoef(0.001f + 0.5f * B(kKnobCvSmooth) * B(kKnobCvSmooth), fs / context->audioFrames);

	const float wetGain = sinf(wetDry * (float)M_PI_2);
	const float dryGain = cosf(wetDry * (float)M_PI_2);
	const float dryField = cosf(dryBalance * (float)M_PI_2);
	const float dryDrone = sinf(dryBalance * (float)M_PI_2);
	const float fadeStep = 1.f / (0.02f * fs);
	const float bypassCoef = onePoleCoef(0.01f, fs);
	const float envCoef = onePoleCoef(0.05f, fs);
	const float meterAttack = onePoleCoef(0.001f, fs);
	const float meterRelease = onePoleCoef(0.15f, fs);
	auto meter = [&](unsigned int m, float x) {
		float a = fabsf(x);
		gMeter[m] += (a > gMeter[m] ? meterAttack : meterRelease) * (a - gMeter[m]);
	};

	for(unsigned int n = 0; n < context->audioFrames; ++n)
	{
		const float rawDrone = audioRead(context, n, 0);
		const float rawField = audioRead(context, n, 1);
		meter(kMeterInL, rawDrone);
		meter(kMeterInR, rawField);
		float drone = rawDrone * droneGain;
		float field = rawField * fieldGain;
		gIn[0][gWrite] = drone;
		gIn[1][gWrite] = field;
		float wet = gOut[gWrite] * gWetFade;
		gOut[gWrite] = 0;
		gWrite = (gWrite + 1) & kBufMask;

		// FFT size change: fade the wet path out, switch, fade back in
		if(gPendingSize != gStamp.getSizeIndex())
		{
			gWetFade = std::max(0.f, gWetFade - fadeStep);
			if(0 == gWetFade && !gBusy)
			{
				gStamp.setSizeIndex(gPendingSize);
				memset(gOut, 0, sizeof(gOut));
				gHopCount = 0;
				rt_printf("REFFT: FFT %u, hop %u\n", gStamp.getSize(), gStamp.getHop());
			}
		}
		else
		{
			gWetFade = std::min(1.f, gWetFade + fadeStep);
			if(++gHopCount >= gStamp.getHop())
			{
				gHopCount = 0;
				if(gBusy)
					++gOverruns;
				else
				{
					gFrameEnd = gWrite;
					gBusy = true;
					Bela_scheduleAuxiliaryTask(gStampTask);
				}
			}
		}

		gBypassGain += bypassCoef * ((gBypass ? 1.f : 0.f) - gBypassGain);
		float dry = field * dryField + drone * dryDrone;
		float mix = dry * dryGain + wet * wetGain;
		mix = mix * (1.f - gBypassGain) + dry * gBypassGain;
		float wetOut = wet * (1.f - gBypassGain);
		const float outL = mix * outGain;
		const float outR = wetOut * outGain;
		audioWrite(context, n, 0, outL);
		audioWrite(context, n, 1, outR);
		meter(kMeterOutL, outL);
		meter(kMeterOutR, outR);
		gScope.log(rawDrone, rawField, outL, outR);
		gWetEnv += envCoef * (fabsf(wetOut) - gWetEnv);
	}

	// CV outs, updated per block
	float target[8];
	target[0] = levelToCv(gFeatures[kFeatDrone]);
	target[1] = levelToCv(gFeatures[kFeatField]);
	target[2] = std::min(1.f, gFeatures[kFeatFlatness].load());
	target[3] = hzToCv(gFeatures[kFeatCentroid]);
	unsigned int frames = gFramesDone;
	if(frames != gLastFrames)
	{
		gLastFrames = frames;
		float flux = gFeatures[kFeatFlux];
		if(flux > 2.f * gFluxAvg + 0.02f)
			gGateBlocks = (unsigned int)(0.02f * fs / context->audioFrames);
		gFluxAvg += 0.1f * (flux - gFluxAvg);
	}
	target[4] = gGateBlocks ? 1.f : 0.f;
	if(gGateBlocks)
		--gGateBlocks;
	target[5] = std::min(1.f, gFeatures[kFeatEvents].load());
	target[6] = levelToCv(gWetEnv * 1.4142f);
	target[7] = hzToCv(gFeatures[kFeatDroneCentroid]);
	for(unsigned int c = 0; c < 8 && c < context->analogOutChannels; ++c)
	{
		// the gate stays sharp, the rest follow the smoothing knob
		gCv[c] = 4 == c ? target[c] : gCv[c] + cvCoef * (target[c] - gCv[c]);
		analogWrite(context, 0, c, gCv[c]);
	}

	// LEDs 0-7: two per meter (in L, in R, out L, out R), on a dB scale from
	// kMeterFloorDb to 0 dBFS: the first LED covers the lower half, the
	// second the upper half. For a second after a size change they show the
	// FFT size instead. LED 8: freeze. LED 9: shift / bypass.
	const unsigned int showBlocks = (unsigned int)(fs / context->audioFrames); // 1 s
	if(gSizeShowBlocks < showBlocks)
	{
		++gSizeShowBlocks;
		gPanel.setBar((gPendingSize + 1) / 8.f, 0, 8);
	}
	else
	{
		for(unsigned int m = 0; m < kNumMeters; ++m)
		{
			float db = 20.f * log10f(gMeter[m] + 1e-9f);
			float v = std::min(1.f, std::max(0.f, 1.f - db / kMeterFloorDb));
			float lo = std::min(1.f, 2.f * v);
			float hi = std::max(0.f, 2.f * v - 1.f);
			// squared: LED light output is linear in duty, perceived brightness isn't
			gPanel.setBrightness(2 * m, lo * lo);
			gPanel.setBrightness(2 * m + 1, hi * hi);
		}
	}
	gPanel.setLed(8, gFreeze);
	bool blink = (gBlockCount / (unsigned int)(0.25f * fs / context->audioFrames)) & 1;
	gPanel.setLed(9, gPanel.getLayer() == 1 || (gBypass && blink));
	++gBlockCount;

	static unsigned int lastOverruns = 0;
	unsigned int overruns = gOverruns;
	if(overruns != lastOverruns && 0 == gBlockCount % (unsigned int)(2.f * fs / context->audioFrames))
	{
		rt_printf("REFFT: %u frames skipped (task too slow)\n", overruns - lastOverruns);
		lastOverruns = overruns;
	}
}

void cleanup(BelaContext* context, void* userData)
{
}
