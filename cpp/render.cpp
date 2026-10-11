/*
REFFT: real-time timbre stamp for Pepper on a classic Bela cape.

Audio in L: drone (supplies the magnitudes)
Audio in R: field recording (supplies phase and noise, gets stamped)
Audio out L: the field-recording sampler (browser GUI, sketch.js); patch it
             into in R to stamp it. Nothing is routed internally.
Audio out R: the stamp (dry/wet mix)
CV out 0-7: analysis features, see README.md

The spectral work runs in an auxiliary task, on another core than the audio
thread. Every hop the audio thread marks the end of a frame and schedules it;
the task reads the last N input samples and overlap-adds its output one hop
ahead of the read position, so it has one hop to finish.
*/

#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/Scope/Scope.h>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>
#include "Limiter.h"
#include "Panel.h"
#include "Sampler.h"
#include "Stamp.h"

constexpr std::array<unsigned int, Panel::kButtons> Panel::kButtonPins;
constexpr std::array<unsigned int, Panel::kLeds> Panel::kLedPins;
constexpr std::array<float, Panel::kKnobs> Panel::kKnobTop;
constexpr float Panel::kKnobTopMargin;

enum Knob { kKnobDroneGain, kKnobFieldGain, kKnobWhiten, kKnobContrast, kKnobFloorOnly, kKnobRelease, kKnobTonality, kKnobWetDry };
enum KnobB { kKnobOutGain, kKnobAttack, kKnobFloorSpeed, kKnobEventDb, kKnobDryBalance, kKnobCvSmooth, kKnobEventLo, kKnobEventHi };
enum Button { kButtonShift, kButtonFreeze, kButtonSize, kButtonBypass };
enum Feature { kFeatDrone, kFeatField, kFeatFlatness, kFeatCentroid, kFeatFlux, kFeatEvents, kFeatDroneCentroid, kFeatMatchDb, kNumFeatures };

static constexpr unsigned int kBufSize = 32768; // power of two, > 2 * max FFT size
static constexpr unsigned int kBufMask = kBufSize - 1;

static Stamp gStamp;
static Panel gPanel;
static Scope gScope; // in L, in R, out L (sampler), out R (stamp)
static AuxiliaryTask gStampTask;
static Sampler gSampler;
static Gui gGui;
static std::vector<float> gSamplerOut;

// written by the audio thread, read by the task
static float gIn[2][kBufSize];
static std::atomic<unsigned int> gFrameEnd {0};
static std::atomic<bool> gBusy {false};
static std::atomic<bool> gFreeze {false};
static std::atomic<float> gWhiten {0.7f}, gContrast {1.f}, gFloorOnly {0.f}, gEventDb {12.f},
	gFloorRiseDb {6.f}, gAttackCoef {0.f}, gReleaseCoef {0.f}, gTonality {0.f},
	gEventLoHz {2000.f}, gEventHiHz {10000.f};
// written by the task, read by the audio thread
static float gOut[kBufSize];
static std::atomic<float> gFeatures[kNumFeatures];
static std::atomic<float> gBandDb[kDisplayBands], gBandEvent[kDisplayBands];
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
// out R auto level: brings the mix's loudness to kAutoTargetDb so the stamp
// meets modular levels however quiet the recording reaching in R is
static constexpr float kAutoTargetDb = -14.f;
static constexpr float kAutoMinDb = -12.f, kAutoMaxDb = 48.f;
static constexpr float kAutoGateDb = -70.f; // below this the gain holds
static float gAutoPow = 0;    // smoothed mix power
static float gAutoDb = 0;     // applied gain
static Limiter gLimiterR;
static float gLimitRPeakDb = 0;
static float gWetEnv = 0;
static float gCv[8] = {};
static unsigned int gLastFrames = 0;
static float gFluxAvg = 0;
static unsigned int gGateBlocks = 0;
static unsigned int gBlockCount = 0;
static unsigned int gSizeShowBlocks = 0;
// meters: peak envelopes of in L, in R, out L (sampler), out R (stamp)
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

// GUI link. Buffers sent to the browser:
// 0 status (floats, layout below: the sampler, the meters, then the panel:
// layer, button states, every stored knob value, whether each has been
// picked up, where the physical knobs are, the CV outs, then the sampler's
// loudness and limiter), 1 file list (text: list id, then one
// name per line), 2 waveform overview of the loaded clip (floats: clip id,
// then min, max per bin), 3 last loader message (text), 4 the field's
// spectrum (floats: event band low and high Hz, then dBFS and event share
// per display band, see Stamp.h).
// Everything is sent from the audio thread; the browser asks for a full
// resend with /sync whenever its copies are out of date.
enum GuiStatus {
	kStClip, kStRunning, kStHead, kStStart, kStEnd, kStGain, kStLoop, kStXfade,
	kStChannel, kStNormalize, kStSeconds, kStRate, kStLoader, kStFile,
	kStListId, kStChannels, kStPeakDb, kStAnalysed, kStPlay, kStUnderruns,
	kStMeters,
	kStLayer = kStMeters + kNumMeters, kStFreeze, kStFft, kStBypass, kStTouched,
	kStKnobs, // layer A then B
	kStPicked = kStKnobs + Panel::kLayers * Panel::kKnobs,
	kStPots = kStPicked + Panel::kLayers * Panel::kKnobs,
	kStCv = kStPots + Panel::kKnobs,
	kStMedianDb = kStCv + 8, kStNormDb, kStLimitDb, kStMatchDb, kStAutoDb, kStLimitRDb,
	kStSize
};
static std::atomic<bool> gGuiSync {true};
static unsigned int gGuiBlocks = 0;
static float gLimitPeakDb = 0; // most gain reduction since the last status
static unsigned int gSentFiles = ~0u, gSentMessage = ~0u, gSentClip = 0;

static bool onGuiControl(JSONObject& root, void*)
{
	auto a = root.find(L"address");
	if(a == root.end() || !a->second || !a->second->IsString())
		return false;
	const std::wstring w = a->second->AsString();
	const std::string addr(w.begin(), w.end());
	float v = 0;
	auto it = root.find(L"value");
	if(it != root.end() && it->second)
	{
		if(it->second->IsNumber())
			v = it->second->AsNumber();
		else if(it->second->IsBool())
			v = it->second->AsBool();
	}
	Sampler& s = gSampler;
	auto clampi = [](float x, int hi) { return std::min(hi, std::max(0, (int)lrintf(x))); };
	if("/sync" == addr)
		gGuiSync = true;
	else if("/rescan" == addr)
		s.rescan();
	else if("/load" == addr)
		s.load((int)lrintf(v));
	else if("/seek" == addr)
		s.seek(std::min(1.f, std::max(0.f, v)));
	else if("/retrigger" == addr)
		s.retrigger();
	else
	{
		if("/play" == addr) s.play = v > 0.5f;
		else if("/loop" == addr) s.loop = v > 0.5f;
		else if("/normalize" == addr) s.normalize = v > 0.5f;
		else if("/gain" == addr) s.gainDb = std::min(24.f, std::max(-60.f, v));
		else if("/xfade" == addr) s.xfadeMs = std::min(2000.f, std::max(2.f, v));
		else if("/start" == addr) s.start = std::min(1.f, std::max(0.f, v));
		else if("/end" == addr) s.end = std::min(1.f, std::max(0.f, v));
		else if("/channel" == addr)
		{
			s.channel = clampi(v, Sampler::kChannelRight);
			s.reload();
		}
		else
			return false;
		s.touch();
	}
	return false;
}

static inline float toDb(float x)
{
	return 20.f * log10f(x + 1e-9f);
}

static void sendGui(BelaContext* context)
{
	const bool sync = gGuiSync.exchange(false);
	const Sampler& s = gSampler;
	const Clip* c = s.clip();
	if(++gGuiBlocks >= (unsigned int)(0.033f * context->audioSampleRate / context->audioFrames) || sync)
	{
		gGuiBlocks = 0;
		float st[kStSize] = {};
		st[kStClip] = c ? c->id : 0;
		st[kStRunning] = s.running();
		st[kStHead] = s.playhead();
		st[kStStart] = s.start;
		st[kStEnd] = s.end;
		st[kStGain] = s.gainDb;
		st[kStLoop] = s.loop;
		st[kStXfade] = s.xfadeMs;
		st[kStChannel] = s.channel;
		st[kStNormalize] = s.normalize;
		st[kStSeconds] = c ? c->frames / c->sampleRate : 0;
		st[kStRate] = c ? c->sampleRate : 0;
		st[kStLoader] = s.loaderState;
		st[kStFile] = s.fileIndex;
		st[kStListId] = s.listId;
		st[kStChannels] = c ? c->channels : 0;
		st[kStPeakDb] = c && c->peak > 0 ? toDb(c->peak) : -180;
		st[kStAnalysed] = c ? c->analysed.load() : 0;
		st[kStPlay] = s.play;
		st[kStUnderruns] = s.underruns;
		for(unsigned int m = 0; m < kNumMeters; ++m)
			st[kStMeters + m] = toDb(gMeter[m]);
		st[kStLayer] = gPanel.getLayer();
		st[kStFreeze] = gFreeze;
		st[kStFft] = Stamp::kSizes[gPendingSize];
		st[kStBypass] = gBypass;
		st[kStTouched] = gPanel.recentlyTouched((unsigned int)(context->audioSampleRate / context->audioFrames));
		for(unsigned int l = 0; l < Panel::kLayers; ++l)
		{
			for(unsigned int k = 0; k < Panel::kKnobs; ++k)
			{
				st[kStKnobs + l * Panel::kKnobs + k] = gPanel.get(l, k);
				st[kStPicked + l * Panel::kKnobs + k] = gPanel.isPicked(l, k);
			}
		}
		for(unsigned int k = 0; k < Panel::kKnobs; ++k)
			st[kStPots + k] = gPanel.knobValue(k);
		for(unsigned int c = 0; c < 8; ++c)
			st[kStCv + c] = gCv[c];
		const bool analysed = c && c->analysed >= 1.f;
		st[kStMedianDb] = analysed ? c->medianDb.load() : -180;
		st[kStNormDb] = analysed ? c->normDb.load() : 0;
		st[kStLimitDb] = gLimitPeakDb;
		st[kStMatchDb] = gFeatures[kFeatMatchDb];
		st[kStAutoDb] = gAutoDb;
		st[kStLimitRDb] = gLimitRPeakDb;
		gLimitRPeakDb = 0;
		gLimitPeakDb = 0;
		gGui.sendBuffer(0, st);

		float bands[2 + 2 * kDisplayBands];
		bands[0] = gEventLoHz;
		bands[1] = gEventHiHz;
		for(unsigned int b = 0; b < kDisplayBands; ++b)
		{
			bands[2 + b] = gBandDb[b];
			bands[2 + kDisplayBands + b] = gBandEvent[b];
		}
		gGui.sendBuffer(4, bands);
	}
	auto sendText = [&](unsigned int id, const SharedText& t, unsigned int& sent) {
		const unsigned int seq = t.seq.load(std::memory_order_acquire);
		if((seq == sent && !sync) || !t.size(seq))
			return;
		gGui.sendBuffer(id, t.data(seq), t.size(seq));
		if(t.seq.load() == seq) // otherwise it may have changed while sending: again next block
			sent = seq;
	};
	sendText(1, s.files, gSentFiles);
	sendText(3, s.message, gSentMessage);
	const std::vector<float>* overview = c ? c->overview.load(std::memory_order_acquire) : nullptr;
	if(overview && (c->id != gSentClip || sync))
	{
		gGui.sendBuffer(2, overview->data(), overview->size());
		gSentClip = c->id;
	}
}

// 100 Hz…4 kHz on a log scale: where the field recordings' centroids sit
static inline float hzToCv(float hz)
{
	if(hz <= 100.f)
		return 0;
	return std::min(1.f, log2f(hz * 0.01f) / log2f(40.f));
}

// event band knobs: 100 Hz…12.8 kHz, log
static inline float knobToHz(float v)
{
	return 100.f * powf(128.f, v);
}

// Spectral features of a signal below the level CVs' range (−60 dB) describe
// the codec's noise, not the signal: fade them out over the 6 dB below it.
static inline float levelGate(float rms)
{
	float db = 20.f * log10f(rms + 1e-9f);
	return std::min(1.f, std::max(0.f, (db + 66.f) / 6.f));
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
	p.eventLoHz = gEventLoHz;
	p.eventHiHz = gEventHiHz;
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
	gFeatures[kFeatMatchDb] = f.matchDb;
	for(unsigned int b = 0; b < kDisplayBands; ++b)
	{
		gBandDb[b] = f.bandDb[b];
		gBandEvent[b] = f.bandEvent[b];
	}
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
	// B7, B8: event band 2…10 kHz
	defaults[1] = {{ 0.5f, 0.f, 0.3f, 0.33f, 0.f, 0.3f, 0.6174f, 0.9491f }};
	gPanel.setup(context, defaults);
	gScope.setup(4, context->audioSampleRate);
	gLimiterR.setup(context->audioSampleRate, -1.f, 0.1f);

	gSamplerOut.resize(context->audioFrames);
	gSampler.setup(std::string("/root/Bela/projects/") + context->projectName, context->audioSampleRate);
	gGui.setup(context->projectName);
	gGui.setControlDataCallback(onGuiControl, nullptr);

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
	// a trim on top of the auto level: off, 0 dB at centre, +12 dB
	float outGain = 4.f * B(kKnobOutGain) * B(kKnobOutGain);
	gAttackCoef = onePoleCoef(2.f * B(kKnobAttack) * B(kKnobAttack), 1.f / hopSec);
	gFloorRiseDb = 0.5f + 30.f * B(kKnobFloorSpeed) * B(kKnobFloorSpeed);
	gEventDb = 3.f + 27.f * B(kKnobEventDb);
	float dryBalance = B(kKnobDryBalance);
	float cvCoef = onePoleCoef(0.001f + 0.5f * B(kKnobCvSmooth) * B(kKnobCvSmooth), fs / context->audioFrames);
	{
		// either knob can be the lower edge
		const float lo = knobToHz(B(kKnobEventLo)), hi = knobToHz(B(kKnobEventHi));
		gEventLoHz = std::min(lo, hi);
		gEventHiHz = std::max(lo, hi);
	}

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

	// auto level for out R, per block: follow the mix's power over ~0.5 s,
	// then move the gain towards the target, down within ~0.2 s, up over
	// ~2 s, so single events don't pump it; hold it in silence
	const float blockSec = context->audioFrames / fs;
	const float autoDownCoef = onePoleCoef(0.2f, 1.f / blockSec);
	const float autoUpCoef = onePoleCoef(2.f, 1.f / blockSec);
	const float autoPowCoef = onePoleCoef(0.5f, 1.f / blockSec);
	const float autoStart = powf(10.f, gAutoDb / 20.f);
	float autoEnd = autoStart;
	{
		const float levelDb = 10.f * log10f(gAutoPow + 1e-20f);
		if(levelDb > kAutoGateDb)
		{
			const float want = std::min(kAutoMaxDb, std::max(kAutoMinDb, kAutoTargetDb - levelDb));
			gAutoDb += (want < gAutoDb ? autoDownCoef : autoUpCoef) * (want - gAutoDb);
		}
		autoEnd = powf(10.f, gAutoDb / 20.f);
	}
	float mixSq = 0;

	gSampler.process(gSamplerOut.data(), context->audioFrames);
	gLimitPeakDb = std::min(gLimitPeakDb, gSampler.limitDb.load());

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
		const float outL = gSamplerOut[n];
		mixSq += mix * mix;
		const float autoGain = autoStart + (autoEnd - autoStart) * n / context->audioFrames;
		const float outR = gLimiterR.process(mix * autoGain * outGain);
		gLimitRPeakDb = std::min(gLimitRPeakDb, gLimiterR.reductionDb());
		audioWrite(context, n, 0, outL);
		audioWrite(context, n, 1, outR);
		meter(kMeterOutL, outL);
		meter(kMeterOutR, outR);
		gScope.log(rawDrone, rawField, outL, outR);
		gWetEnv += envCoef * (fabsf(wetOut) - gWetEnv);
	}
	gAutoPow += autoPowCoef * (mixSq / context->audioFrames - gAutoPow);

	// CV outs, updated per block
	float target[8];
	target[0] = levelToCv(gFeatures[kFeatDrone]);
	target[1] = levelToCv(gFeatures[kFeatField]);
	const float fieldGate = levelGate(gFeatures[kFeatField]);
	target[2] = fieldGate * std::min(1.f, gFeatures[kFeatFlatness].load());
	target[3] = fieldGate * hzToCv(gFeatures[kFeatCentroid]);
	unsigned int frames = gFramesDone;
	if(frames != gLastFrames)
	{
		gLastFrames = frames;
		float flux = gFeatures[kFeatFlux];
		if(flux > 2.f * gFluxAvg + 0.02f && fieldGate > 0.5f)
			gGateBlocks = (unsigned int)(0.02f * fs / context->audioFrames);
		gFluxAvg += 0.1f * (flux - gFluxAvg);
	}
	target[4] = gGateBlocks ? 1.f : 0.f;
	if(gGateBlocks)
		--gGateBlocks;
	target[5] = fieldGate * std::min(1.f, gFeatures[kFeatEvents].load());
	target[6] = levelToCv(gWetEnv * 1.4142f);
	target[7] = levelGate(gFeatures[kFeatDrone]) * hzToCv(gFeatures[kFeatDroneCentroid]);
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

	sendGui(context);

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
	gGui.cleanup();
	gSampler.cleanup();
}
