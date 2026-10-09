#pragma once
// Field-recording player on audio out L. It doesn't feed the stamp: patch
// out L into in R for that.
// Files stream from disk, so any length plays. Two voices, each with its own
// ring buffer that a disk thread keeps filled; the voice that isn't playing
// waits armed at the region start, so a loop point is an instant
// equal-power crossfade. A seek arms that voice somewhere else and crossfades
// once it holds a few thousand frames (a few ms).
//
// A loader thread scans for files, opens them, and analyses each new file in
// the background: its loudness (for normalise), peak and a waveform overview
// for the GUI, all cached in .cache/ so a file is read through only once.
// Normalise sets each file's typical (median) level to the same loudness; a
// lookahead limiter on the output catches the peaks this pushes up.
//
// Parameters are atomics that any thread may write (the GUI callback, the
// loader when it restores sampler.cfg). The audio thread only reads and
// writes atomics and the ring buffers: it never allocates, locks or does I/O.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Clip {
	std::string name;           // path relative to the project folder
	float sampleRate = 44100;
	int64_t frames = 0;
	unsigned int channels = 1;
	int channelMode = 0;        // Sampler::Channel; a change makes a new clip
	unsigned int id = 0;
	std::atomic<float> peak {0};        // of the mono signal; 0 until analysed
	std::atomic<float> medianDb {0};    // median 100 ms RMS level, dBFS
	std::atomic<float> normDb {0};      // normalising gain; valid once analysed == 1
	std::atomic<float> analysed {0};    // analysis progress, 0..1
	std::vector<float> overviewData;    // clip id, then min and max per bin (normalised)
	std::atomic<const std::vector<float>*> overview {nullptr}; // set when complete
};

// Mono audio from one position of one clip. The audio thread requests a
// start (clip, frame, new generation); the disk thread seeks, then fills.
struct Stream {
	static constexpr unsigned int kRing = 1 << 17; // 3 s at 44.1 kHz
	static constexpr unsigned int kMask = kRing - 1;
	float ring[kRing];
	// request, written by the audio thread
	std::atomic<const Clip*> reqClip {nullptr};
	std::atomic<int64_t> reqStart {0};
	std::atomic<unsigned int> reqGen {0};
	// fill state, written by the disk thread, valid when fillGen == reqGen
	std::atomic<unsigned int> fillGen {0};
	std::atomic<int64_t> written {0};    // frames from reqStart
	std::atomic<bool> atEnd {false};     // the file ends after `written`
	// written by the audio thread: frames it no longer needs
	std::atomic<int64_t> consumed {0};
};

// Text written by the loader and sent to the GUI by the audio thread: two
// slots, the writer fills the one not being published, then bumps seq.
struct SharedText {
	static constexpr unsigned int kMax = 8192;
	char buf[2][kMax];
	unsigned int len[2] = {};
	std::atomic<unsigned int> seq {0};
	void write(const std::string& s);
	const char* data(unsigned int s) const { return buf[s & 1]; }
	unsigned int size(unsigned int s) const { return len[s & 1]; }
};

// Peak limiter: the gain is the minimum of the needed gain over a lookahead
// window, released by a one-pole, then box-averaged over the same window;
// the signal is delayed by the window, so the gain is down before a peak
// arrives and the ramp has no corners.
class Limiter {
public:
	static constexpr unsigned int kLook = 64; // 1.5 ms at 44.1 kHz
	void setup(float fs, float ceilingDb, float releaseSec);
	float process(float x);
	float reductionDb() const; // current gain reduction, ≤ 0
private:
	float ceiling = 1, relCoef = 0;
	float delay[kLook] = {}, need[kLook] = {}, env[kLook] = {};
	unsigned int pos = 0;
	float held = 1;
	double envSum = kLook;
	float gain = 1;
};

class Sampler {
public:
	enum Channel { kChannelMix, kChannelLeft, kChannelRight };
	enum LoaderState { kLoaderIdle, kLoaderLoading, kLoaderError };
	static constexpr unsigned int kOverviewBins = 1024;
	static constexpr float kTargetDb = -26.f;   // normalised median level
	static constexpr float kCeilingDb = -1.f;   // limiter ceiling
	static constexpr float kMaxLimitDb = 12.f;  // normalise never makes the limiter take more than
	                                            // this off the loudest 0.1% of a file

	// non-RT
	bool setup(const std::string& projectDir, float audioSampleRate);
	void cleanup();
	void rescan();
	void load(int index);    // index into the last scanned file list
	void reload();           // same file again, e.g. after a channel change
	void touch();            // settings changed: save sampler.cfg soon

	// parameters, from any thread
	std::atomic<bool> play {false};      // wanted state; cleared at the end of a one-shot
	std::atomic<bool> loop {true};
	std::atomic<bool> normalize {true};  // play at full scale instead of the file's level
	std::atomic<float> gainDb {0.f};
	std::atomic<float> xfadeMs {50.f};
	std::atomic<float> start {0.f};      // region, as fractions of the clip
	std::atomic<float> end {1.f};
	std::atomic<int> channel {kChannelMix};
	void seek(float pos) { seekTo = pos; }
	void retrigger() { ++retriggers; }

	// status, for the GUI
	std::atomic<int> loaderState {kLoaderIdle};
	std::atomic<int> fileIndex {-1};
	std::atomic<unsigned int> listId {0};
	std::atomic<unsigned int> underruns {0};
	std::atomic<float> limitDb {0};  // limiter gain reduction, peak over the last block
	SharedText files;    // "listId\nname\nname..."
	SharedText message;  // last loader message

	// audio thread
	void process(float* out, unsigned int frames);
	const Clip* clip() const { return current; }
	bool running() const { return isRunning; }
	float playhead() const; // fraction of the clip

private:
	struct Voice {
		Stream* stream = nullptr;
		const Clip* clip = nullptr;
		unsigned int gen = 0;
		int64_t start = 0;   // clip frame where the stream starts
		double rel = 0;      // read position from start
		float ramp = 0;      // 0..1, weighted with sin() for equal power
		float target = 0;
		float level = 1;     // smoothed normalisation gain
		bool armed = false;  // holds a request that hasn't been played through
	};
	void arm(Voice& v, const Clip* c, int64_t pos);
	bool ready(const Voice& v) const;
	float read(const Voice& v);
	void stop();

	void loaderLoop();
	void diskLoop();
	void doScan();
	void doLoad(const std::string& name, bool keepRegion);
	void analyse(Clip* c);
	void loadConfig();
	void saveConfig();

	std::string dir;
	float fs = 44100;

	// audio thread state
	Stream streams[2];
	Voice voices[2];
	unsigned int active = 0;
	bool isRunning = false;
	const Clip* current = nullptr;
	float gainLin = 1;
	Limiter limiter;
	double headPos = 0;      // clip frames
	double pendingJump = -1; // clip frame to crossfade to once the idle voice is ready
	unsigned int seenRetriggers = 0;

	// hand-over between threads
	std::atomic<Clip*> pending {nullptr};
	std::atomic<float> seekTo {-1.f};
	std::atomic<unsigned int> retriggers {0};
	std::atomic<bool> dirty {false};
	std::atomic<bool> quit {false};
	std::atomic<bool> loadPending {false};

	// loader thread
	std::thread loader;
	std::thread disk;
	std::mutex mtx;
	std::condition_variable cv;
	bool scanRequested = false;
	std::string loadRequest;      // empty: none
	bool loadKeepsRegion = false;
	std::vector<std::string> fileList;
	std::string loadedName;
	unsigned int nextClipId = 1;
	std::vector<std::unique_ptr<Clip>> clips; // all clips ever opened: small, freed at cleanup
};
