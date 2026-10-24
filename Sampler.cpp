#include "Sampler.h"
#include <libraries/sndfile/sndfile.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>

static constexpr const char* kConfigFile = "sampler.cfg";
static constexpr int64_t kReadyFrames = 4096;   // a stream this full can start playing
static constexpr unsigned int kDiskChunk = 4096;

constexpr unsigned int Stream::kRing;
constexpr unsigned int Stream::kMask;
constexpr unsigned int SharedText::kMax;
constexpr unsigned int Sampler::kOverviewBins;
constexpr float Sampler::kTargetDb;
constexpr float Sampler::kCeilingDb;
constexpr float Sampler::kMaxLimitDb;

void SharedText::write(const std::string& s)
{
	const unsigned int next = seq.load() + 1;
	const unsigned int slot = next & 1;
	const unsigned int n = std::min((unsigned int)s.size(), kMax);
	memcpy(buf[slot], s.data(), n);
	len[slot] = n;
	seq.store(next, std::memory_order_release);
}

// One sample of the clip's mono signal from an interleaved frame.
static inline float mono(const float* fr, unsigned int ch, int mode)
{
	if(ch < 2)
		return fr[0];
	if(Sampler::kChannelLeft == mode)
		return fr[0];
	if(Sampler::kChannelRight == mode)
		return fr[1];
	float sum = 0;
	for(unsigned int k = 0; k < ch; ++k)
		sum += fr[k];
	return sum / ch;
}

bool Sampler::setup(const std::string& projectDir, float audioSampleRate)
{
	dir = projectDir;
	fs = audioSampleRate;
	limiter.setup(fs, kCeilingDb, 0.1f);
	for(unsigned int v = 0; v < 2; ++v)
		voices[v].stream = &streams[v];
	loadConfig();
	disk = std::thread(&Sampler::diskLoop, this);
	loader = std::thread(&Sampler::loaderLoop, this);
	return true;
}

void Sampler::cleanup()
{
	{
		std::lock_guard<std::mutex> lock(mtx);
		quit = true;
	}
	cv.notify_one();
	if(loader.joinable())
		loader.join();
	if(disk.joinable())
		disk.join();
	saveConfig();
}

void Sampler::rescan()
{
	{
		std::lock_guard<std::mutex> lock(mtx);
		scanRequested = true;
	}
	cv.notify_one();
}

void Sampler::load(int index)
{
	{
		std::lock_guard<std::mutex> lock(mtx);
		if(index < 0 || index >= (int)fileList.size())
			return;
		loadRequest = fileList[index];
		loadKeepsRegion = false;
		loadPending = true;
	}
	cv.notify_one();
}

void Sampler::reload()
{
	{
		std::lock_guard<std::mutex> lock(mtx);
		if(loadedName.empty())
			return;
		loadRequest = loadedName;
		loadKeepsRegion = true;
		loadPending = true;
	}
	cv.notify_one();
}

void Sampler::touch()
{
	dirty = true;
}

float Sampler::playhead() const
{
	return current && current->frames ? headPos / current->frames : 0;
}

// --- audio thread ---------------------------------------------------------

static inline float levelFor(const Clip* c, bool normalize)
{
	if(!normalize || c->analysed.load(std::memory_order_relaxed) < 1.f)
		return 1.f;
	return powf(10.f, c->normDb.load(std::memory_order_relaxed) / 20.f);
}

// Ask the disk thread for `c` from frame `pos` on this voice's stream.
void Sampler::arm(Voice& v, const Clip* c, int64_t pos)
{
	Stream& s = *v.stream;
	s.consumed.store(0, std::memory_order_relaxed);
	s.reqClip.store(c, std::memory_order_relaxed);
	s.reqStart.store(pos, std::memory_order_relaxed);
	v.gen = s.reqGen.load(std::memory_order_relaxed) + 1;
	s.reqGen.store(v.gen, std::memory_order_release);
	v.clip = c;
	v.start = pos;
	v.rel = 0;
	v.ramp = 0;
	v.target = 0;
	v.level = levelFor(c, normalize);
	v.armed = true;
}

bool Sampler::ready(const Voice& v) const
{
	const Stream& s = *v.stream;
	return v.armed && s.fillGen.load(std::memory_order_acquire) == v.gen
		&& (s.written.load(std::memory_order_acquire) >= kReadyFrames || s.atEnd.load());
}

// Cubic Hermite interpolation; before the start or past the end reads silence.
float Sampler::read(const Voice& v)
{
	const Stream& s = *v.stream;
	const int64_t written = s.written.load(std::memory_order_acquire);
	const double fl = floor(v.rel);
	const int64_t i = (int64_t)fl;
	const float t = (float)(v.rel - fl);
	if(i + 2 >= written && !s.atEnd.load(std::memory_order_relaxed))
		underruns.fetch_add(1, std::memory_order_relaxed);
	auto at = [&](int64_t k) -> float { return k >= 0 && k < written ? s.ring[k & Stream::kMask] : 0.f; };
	const float xm1 = at(i - 1), x0 = at(i), x1 = at(i + 1), x2 = at(i + 2);
	const float c1 = 0.5f * (x1 - xm1);
	const float c2 = xm1 - 2.5f * x0 + 2.f * x1 - 0.5f * x2;
	const float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
	return ((c3 * t + c2) * t + c1) * t + x0;
}

void Sampler::stop()
{
	voices[active].target = 0;
	isRunning = false;
}

void Sampler::process(float* out, unsigned int frames)
{
	bool newClip = false;
	bool sameFile = false; // a channel change: keep the playhead
	if(Clip* n = pending.exchange(nullptr, std::memory_order_acquire))
	{
		sameFile = current && current->name == n->name;
		current = n;
		newClip = true;
	}
	const Clip* c = current;

	// region in clip frames, at least 100 ms long
	int64_t a = 0, b = 0;
	float xf = xfadeMs * 0.001f * fs;
	if(c)
	{
		float s = std::min(1.f, std::max(0.f, start.load()));
		float e = std::min(1.f, std::max(0.f, end.load()));
		if(e < s)
			std::swap(s, e);
		a = (int64_t)((double)s * c->frames);
		b = (int64_t)((double)e * c->frames);
		const int64_t minLen = (int64_t)(0.1f * c->sampleRate);
		if(b - a < minLen)
		{
			b = std::min(c->frames, a + minLen);
			a = std::max((int64_t)0, b - minLen);
		}
		// a crossfade lasts at most half the region, so jumps never overlap
		xf = std::min(xf, 0.5f * (float)(b - a) * fs / c->sampleRate);
	}
	const float rampStep = 1.f / std::max(16.f, xf);

	// transport: everything that moves the playhead becomes a pending jump,
	// taken as soon as the idle voice holds audio from there
	const bool wantPlay = play;
	if(newClip)
	{
		if(!sameFile || headPos < a || headPos >= b)
			headPos = a;
		if(isRunning)
			pendingJump = headPos;
	}
	if(!c || !wantPlay)
	{
		if(isRunning)
			stop();
		pendingJump = -1;
	}
	else if(!isRunning && pendingJump < 0)
		pendingJump = headPos >= a && headPos < b ? headPos : a;
	const unsigned int r = retriggers;
	if(r != seenRetriggers)
	{
		seenRetriggers = r;
		if(c && wantPlay)
			pendingJump = a;
		else
			headPos = a;
	}
	const float sk = seekTo.exchange(-1.f);
	if(sk >= 0 && c)
	{
		const double p = std::min(1.f, sk) * (double)(c->frames - 1);
		if(wantPlay)
			pendingJump = p;
		else
			headPos = p;
	}

	// The idle voice, once silent, waits armed where the next jump goes: the
	// pending jump, or else the region start for the next loop.
	auto serviceIdle = [&]() {
		Voice& idle = voices[active ^ 1];
		if(!c || idle.ramp > 0 || idle.target > 0)
			return;
		const int64_t want = pendingJump >= 0 ? (int64_t)pendingJump : a;
		if(!idle.armed || idle.clip != c || idle.start != want)
			arm(idle, c, want);
		else if(pendingJump >= 0 && ready(idle))
		{
			voices[active].target = 0;
			idle.target = 1;
			idle.armed = false;
			active ^= 1;
			isRunning = true;
			pendingJump = -1;
			headPos = idle.start;
		}
	};
	serviceIdle();

	const float targetGain = powf(10.f, gainDb / 20.f);
	const float gainCoef = 1.f - expf(-1.f / (0.02f * fs));
	const float levelCoef = 1.f - expf(-1.f / (0.5f * fs)); // normalisation glides in
	const bool norm = normalize;
	for(unsigned int n = 0; n < frames; ++n)
	{
		gainLin += gainCoef * (targetGain - gainLin);
		float y = 0;
		for(auto& v : voices)
		{
			if(!v.clip || (v.ramp <= 0 && v.target <= 0))
				continue;
			v.level += levelCoef * (levelFor(v.clip, norm) - v.level);
			y += sinf(v.ramp * (float)M_PI_2) * v.level * read(v);
			v.rel += v.clip->sampleRate / fs;
			if(v.ramp < v.target)
				v.ramp = std::min(v.target, v.ramp + rampStep);
			else if(v.ramp > v.target)
				v.ramp = std::max(v.target, v.ramp - rampStep);
		}
		out[n] = y * gainLin;

		// (while a new file fades in, the old one plays on unchecked)
		if(isRunning && voices[active].clip == c)
		{
			const Voice& v = voices[active];
			headPos = v.start + v.rel;
			const Stream& s = *v.stream;
			const bool fileEnded = s.atEnd.load(std::memory_order_relaxed)
				&& v.rel >= s.written.load(std::memory_order_relaxed);
			if(headPos >= b || fileEnded)
			{
				if(!loop)
				{
					stop();
					play = false;
					headPos = a;
				}
				else if(pendingJump < 0)
					pendingJump = a;
			}
			else if(headPos < a - 1 && pendingJump < 0)
				pendingJump = a; // the region moved away from the playhead
		}
		if(pendingJump >= 0)
			serviceIdle();
	}

	float minGain = 0;
	for(unsigned int n = 0; n < frames; ++n)
	{
		out[n] = limiter.process(out[n]);
		minGain = std::min(minGain, limiter.reductionDb());
	}
	limitDb.store(minGain, std::memory_order_relaxed);

	// let the disk thread refill what has been played
	for(auto& v : voices)
		if(v.clip && !v.armed)
			v.stream->consumed.store(std::max((int64_t)0, (int64_t)v.rel - 4), std::memory_order_release);
}

// --- disk thread ------------------------------------------------------------

void Sampler::diskLoop()
{
	struct Handle {
		SNDFILE* file = nullptr;
		const Clip* clip = nullptr;
		unsigned int gen = 0;
		std::vector<float> chunk;
	};
	Handle handles[2];
	while(!quit)
	{
		bool busy = false;
		for(unsigned int i = 0; i < 2; ++i)
		{
			Stream& s = streams[i];
			Handle& h = handles[i];
			const unsigned int gen = s.reqGen.load(std::memory_order_acquire);
			if(gen != h.gen)
			{
				// a new request: open the clip if needed, seek
				h.gen = gen;
				const Clip* c = s.reqClip.load(std::memory_order_relaxed);
				if(c != h.clip)
				{
					if(h.file)
						sf_close(h.file);
					SF_INFO info {};
					h.file = c ? sf_open((dir + "/" + c->name).c_str(), SFM_READ, &info) : nullptr;
					h.clip = c;
					if(c)
						h.chunk.resize(kDiskChunk * c->channels);
				}
				bool ok = h.file && sf_seek(h.file, s.reqStart.load(std::memory_order_relaxed), SEEK_SET) >= 0;
				s.written.store(0, std::memory_order_relaxed);
				s.atEnd.store(!ok, std::memory_order_relaxed);
				s.fillGen.store(gen, std::memory_order_release);
				busy = true;
			}
			if(!h.file || s.atEnd.load(std::memory_order_relaxed))
				continue;
			const int64_t written = s.written.load(std::memory_order_relaxed);
			const int64_t space = (int64_t)Stream::kRing - 8 - (written - s.consumed.load(std::memory_order_acquire));
			if(space < (int64_t)kDiskChunk)
				continue;
			const sf_count_t got = sf_readf_float(h.file, h.chunk.data(), kDiskChunk);
			const unsigned int ch = h.clip->channels;
			for(sf_count_t k = 0; k < std::max((sf_count_t)0, got); ++k)
				s.ring[(written + k) & Stream::kMask] = mono(&h.chunk[k * ch], ch, h.clip->channelMode);
			if(got > 0)
				s.written.store(written + got, std::memory_order_release);
			if(got < (sf_count_t)kDiskChunk)
				s.atEnd.store(true, std::memory_order_relaxed);
			busy = true;
		}
		if(!busy)
			usleep(2000);
	}
	for(auto& h : handles)
		if(h.file)
			sf_close(h.file);
}

// --- loader thread ----------------------------------------------------------

void Sampler::loaderLoop()
{
	using Clock = std::chrono::steady_clock;
	doScan();
	std::string first;
	{
		std::lock_guard<std::mutex> lock(mtx);
		first = loadedName;
	}
	if(!first.empty())
		doLoad(first, true);

	bool savePending = false;
	Clock::time_point lastChange;
	std::unique_lock<std::mutex> lock(mtx);
	while(!quit)
	{
		cv.wait_for(lock, std::chrono::milliseconds(200));
		const bool scan = scanRequested;
		scanRequested = false;
		std::string name;
		name.swap(loadRequest);
		const bool keep = loadKeepsRegion;
		loadPending = false;
		lock.unlock();

		if(scan)
			doScan();
		if(!name.empty())
			doLoad(name, keep);
		if(dirty.exchange(false))
		{
			savePending = true;
			lastChange = Clock::now();
		}
		if(savePending && Clock::now() - lastChange > std::chrono::seconds(2))
		{
			saveConfig();
			savePending = false;
		}
		lock.lock();
	}
}

static bool isAudioFile(const std::string& name)
{
	const size_t dot = name.rfind('.');
	if(dot == std::string::npos)
		return false;
	std::string ext = name.substr(dot + 1);
	for(auto& ch : ext)
		ch = tolower(ch);
	static const std::set<std::string> exts { "wav", "aif", "aiff", "flac", "ogg", "mp3", "caf", "w64" };
	return exts.count(ext);
}

// Audio files in the project folder and its samples/ folder.
void Sampler::doScan()
{
	std::vector<std::string> names;
	for(std::string sub : { "", "samples/" })
	{
		DIR* d = opendir((dir + "/" + sub).c_str());
		if(!d)
			continue;
		while(dirent* e = readdir(d))
		{
			std::string n = e->d_name;
			if(n.empty() || '.' == n[0] || !isAudioFile(n))
				continue;
			names.push_back(sub + n);
		}
		closedir(d);
	}
	std::sort(names.begin(), names.end());

	const unsigned int id = listId + 1;
	std::string text = std::to_string(id);
	unsigned int kept = 0;
	for(auto& n : names)
	{
		if(text.size() + n.size() + 1 > SharedText::kMax)
			break;
		text += "\n" + n;
		++kept;
	}
	names.resize(kept);
	std::lock_guard<std::mutex> lock(mtx);
	fileList = names;
	auto it = std::find(fileList.begin(), fileList.end(), loadedName);
	fileIndex = it == fileList.end() ? -1 : (int)(it - fileList.begin());
	files.write(text);
	listId = id;
}

static std::string formatTime(double seconds)
{
	char s[32];
	const int t = (int)(seconds + 0.5);
	if(t >= 3600)
		snprintf(s, sizeof(s), "%d:%02d:%02d", t / 3600, t / 60 % 60, t % 60);
	else
		snprintf(s, sizeof(s), "%d:%02d", t / 60, t % 60);
	return s;
}

// Analysis results are cached per file and channel mode in .cache/, keyed
// by the file's size and modification time.
struct CacheHeader {
	uint32_t magic = 0x52464f32; // "RFO2"
	uint32_t bins = Sampler::kOverviewBins;
	int64_t size = 0;
	int64_t mtime = 0;
	int64_t frames = 0;
	float peak = 0;
	float medianDb = 0;
	float normDb = 0;
};

static std::string cachePath(const std::string& dir, const Clip* c)
{
	std::string flat = c->name;
	std::replace(flat.begin(), flat.end(), '/', '_');
	return dir + "/.cache/" + flat + "." + std::to_string(c->channelMode) + ".ov";
}

static bool cacheKey(const std::string& dir, const Clip* c, CacheHeader& h)
{
	struct stat st;
	if(stat((dir + "/" + c->name).c_str(), &st))
		return false;
	h.size = st.st_size;
	h.mtime = st.st_mtime;
	h.frames = c->frames;
	return true;
}

void Sampler::doLoad(const std::string& name, bool keepRegion)
{
	loaderState = kLoaderLoading;
	SF_INFO info {};
	SNDFILE* f = sf_open((dir + "/" + name).c_str(), SFM_READ, &info);
	if(f)
		sf_close(f);
	if(!f || info.channels < 1 || info.frames < 1 || info.samplerate < 1)
	{
		const std::string why = f ? "empty or unreadable" : sf_strerror(nullptr);
		message.write(name + ": " + why);
		loaderState = kLoaderError;
		fprintf(stderr, "Sampler: %s: %s\n", name.c_str(), why.c_str());
		return;
	}
	std::unique_ptr<Clip> owned(new Clip);
	Clip* c = owned.get();
	c->name = name;
	c->sampleRate = info.samplerate;
	c->frames = info.frames;
	c->channels = info.channels;
	c->channelMode = info.channels < 2 ? kChannelMix : channel.load();
	c->id = nextClipId++;

	// cached analysis?
	CacheHeader want, got;
	if(cacheKey(dir, c, want))
	{
		std::ifstream in(cachePath(dir, c), std::ios::binary);
		if(in.read((char*)&got, sizeof(got)) && got.magic == want.magic && got.bins == want.bins
			&& got.size == want.size && got.mtime == want.mtime && got.frames == want.frames)
		{
			c->overviewData.resize(1 + 2 * kOverviewBins);
			if(in.read((char*)&c->overviewData[1], 2 * kOverviewBins * sizeof(float)))
			{
				c->overviewData[0] = c->id;
				c->peak = got.peak;
				c->medianDb = got.medianDb;
				c->normDb = got.normDb;
				c->analysed = 1;
				c->overview.store(&c->overviewData, std::memory_order_release);
			}
		}
	}

	if(!keepRegion)
	{
		start = 0;
		end = 1;
	}
	{
		std::lock_guard<std::mutex> lock(mtx);
		loadedName = name;
		auto it = std::find(fileList.begin(), fileList.end(), name);
		fileIndex = it == fileList.end() ? -1 : (int)(it - fileList.begin());
	}
	clips.push_back(std::move(owned));
	pending.store(c, std::memory_order_release);

	static const char* modeNames[] = { "mix", "left", "right" };
	message.write(name + " · " + formatTime(info.frames / (double)info.samplerate) + " · "
		+ std::to_string(info.samplerate / 1000.f).substr(0, 4) + " kHz · "
		+ (1 == info.channels ? std::string("mono") : std::to_string(info.channels) + " ch → " + modeNames[c->channelMode]));
	loaderState = kLoaderIdle;
	touch();
	if(!c->overview.load())
		analyse(c);
}

// Read the whole file once for its loudness, peak and overview. Gives up if
// another file is requested meanwhile.
//
// Loudness is the median of 100 ms RMS levels: in a field recording that is
// the ambience, not the occasional bump or call. Normalise brings it to
// kTargetDb, unless that would push the loudest 0.1% of blocks more than
// kMaxLimitDb into the limiter (a quiet bed with loud events).
void Sampler::analyse(Clip* c)
{
	SF_INFO info {};
	SNDFILE* f = sf_open((dir + "/" + c->name).c_str(), SFM_READ, &info);
	if(!f)
		return;
	constexpr unsigned int kChunk = 65536;
	const unsigned int ch = c->channels;
	std::vector<float> chunk(kChunk * ch);
	std::vector<float> ov(1 + 2 * kOverviewBins, 0.f);
	float peak = 0;
	int64_t done = 0;
	// histogram of 100 ms RMS levels, 0.5 dB steps from -120 dBFS
	constexpr int kHistBins = 240;
	std::vector<int64_t> hist(kHistBins, 0);
	const unsigned int block = std::max(1u, (unsigned int)(0.1f * c->sampleRate));
	double blockSq = 0;
	unsigned int blockFill = 0;
	int64_t blocks = 0;
	while(done < c->frames)
	{
		if(quit || loadPending)
		{
			sf_close(f);
			return;
		}
		const sf_count_t got = sf_readf_float(f, chunk.data(), kChunk);
		if(got <= 0)
			break;
		for(sf_count_t k = 0; k < got && done < c->frames; ++k, ++done)
		{
			const float x = mono(&chunk[k * ch], ch, c->channelMode);
			peak = std::max(peak, fabsf(x));
			blockSq += x * x;
			if(++blockFill == block)
			{
				const float db = 10.f * log10f((float)(blockSq / block) + 1e-13f);
				++hist[std::min(kHistBins - 1, std::max(0, (int)((db + 120.f) * 2.f)))];
				++blocks;
				blockSq = 0;
				blockFill = 0;
			}
			const unsigned int bin = (unsigned int)(done * kOverviewBins / c->frames);
			ov[1 + 2 * bin] = std::min(ov[1 + 2 * bin], x);
			ov[2 + 2 * bin] = std::max(ov[2 + 2 * bin], x);
		}
		c->analysed = (float)done / c->frames;
	}
	sf_close(f);
	auto percentileDb = [&](double p) {
		int64_t want = (int64_t)(p * blocks), seen = 0;
		for(int b = 0; b < kHistBins; ++b)
			if((seen += hist[b]) > want)
				return -120.f + 0.5f * b + 0.25f;
		return 0.f;
	};
	const float medianDb = blocks ? percentileDb(0.5) : -120.f;
	const float loudDb = blocks ? percentileDb(0.999) : -120.f;
	float normDb = std::min(kTargetDb - medianDb, kCeilingDb + kMaxLimitDb - loudDb);
	normDb = medianDb < -100.f ? 0.f : std::min(40.f, std::max(-20.f, normDb)); // silent: leave it
	if(peak < 1e-6f)
		peak = 1; // silent file: don't blow up the noise
	for(unsigned int k = 1; k < ov.size(); ++k)
		ov[k] /= peak;
	ov[0] = c->id;
	c->overviewData = ov;
	c->peak = peak;
	c->medianDb = medianDb;
	c->normDb = normDb;
	c->analysed = 1;
	c->overview.store(&c->overviewData, std::memory_order_release);

	CacheHeader h;
	if(cacheKey(dir, c, h))
	{
		h.peak = peak;
		h.medianDb = medianDb;
		h.normDb = normDb;
		mkdir((dir + "/.cache").c_str(), 0755);
		std::ofstream out(cachePath(dir, c), std::ios::binary);
		out.write((const char*)&h, sizeof(h));
		out.write((const char*)&ov[1], 2 * kOverviewBins * sizeof(float));
	}
}

// --- settings ---------------------------------------------------------------

void Sampler::loadConfig()
{
	std::ifstream in(dir + "/" + kConfigFile);
	std::string line;
	while(std::getline(in, line))
	{
		const size_t eq = line.find('=');
		if(eq == std::string::npos)
			continue;
		const std::string key = line.substr(0, eq);
		const std::string val = line.substr(eq + 1);
		float v = 0;
		try { v = std::stof(val); } catch(...) {}
		if("file" == key)
			loadedName = val;
		else if("play" == key) play = v > 0.5f;
		else if("loop" == key) loop = v > 0.5f;
		else if("normalize" == key) normalize = v > 0.5f;
		else if("gain" == key) gainDb = v;
		else if("xfade" == key) xfadeMs = v;
		else if("start" == key) start = v;
		else if("end" == key) end = v;
		else if("channel" == key) channel = (int)v;
	}
}

void Sampler::saveConfig()
{
	std::string name;
	{
		std::lock_guard<std::mutex> lock(mtx);
		name = loadedName;
	}
	std::ostringstream o;
	o.precision(9);
	o << "file=" << name << "\n"
		<< "play=" << play.load() << "\n"
		<< "loop=" << loop.load() << "\n"
		<< "normalize=" << normalize.load() << "\n"
		<< "gain=" << gainDb.load() << "\n"
		<< "xfade=" << xfadeMs.load() << "\n"
		<< "start=" << start.load() << "\n"
		<< "end=" << end.load() << "\n"
		<< "channel=" << channel.load() << "\n";
	const std::string path = dir + "/" + kConfigFile;
	{
		std::ofstream out(path + ".tmp");
		out << o.str();
		if(!out)
			return;
	}
	rename((path + ".tmp").c_str(), path.c_str());
}
