// refft-render: run the REFFT stamp on files or test signals, on the desktop.
// See offline/README.md for the options; `refft-render --help` lists them.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "engine.h"
#include "knobs.h"
#include "signals.h"
#include "wav.h"

static constexpr float kFs = 44100;

static void usage()
{
	fprintf(stderr,
		"usage: refft-render -d DRONE -f FIELD -o OUT.wav [options]\n"
		"\n"
		"DRONE, FIELD: a .wav file (any rate; resampled to 44.1 kHz like the sampler),\n"
		"  or a test signal: sine:HZ[@dB], saw:HZ[@dB] (peak dBFS, default -6),\n"
		"  steps:HZ,HZ,...[@dB] (1 s each), white:dB, pink:dB (RMS dBFS),\n"
		"  birds, whistle, gaps, silence. Shorter sources loop.\n"
		"\n"
		"  --set KNOB=V         knob position 0-1, as on the panel (repeatable)\n"
		"  --sweep KNOB=V,V,... one step per value, --step seconds each\n"
		"  --step SEC           length of a sweep step (default 4)\n"
		"  --ramp KNOB=A:B      move a knob linearly over the whole render\n"
		"  --seconds SEC        length (default: the sweep, else the field file, else 20)\n"
		"  --size N             FFT size 1024, 2048 (default), 4096 or 8192\n"
		"  --freeze-at SEC      press freeze at this time\n"
		"  --tap out|wet|dry    what to write: out R at the jack (default), the stamp alone,\n"
		"                       or the dry blend\n"
		"  --adc-noise DB       add white noise of this RMS dBFS to both inputs (the codec's hiss)\n"
		"  --drone-ch N, --field-ch N  channel of a multichannel file (default: their mean)\n"
		"  --no-ticks           no click at the start of each sweep step\n"
		"\n"
		"KNOB: drone-gain field-gain whiten contrast floor-only smear tonality wet\n"
		"      out-gain attack floor-speed event-thresh dry-source cv-smooth event-low event-high\n"
		"      or A0-A7, B0-B7 (numbered as on the faceplate). In sweeps also: size (1024...8192), freeze (0/1), bypass (0/1).\n"
		"Unset knobs sit at the board's boot defaults.\n");
}

static bool parseFloat(const std::string& s, float& v)
{
	char* end;
	v = strtof(s.c_str(), &end);
	return !s.empty() && '\0' == *end;
}

// A source spec or a file, at kFs; `seconds` is the length for synthetic ones
static bool loadSource(const std::string& spec, int ch, double seconds, Signal& out, std::string& err)
{
	const size_t n = (size_t)(seconds * kFs);
	const size_t colon = spec.find(':');
	const std::string kind = spec.substr(0, colon);
	std::string arg = colon == std::string::npos ? "" : spec.substr(colon + 1);
	float db = -6;
	const size_t at = arg.find('@');
	if(at != std::string::npos)
	{
		if(!parseFloat(arg.substr(at + 1), db))
		{
			err = "bad level in " + spec;
			return false;
		}
		arg = arg.substr(0, at);
	}
	float v = 0;
	if("sine" == kind && parseFloat(arg, v))
		out = sine(n, kFs, v, db);
	else if("saw" == kind && parseFloat(arg, v))
		out = saw(n, kFs, v, db);
	else if("steps" == kind)
	{
		std::vector<float> hz;
		size_t p = 0;
		while(p <= arg.size())
		{
			size_t q = arg.find(',', p);
			if(q == std::string::npos)
				q = arg.size();
			if(!parseFloat(arg.substr(p, q - p), v))
			{
				err = "bad pitch list in " + spec;
				return false;
			}
			hz.push_back(v);
			p = q + 1;
		}
		out = steps(n, kFs, hz, 1.f, db);
	}
	else if("white" == kind && parseFloat(arg, v))
		out = white(n, v);
	else if("pink" == kind && parseFloat(arg, v))
		out = pink(n, v);
	else if("birds" == spec)
		out = birds(n, kFs, -40, -22, -28);
	else if("whistle" == spec)
		out = whistle(n, kFs, -40, 4000, -25, 3, 8);
	else if("gaps" == spec)
		out = gaps(n, kFs, -30, 1, 1);
	else if("silence" == spec)
		out.assign(n, 0.f);
	else
	{
		Wav w;
		if(!readWav(spec, w, err))
			return false;
		out = resample(w.channel(ch), w.rate, kFs);
	}
	return true;
}

struct Sweep {
	std::string name;
	int layer = -1, index = -1; // a knob, or a pseudo-knob by name
	std::vector<float> values;
};

static std::string sweepLabel(const Sweep& s, float v)
{
	char b[64];
	if("size" == s.name)
		snprintf(b, sizeof(b), "size %.0f", v);
	else if("freeze" == s.name || "bypass" == s.name)
		snprintf(b, sizeof(b), "%s %s", s.name.c_str(), v > 0.5f ? "on" : "off");
	else
		snprintf(b, sizeof(b), "%s %.2f (%s)", s.name.c_str(), v, knobUnits(s.layer, s.index, v).c_str());
	return b;
}

static int sizeIndexOf(float n)
{
	for(unsigned int i = 0; i < Stamp::kSizes.size(); ++i)
		if((unsigned int)lrintf(n) == Stamp::kSizes[i])
			return (int)i;
	return -1;
}

int main(int argc, char** argv)
{
	std::string droneSpec, fieldSpec, outPath, tap = "out";
	Knobs base;
	Sweep sweep;
	bool haveRamp = false;
	int rampLayer = 0, rampIndex = 0;
	float rampA = 0, rampB = 0;
	float stepSec = 4, seconds = 0, freezeAt = -1, adcNoiseDb = 0;
	bool adcNoise = false, ticks = true;
	int droneCh = -1, fieldCh = -1;

	for(int i = 1; i < argc; ++i)
	{
		const std::string a = argv[i];
		auto next = [&]() -> std::string {
			if(i + 1 >= argc)
			{
				fprintf(stderr, "%s needs a value\n", a.c_str());
				exit(2);
			}
			return argv[++i];
		};
		auto num = [&]() {
			float v;
			const std::string s = next();
			if(!parseFloat(s, v))
			{
				fprintf(stderr, "%s: not a number: %s\n", a.c_str(), s.c_str());
				exit(2);
			}
			return v;
		};
		auto split = [&](const std::string& s, std::string& key, std::string& val) {
			const size_t eq = s.find('=');
			if(eq == std::string::npos)
			{
				fprintf(stderr, "%s: expected KNOB=VALUE, got %s\n", a.c_str(), s.c_str());
				exit(2);
			}
			key = s.substr(0, eq);
			val = s.substr(eq + 1);
		};
		if("-h" == a || "--help" == a)
		{
			usage();
			return 0;
		}
		else if("-d" == a || "--drone" == a)
			droneSpec = next();
		else if("-f" == a || "--field" == a)
			fieldSpec = next();
		else if("-o" == a || "--out" == a)
			outPath = next();
		else if("--set" == a)
		{
			std::string key, val;
			split(next(), key, val);
			int l, k;
			float v;
			if(!findKnob(key, l, k) || !parseFloat(val, v))
			{
				fprintf(stderr, "--set: unknown knob or bad value: %s=%s\n", key.c_str(), val.c_str());
				return 2;
			}
			knobRef(base, l, k) = std::min(1.f, std::max(0.f, v));
		}
		else if("--sweep" == a)
		{
			std::string val;
			split(next(), sweep.name, val);
			if(!findKnob(sweep.name, sweep.layer, sweep.index) && "size" != sweep.name && "freeze" != sweep.name && "bypass" != sweep.name)
			{
				fprintf(stderr, "--sweep: unknown knob %s\n", sweep.name.c_str());
				return 2;
			}
			if(sweep.layer >= 0)
				sweep.name = knobName(sweep.layer, sweep.index);
			size_t p = 0;
			while(p <= val.size())
			{
				size_t q = val.find(',', p);
				if(q == std::string::npos)
					q = val.size();
				float v;
				if(!parseFloat(val.substr(p, q - p), v) || ("size" == sweep.name && sizeIndexOf(v) < 0))
				{
					fprintf(stderr, "--sweep: bad value list %s\n", val.c_str());
					return 2;
				}
				sweep.values.push_back(v);
				p = q + 1;
			}
		}
		else if("--ramp" == a)
		{
			std::string key, val;
			split(next(), key, val);
			const size_t c = val.find(':');
			if(!findKnob(key, rampLayer, rampIndex) || c == std::string::npos
				|| !parseFloat(val.substr(0, c), rampA) || !parseFloat(val.substr(c + 1), rampB))
			{
				fprintf(stderr, "--ramp: expected KNOB=FROM:TO, got %s=%s\n", key.c_str(), val.c_str());
				return 2;
			}
			haveRamp = true;
		}
		else if("--step" == a)
			stepSec = num();
		else if("--seconds" == a)
			seconds = num();
		else if("--size" == a)
		{
			const int s = sizeIndexOf(num());
			if(s < 0)
			{
				fprintf(stderr, "--size: one of 1024 2048 4096 8192\n");
				return 2;
			}
			base.sizeIndex = s;
		}
		else if("--freeze-at" == a)
			freezeAt = num();
		else if("--tap" == a)
		{
			tap = next();
			if("out" != tap && "wet" != tap && "dry" != tap)
			{
				fprintf(stderr, "--tap: out, wet or dry\n");
				return 2;
			}
		}
		else if("--adc-noise" == a)
		{
			adcNoiseDb = num();
			adcNoise = true;
		}
		else if("--drone-ch" == a)
			droneCh = (int)num();
		else if("--field-ch" == a)
			fieldCh = (int)num();
		else if("--no-ticks" == a)
			ticks = false;
		else
		{
			fprintf(stderr, "unknown option %s\n\n", a.c_str());
			usage();
			return 2;
		}
	}
	if(droneSpec.empty() || fieldSpec.empty() || outPath.empty())
	{
		usage();
		return 2;
	}

	// length: explicit, else the sweep, else a field file's own, else 20 s
	double len = seconds;
	if(len <= 0 && !sweep.values.empty())
		len = stepSec * sweep.values.size();
	Signal drone, field;
	std::string err;
	if(len <= 0)
	{
		if(!loadSource(fieldSpec, fieldCh, 20, field, err))
		{
			fprintf(stderr, "%s\n", err.c_str());
			return 1;
		}
		len = field.size() / kFs;
		field.clear();
	}
	if(!loadSource(droneSpec, droneCh, len, drone, err) || !loadSource(fieldSpec, fieldCh, len, field, err))
	{
		fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}
	if(drone.empty() || field.empty())
	{
		fprintf(stderr, "empty source\n");
		return 1;
	}
	const size_t total = (size_t)(len * kFs);
	Signal noiseL, noiseR;
	if(adcNoise)
	{
		noiseL = white(total, adcNoiseDb, 101);
		noiseR = white(total, adcNoiseDb, 102);
	}

	Engine engine;
	if(!engine.setup(kFs))
	{
		fprintf(stderr, "stamp setup failed\n");
		return 1;
	}

	// What each moment's knobs are
	const size_t stepLen = (size_t)(stepSec * kFs);
	auto knobsAt = [&](size_t n) {
		Knobs k = base;
		if(!sweep.values.empty())
		{
			const float v = sweep.values[std::min(sweep.values.size() - 1, n / stepLen)];
			if("size" == sweep.name)
				k.sizeIndex = sizeIndexOf(v);
			else if("freeze" == sweep.name)
				k.freeze = v > 0.5f;
			else if("bypass" == sweep.name)
				k.bypass = v > 0.5f;
			else
				knobRef(k, sweep.layer, sweep.index) = std::min(1.f, std::max(0.f, v));
		}
		if(haveRamp)
			knobRef(k, rampLayer, rampIndex) = std::min(1.f, std::max(0.f, rampA + (rampB - rampA) * n / (float)total));
		if(freezeAt >= 0 && n >= freezeAt * kFs)
			k.freeze = true;
		return k;
	};

	// The report: one row per sweep step (measured over its last 3/4, after
	// the change has settled), else every 2 s
	const size_t rowLen = sweep.values.empty() ? (size_t)(2 * kFs) : stepLen;
	const size_t settle = sweep.values.empty() ? 0 : rowLen / 4;
	printf("REFFT offline: %s  ×  %s, %.1f s, FFT %u%s\n", droneSpec.c_str(), fieldSpec.c_str(), len,
		Stamp::kSizes[base.sizeIndex], adcNoise ? (", ADC noise " + std::to_string((int)adcNoiseDb) + " dBFS").c_str() : "");
	for(int l = 0; l < 2; ++l)
	{
		std::string line;
		for(int i = 0; i < 8; ++i)
		{
			const bool swept = sweep.layer == l && sweep.index == i;
			const bool ramped = haveRamp && rampLayer == l && rampIndex == i;
			if(!swept && !ramped)
				line += (line.empty() ? "" : ", ") + std::string(knobName(l, i)) + " " + knobUnits(l, i, l ? base.b[i] : base.a[i]);
		}
		printf("layer %c: %s\n", l ? 'B' : 'A', line.c_str());
	}
	if(haveRamp)
		printf("ramp: %s %.2f -> %.2f\n", knobName(rampLayer, rampIndex), rampA, rampB);
	printf("\n%-30s %7s %7s %7s %7s %6s %5s %6s %6s\n", sweep.values.empty() ? "time" : "step",
		"drone", "field", "wet", "out R", "limit", "flat", "bright", "events");
	printf("%-30s %7s %7s %7s %7s %6s %5s %6s %6s\n", "", "dBFS", "dBFS", "dBFS", "dBFS", "dB", "", "Hz", "");

	Wav outWav;
	outWav.rate = kFs;
	outWav.channels = 1;
	outWav.data.resize(total);
	double sq[4] = {}; // raw drone, raw field, wet, out R over the row
	double flat = 0, bright = 0, events = 0;
	unsigned int featN = 0;
	float limit = 0;
	std::vector<std::pair<double, std::string>> labels;

	for(size_t n = 0; n < total; n += Engine::kBlock)
	{
		const Knobs k = knobsAt(n);
		engine.setKnobs(k);
		for(size_t j = n; j < std::min(total, n + Engine::kBlock); ++j)
		{
			float d = drone[j % drone.size()], f = field[j % field.size()];
			if(adcNoise)
			{
				d += noiseL[j];
				f += noiseR[j];
			}
			const Taps t = engine.process(d, f);
			outWav.data[j] = "wet" == tap ? t.wet : "dry" == tap ? t.dry : t.outR;
			if(j % rowLen >= settle)
			{
				sq[0] += d * d;
				sq[1] += f * f;
				sq[2] += t.wet * t.wet;
				sq[3] += t.outR * t.outR;
			}
			if(j % rowLen == rowLen - 1 || j == total - 1)
			{
				const size_t m = j % rowLen + 1 - settle;
				char label[64];
				if(sweep.values.empty())
					snprintf(label, sizeof(label), "%.0f–%.0f s", (j + 1 - (j % rowLen + 1)) / kFs, (j + 1) / kFs);
				const std::string row = sweep.values.empty() ? label : sweepLabel(sweep, sweep.values[std::min(sweep.values.size() - 1, j / stepLen)]);
				auto db = [&](double s) { return 10 * log10(s / std::max<size_t>(1, m) + 1e-24); };
				printf("%-30s %7.1f %7.1f %7.1f %7.1f %6.1f %5.2f %6.0f %6.2f\n", row.c_str(),
					db(sq[0]), db(sq[1]), db(sq[2]), db(sq[3]), limit, featN ? flat / featN : 0., featN ? bright / featN : 0., featN ? events / featN : 0.);
				std::fill(std::begin(sq), std::end(sq), 0.);
				flat = bright = events = 0;
				featN = 0;
				limit = 0;
			}
		}
		if(n % rowLen >= settle)
		{
			const StampFeatures& f = engine.features();
			flat += f.fieldFlatness;
			bright += f.fieldCentroid;
			events += f.eventAmount;
			++featN;
			limit = std::min(limit, engine.takeLimitDb());
		}
		else
			engine.takeLimitDb();
	}

	// a click at each step, so the steps can be told apart by ear
	if(!sweep.values.empty())
	{
		for(size_t s = 0; s < sweep.values.size(); ++s)
		{
			const size_t at = s * stepLen;
			labels.push_back({ at / (double)kFs, sweepLabel(sweep, sweep.values[s]) });
			if(!ticks)
				continue;
			const size_t clickLen = (size_t)(0.003 * kFs);
			for(size_t i = 0; i < clickLen && at + i < total; ++i)
				outWav.data[at + i] += dbToAmp(-24) * sinf((float)(M_PI * i / clickLen)) * sinf((float)(2 * M_PI * 1500 * i / kFs));
		}
	}
	if(!writeWav(outPath, outWav))
	{
		fprintf(stderr, "can't write %s\n", outPath.c_str());
		return 1;
	}
	// Audacity label track next to the file
	if(!labels.empty())
	{
		std::string lp = outPath;
		const size_t dot = lp.rfind('.');
		lp = (dot == std::string::npos ? lp : lp.substr(0, dot)) + ".txt";
		if(FILE* f = fopen(lp.c_str(), "w"))
		{
			for(size_t i = 0; i < labels.size(); ++i)
				fprintf(f, "%.3f\t%.3f\t%s\n", labels[i].first, i + 1 < labels.size() ? labels[i + 1].first : len, labels[i].second.c_str());
			fclose(f);
		}
	}
	printf("\nwrote %s (%s, %u frames done)\n", outPath.c_str(), tap.c_str(), engine.framesDone());
	return 0;
}
