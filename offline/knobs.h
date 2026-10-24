#pragma once
// Knob names for the command line, and each knob's value in its own units,
// as the GUI shows them (sketch.js KNOBS).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include "engine.h"

struct KnobName {
	const char* name;
	int layer; // 0 = A, 1 = B
	int index;
};

static const KnobName kKnobNames[] = {
	{ "drone-gain", 0, Knobs::kDroneGain }, { "field-gain", 0, Knobs::kFieldGain },
	{ "whiten", 0, Knobs::kWhiten }, { "contrast", 0, Knobs::kContrast },
	{ "floor-only", 0, Knobs::kFloorOnly }, { "smear", 0, Knobs::kSmear },
	{ "tonality", 0, Knobs::kTonality }, { "wet", 0, Knobs::kWet },
	{ "out-gain", 1, Knobs::kOutGain }, { "attack", 1, Knobs::kAttack },
	{ "floor-speed", 1, Knobs::kFloorSpeed }, { "event-thresh", 1, Knobs::kEventThresh },
	{ "dry-source", 1, Knobs::kDrySource }, { "cv-smooth", 1, Knobs::kCvSmooth },
	{ "event-low", 1, Knobs::kEventLow }, { "event-high", 1, Knobs::kEventHigh },
};

// "whiten", "A2" or "b1" (numbered 0-7 as on the faceplate); false if unknown
static inline bool findKnob(const std::string& s, int& layer, int& index)
{
	for(const auto& k : kKnobNames)
	{
		if(s == k.name)
		{
			layer = k.layer;
			index = k.index;
			return true;
		}
	}
	if(2 == s.size() && strchr("AaBb", s[0]) && s[1] >= '0' && s[1] <= '7')
	{
		layer = 'A' == toupper(s[0]) ? 0 : 1;
		index = s[1] - '0';
		return true;
	}
	return false;
}

static inline const char* knobName(int layer, int index)
{
	for(const auto& k : kKnobNames)
		if(k.layer == layer && k.index == index)
			return k.name;
	return "?";
}

static inline std::string fmtSecs(float s)
{
	char b[32];
	if(s < 1)
		snprintf(b, sizeof(b), "%.0f ms", 1000 * s);
	else
		snprintf(b, sizeof(b), "%.2f s", s);
	return b;
}

static inline std::string fmtHz(float hz)
{
	char b[32];
	if(hz < 1000)
		snprintf(b, sizeof(b), "%.0f Hz", hz);
	else
		snprintf(b, sizeof(b), "%.1f kHz", hz / 1000);
	return b;
}

// The knob's value in its units, as in render.cpp and the GUI
static inline std::string knobUnits(int layer, int index, float v)
{
	char b[48];
	auto gainDb = [&](float x) {
		const float g = 4 * x * x;
		if(g < 1e-3f)
			snprintf(b, sizeof(b), "off");
		else
			snprintf(b, sizeof(b), "%+.1f dB", 20 * log10f(g));
		return std::string(b);
	};
	auto pct = [&](float x) { snprintf(b, sizeof(b), "%.0f%%", 100 * x); return std::string(b); };
	if(0 == layer)
	{
		switch(index)
		{
		case Knobs::kDroneGain:
		case Knobs::kFieldGain: return gainDb(v);
		case Knobs::kContrast: snprintf(b, sizeof(b), "^%.2f", 0.5f * powf(8, v)); return b;
		case Knobs::kSmear: return fmtSecs(5 * v * v);
		default: return pct(v);
		}
	}
	switch(index)
	{
	case Knobs::kOutGain:
		if(v < 0.02f)
			return "off";
		snprintf(b, sizeof(b), "%+.1f dB", -24 + 48 * v);
		return b;
	case Knobs::kAttack: return fmtSecs(2 * v * v);
	case Knobs::kFloorSpeed: snprintf(b, sizeof(b), "%.1f dB/s", 0.5f + 30 * v * v); return b;
	case Knobs::kEventThresh: snprintf(b, sizeof(b), "%.1f dB", 3 + 27 * v); return b;
	case Knobs::kDrySource: return v < 0.02f ? "field" : v > 0.98f ? "drone" : "drone " + pct(v);
	case Knobs::kCvSmooth: return fmtSecs(0.001f + 0.5f * v * v);
	default: return fmtHz(100 * powf(128, v));
	}
}

static inline float& knobRef(Knobs& k, int layer, int index)
{
	return layer ? k.b[index] : k.a[index];
}
