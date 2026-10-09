#pragma once
// Pepper front panel on a classic Bela cape: 8 knob/CV inputs with two
// layers and pickup, 4 debounced buttons, 10 LEDs with PWM brightness. Call
// process() once per audio block from render().

#include <Bela.h>
#include <algorithm>
#include <array>
#include <cmath>

class Panel {
public:
	static constexpr unsigned int kKnobs = 8;
	static constexpr unsigned int kLayers = 2;
	static constexpr unsigned int kButtons = 4;
	static constexpr unsigned int kLeds = 10;
	// digital channels, left to right as in the Pepper Pd examples
	static constexpr std::array<unsigned int, kButtons> kButtonPins = {{ 15, 14, 13, 12 }};
	static constexpr std::array<unsigned int, kLeds> kLedPins = {{ 6, 7, 10, 2, 3, 0, 1, 4, 5, 8 }};

	void setup(BelaContext* context, const std::array<std::array<float, kKnobs>, kLayers>& defaults)
	{
		for(auto pin : kButtonPins)
			pinMode(context, 0, pin, INPUT);
		for(auto pin : kLedPins)
			pinMode(context, 0, pin, OUTPUT);
		values = defaults;
		for(auto& layerPicked : picked)
			layerPicked.fill(false);
		picked[0].fill(true); // layer A follows the knobs from boot
		smoothCoef = 1.f - expf(-(float)context->audioFrames / (0.005f * context->audioSampleRate));
		debounceBlocks = (unsigned int)(0.003f * context->audioSampleRate / context->audioFrames) + 1;
		calibrationBlocks = (unsigned int)(0.05f * context->audioSampleRate / context->audioFrames);
		pwmStep = kPwmHz / context->digitalSampleRate;
	}

	void process(BelaContext* context)
	{
		// knobs
		for(unsigned int c = 0; c < kKnobs && c < context->analogInChannels; ++c)
		{
			float raw = analogRead(context, 0, c);
			if(firstBlock)
				smoothed[c] = raw;
			float prev = smoothed[c];
			smoothed[c] += smoothCoef * (raw - smoothed[c]);
			if(fabsf(smoothed[c] - lastMoved[c]) > 0.01f)
			{
				lastMoved[c] = smoothed[c];
				touchedKnob = c;
				touchedBlocks = 0;
			}
			float& v = values[layer][c];
			if(!picked[layer][c])
			{
				// pick up when the knob reaches or crosses the stored value
				bool crossed = (prev - v) * (smoothed[c] - v) <= 0;
				if(fabsf(smoothed[c] - v) < 0.02f || (!firstBlock && crossed))
					picked[layer][c] = true;
			}
			if(picked[layer][c])
				v = smoothed[c];
		}
		firstBlock = false;
		++touchedBlocks;

		// buttons: the state seen during the first 50 ms counts as released,
		// so the polarity of the hardware doesn't matter
		for(unsigned int b = 0; b < kButtons; ++b)
		{
			int raw = digitalRead(context, 0, kButtonPins[b]);
			pressedEvent[b] = releasedEvent[b] = false;
			if(calibrationCount < calibrationBlocks)
			{
				idleLevel[b] = raw;
				continue;
			}
			bool down = raw != idleLevel[b];
			if(down != candidate[b])
			{
				candidate[b] = down;
				stableCount[b] = 0;
			}
			else if(stableCount[b] < debounceBlocks && ++stableCount[b] == debounceBlocks && down != held[b])
			{
				held[b] = down;
				(down ? pressedEvent : releasedEvent)[b] = true;
			}
		}
		if(calibrationCount < calibrationBlocks)
			++calibrationCount;

		// LEDs: per-frame PWM, on while the ramp is below the duty cycle
		for(unsigned int n = 0; n < context->digitalFrames; ++n)
		{
			for(unsigned int l = 0; l < kLeds; ++l)
				digitalWriteOnce(context, n, kLedPins[l], pwmPhase < duty[l]);
			pwmPhase += pwmStep;
			if(pwmPhase >= 1.f)
				pwmPhase -= 1.f;
		}
	}

	// Switching layer makes every knob of the new layer wait for pickup.
	void setLayer(unsigned int l)
	{
		if(l == layer || l >= kLayers)
			return;
		layer = l;
		picked[layer].fill(false);
	}
	unsigned int getLayer() const { return layer; }
	float get(unsigned int l, unsigned int knob) const { return values[l][knob]; }
	// false while a knob of the current layer waits to reach its stored value
	bool isPicked(unsigned int l, unsigned int knob) const { return picked[l][knob]; }
	bool isHeld(unsigned int b) const { return held[b]; }
	bool pressed(unsigned int b) const { return pressedEvent[b]; }
	bool released(unsigned int b) const { return releasedEvent[b]; }
	// knob moved recently (index), or -1
	int recentlyTouched(unsigned int maxBlocks) const { return touchedKnob >= 0 && touchedBlocks < maxBlocks ? touchedKnob : -1; }
	float knobValue(unsigned int knob) const { return smoothed[knob]; }
	void setLed(unsigned int l, bool on) { setBrightness(l, on ? 1.f : 0.f); }
	// 0 (off) to 1 (fully on), as PWM duty cycle
	void setBrightness(unsigned int l, float b) { if(l < kLeds) duty[l] = std::min(1.f, std::max(0.f, b)); }
	// light the first `count` LEDs of `n` starting at `first`
	void setBar(float value, unsigned int first, unsigned int n)
	{
		int lit = (int)(value * n + 0.5f);
		for(unsigned int l = 0; l < n; ++l)
			setLed(first + l, (int)l < lit);
	}

private:
	std::array<std::array<float, kKnobs>, kLayers> values {};
	std::array<std::array<bool, kKnobs>, kLayers> picked {};
	std::array<float, kKnobs> smoothed {};
	std::array<float, kKnobs> lastMoved {};
	unsigned int layer = 0;
	int touchedKnob = -1;
	unsigned int touchedBlocks = 0;
	float smoothCoef = 1;
	bool firstBlock = true;

	std::array<int, kButtons> idleLevel {};
	std::array<bool, kButtons> candidate {};
	std::array<bool, kButtons> held {};
	std::array<bool, kButtons> pressedEvent {};
	std::array<bool, kButtons> releasedEvent {};
	std::array<unsigned int, kButtons> stableCount {};
	unsigned int debounceBlocks = 8;
	unsigned int calibrationBlocks = 128;
	unsigned int calibrationCount = 0;

	static constexpr float kPwmHz = 1000.f;
	std::array<float, kLeds> duty {};
	float pwmPhase = 0;
	float pwmStep = 0;
};
