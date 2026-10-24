#pragma once

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
