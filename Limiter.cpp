#include "Limiter.h"
#include <algorithm>
#include <cmath>
#include <iterator>

constexpr unsigned int Limiter::kLook;

void Limiter::setup(float fs, float ceilingDb, float releaseSec)
{
	ceiling = powf(10.f, ceilingDb / 20.f);
	relCoef = 1.f - expf(-1.f / (releaseSec * fs));
	std::fill(std::begin(need), std::end(need), 1.f);
	std::fill(std::begin(env), std::end(env), 1.f);
	envSum = kLook;
	held = gain = 1.f;
}

float Limiter::process(float x)
{
	const float a = fabsf(x);
	need[pos] = a > ceiling ? ceiling / a : 1.f;
	float m = 1.f;
	for(float v : need)
		m = std::min(m, v);
	held = m < held ? m : held + relCoef * (m - held);
	// average of `held` over the kLook samples before this one: each of them
	// saw the outgoing sample in its window, so none is above what it needs
	gain = std::min(1.f, (float)(envSum / kLook));
	envSum += held - env[pos];
	env[pos] = held;
	const float y = delay[pos];
	delay[pos] = x;
	pos = (pos + 1) % kLook;
	// the box average can't overshoot by more than rounding: clamp anyway
	return std::min(ceiling, std::max(-ceiling, y * gain));
}

float Limiter::reductionDb() const
{
	return 20.f * log10f(gain);
}
