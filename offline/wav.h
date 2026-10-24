#pragma once
// Minimal WAV reading (PCM 16/24/32-bit, float 32/64, mono or multichannel,
// WAVE_FORMAT_EXTENSIBLE included) and writing (float 32 or PCM 24).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

struct Wav {
	float rate = 44100;
	unsigned int channels = 1;
	std::vector<float> data; // interleaved

	size_t frames() const { return channels ? data.size() / channels : 0; }
	// one channel, or the mean of all of them (ch < 0)
	std::vector<float> channel(int ch) const
	{
		std::vector<float> out(frames());
		for(size_t n = 0; n < out.size(); ++n)
		{
			if(ch >= 0)
				out[n] = data[n * channels + std::min<unsigned int>(ch, channels - 1)];
			else
			{
				float s = 0;
				for(unsigned int c = 0; c < channels; ++c)
					s += data[n * channels + c];
				out[n] = s / channels;
			}
		}
		return out;
	}
};

static inline uint32_t le32(const unsigned char* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static inline uint16_t le16(const unsigned char* p) { return p[0] | p[1] << 8; }

static inline bool readWav(const std::string& path, Wav& w, std::string& err)
{
	FILE* f = fopen(path.c_str(), "rb");
	if(!f)
	{
		err = "can't open " + path;
		return false;
	}
	std::vector<unsigned char> b;
	unsigned char chunk[65536];
	size_t got;
	while((got = fread(chunk, 1, sizeof(chunk), f)) > 0)
		b.insert(b.end(), chunk, chunk + got);
	fclose(f);
	if(b.size() < 12 || memcmp(&b[0], "RIFF", 4) || memcmp(&b[8], "WAVE", 4))
	{
		err = path + ": not a RIFF/WAVE file (convert it, e.g. sox in.flac out.wav)";
		return false;
	}
	unsigned int format = 0, bits = 0;
	size_t pos = 12;
	const unsigned char* pcm = nullptr;
	size_t pcmBytes = 0;
	while(pos + 8 <= b.size())
	{
		const uint32_t size = le32(&b[pos + 4]);
		const unsigned char* body = &b[pos + 8];
		const size_t avail = std::min<size_t>(size, b.size() - pos - 8);
		if(!memcmp(&b[pos], "fmt ", 4) && avail >= 16)
		{
			format = le16(body);
			w.channels = le16(body + 2);
			w.rate = (float)le32(body + 4);
			bits = le16(body + 14);
			if(0xFFFE == format && avail >= 26)
				format = le16(body + 24); // extensible: the subformat's first two bytes
		}
		else if(!memcmp(&b[pos], "data", 4))
		{
			pcm = body;
			pcmBytes = avail;
		}
		pos += 8 + size + (size & 1);
	}
	if(!pcm || !w.channels || !(1 == format || 3 == format))
	{
		err = path + ": unsupported WAV (need PCM or float)";
		return false;
	}
	const unsigned int bytes = bits / 8;
	const size_t count = pcmBytes / bytes;
	w.data.resize(count);
	for(size_t i = 0; i < count; ++i)
	{
		const unsigned char* p = pcm + i * bytes;
		float v = 0;
		if(3 == format && 32 == bits)
			memcpy(&v, p, 4);
		else if(3 == format && 64 == bits)
		{
			double d;
			memcpy(&d, p, 8);
			v = (float)d;
		}
		else if(16 == bits)
			v = (int16_t)le16(p) / 32768.f;
		else if(24 == bits)
			v = (int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24) / 2147483648.f;
		else if(32 == bits)
			v = (int32_t)le32(p) / 2147483648.f;
		else
		{
			err = path + ": unsupported bit depth " + std::to_string(bits);
			return false;
		}
		w.data[i] = v;
	}
	w.data.resize(w.frames() * w.channels);
	return true;
}

// float32 by default; pcm24 for files the board's sampler reads
static inline bool writeWav(const std::string& path, const Wav& w, bool pcm24 = false)
{
	FILE* f = fopen(path.c_str(), "wb");
	if(!f)
		return false;
	const unsigned int bytes = pcm24 ? 3 : 4;
	const uint32_t dataBytes = (uint32_t)(w.data.size() * bytes);
	auto put32 = [&](uint32_t v) { unsigned char p[4] = { (unsigned char)v, (unsigned char)(v >> 8), (unsigned char)(v >> 16), (unsigned char)(v >> 24) }; fwrite(p, 1, 4, f); };
	auto put16 = [&](uint16_t v) { unsigned char p[2] = { (unsigned char)v, (unsigned char)(v >> 8) }; fwrite(p, 1, 2, f); };
	fwrite("RIFF", 1, 4, f);
	put32(36 + dataBytes);
	fwrite("WAVEfmt ", 1, 8, f);
	put32(16);
	put16(pcm24 ? 1 : 3);
	put16((uint16_t)w.channels);
	put32((uint32_t)w.rate);
	put32((uint32_t)w.rate * w.channels * bytes);
	put16((uint16_t)(w.channels * bytes));
	put16((uint16_t)(bytes * 8));
	fwrite("data", 1, 4, f);
	put32(dataBytes);
	for(float v : w.data)
	{
		if(pcm24)
		{
			const int32_t s = (int32_t)lrintf(std::min(1.f, std::max(-1.f, v)) * 8388607.f);
			unsigned char p[3] = { (unsigned char)s, (unsigned char)(s >> 8), (unsigned char)(s >> 16) };
			fwrite(p, 1, 3, f);
		}
		else
			fwrite(&v, 4, 1, f);
	}
	return 0 == fclose(f);
}

// To `rate` with the sampler's cubic Hermite interpolation (Sampler::read),
// so a file reaches the stamp as it would from the board's player.
static inline std::vector<float> resample(const std::vector<float>& x, float from, float to)
{
	if(from == to || x.empty())
		return x;
	const double step = from / to;
	std::vector<float> y((size_t)(x.size() / step));
	auto at = [&](int64_t k) { return k >= 0 && k < (int64_t)x.size() ? x[k] : 0.f; };
	for(size_t n = 0; n < y.size(); ++n)
	{
		const double pos = n * step;
		const int64_t i = (int64_t)pos;
		const float t = (float)(pos - i);
		const float xm1 = at(i - 1), x0 = at(i), x1 = at(i + 1), x2 = at(i + 2);
		const float c1 = 0.5f * (x1 - xm1);
		const float c2 = xm1 - 2.5f * x0 + 2.f * x1 - 0.5f * x2;
		const float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
		y[n] = ((c3 * t + c2) * t + c1) * t + x0;
	}
	return y;
}
