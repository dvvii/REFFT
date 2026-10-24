#pragma once
// Stand-in for Bela's libraries/Fft/Fft.h (an NE10 wrapper) with the same
// interface, so Stamp.cpp builds unchanged on the desktop. A plain radix-2
// complex FFT: slower than NE10, exact enough to compare by ear and by
// number. The inverse is unscaled, like NE10's; Stamp measures the
// round-trip gain at setup and doesn't depend on it.

#include <cmath>
#include <complex>
#include <vector>

class Fft
{
public:
	struct Cpx { float r, i; };

	int setup(unsigned int n)
	{
		if(!isPowerOfTwo(n))
			return 1;
		length = n;
		timeDomain.assign(n, 0.f);
		frequencyDomain.assign(n / 2 + 1, Cpx{ 0.f, 0.f });
		work.assign(n, 0.);
		twiddle.resize(n / 2);
		for(unsigned int k = 0; k < n / 2; ++k)
			twiddle[k] = std::polar(1., -2. * M_PI * k / n);
		return 0;
	}
	void fft()
	{
		for(unsigned int n = 0; n < length; ++n)
			work[n] = timeDomain[n];
		transform(false);
		for(unsigned int k = 0; k <= length / 2; ++k)
			frequencyDomain[k] = Cpx{ (float)work[k].real(), (float)work[k].imag() };
	}
	void ifft()
	{
		const unsigned int h = length / 2;
		for(unsigned int k = 0; k <= h; ++k)
			work[k] = { frequencyDomain[k].r, frequencyDomain[k].i };
		for(unsigned int k = h + 1; k < length; ++k)
			work[k] = std::conj(work[length - k]);
		transform(true);
		for(unsigned int n = 0; n < length; ++n)
			timeDomain[n] = (float)work[n].real();
	}
	float& fdr(unsigned int n) { return frequencyDomain[n].r; }
	float& fdi(unsigned int n) { return frequencyDomain[n].i; }
	float fda(unsigned int n) { return sqrtf(fdr(n) * fdr(n) + fdi(n) * fdi(n)); }
	float& td(unsigned int n) { return timeDomain[n]; }
	static bool isPowerOfTwo(unsigned int n) { return n && !(n & (n - 1)); }

private:
	// iterative radix-2, in place on `work`, in double for headroom
	void transform(bool inverse)
	{
		const unsigned int n = length;
		for(unsigned int i = 1, j = 0; i < n; ++i)
		{
			unsigned int bit = n >> 1;
			for(; j & bit; bit >>= 1)
				j ^= bit;
			j ^= bit;
			if(i < j)
				std::swap(work[i], work[j]);
		}
		for(unsigned int len = 2; len <= n; len <<= 1)
		{
			const unsigned int step = n / len;
			for(unsigned int i = 0; i < n; i += len)
			{
				for(unsigned int k = 0; k < len / 2; ++k)
				{
					std::complex<double> w = twiddle[k * step];
					if(inverse)
						w = std::conj(w);
					const std::complex<double> u = work[i + k];
					const std::complex<double> v = work[i + k + len / 2] * w;
					work[i + k] = u + v;
					work[i + k + len / 2] = u - v;
				}
			}
		}
	}

	unsigned int length = 0;
	std::vector<float> timeDomain;
	std::vector<Cpx> frequencyDomain;
	std::vector<std::complex<double>> work;
	std::vector<std::complex<double>> twiddle;
};
