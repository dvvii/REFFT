// refft-gen DIR: write the listening kit.
//   DIR/sampler/  field signals for the board's sampler (24-bit, 44.1 kHz):
//                 copy them into the project's samples/ folder
//   DIR/drones/   drones for offline renders (on the board the drone comes
//                 from the modular)

#include <cstdio>
#include <string>
#include <sys/stat.h>
#include "signals.h"
#include "wav.h"

static constexpr float kFs = 44100;

static bool save(const std::string& path, const Signal& x)
{
	Wav w;
	w.rate = kFs;
	w.channels = 1;
	w.data = x;
	if(!writeWav(path, w, true))
	{
		fprintf(stderr, "can't write %s\n", path.c_str());
		return false;
	}
	printf("%-44s %5.1f s  RMS %6.1f dBFS\n", path.c_str(), x.size() / kFs, ampToDb(rmsOf(x)));
	return true;
}

int main(int argc, char** argv)
{
	if(argc != 2)
	{
		fprintf(stderr, "usage: refft-gen DIR\n");
		return 2;
	}
	const std::string dir = argv[1];
	const std::string s = dir + "/sampler/", d = dir + "/drones/";
	mkdir(dir.c_str(), 0755);
	mkdir(s.c_str(), 0755);
	mkdir(d.c_str(), 0755);
	const size_t sec30 = (size_t)(30 * kFs), sec60 = (size_t)(60 * kFs);
	bool ok = true;

	// noise at three levels: the whiten crossover moves with the field's level
	for(int db : { -20, -40, -60 })
	{
		ok &= save(s + "test-white" + std::to_string(db) + ".wav", white(sec30, (float)db, 10 + db));
		ok &= save(s + "test-pink" + std::to_string(db) + ".wav", pink(sec30, (float)db, 20 + db));
	}
	// 1 s of pink noise, 1 s of digital silence: what the stamp does when the field stops
	ok &= save(s + "test-gaps.wav", gaps(sec30, kFs, -30, 1, 1));
	// a 3 s 4 kHz whistle every 8 s over a pink bed: one long event
	ok &= save(s + "test-whistle.wav", whistle((size_t)(32 * kFs), kFs, -40, 4000, -25, 3, 8));
	// pink bed, chirping birds in the event band, frog pulses below it
	ok &= save(s + "test-birds.wav", birds(sec60, kFs, -40, -22, -28));

	ok &= save(d + "drone-sine220.wav", sine(sec30, kFs, 220, -6));
	ok &= save(d + "drone-sine1000.wav", sine(sec30, kFs, 1000, -6));
	ok &= save(d + "drone-saw110.wav", saw(sec30, kFs, 110, -6));
	// four notes, 1 s each, round and round: for smear, attack and freeze
	ok &= save(d + "drone-steps.wav", steps((size_t)(32 * kFs), kFs, { 220.f, 277.18f, 329.63f, 440.f }, 1.f, -6));
	return ok ? 0 : 1;
}
