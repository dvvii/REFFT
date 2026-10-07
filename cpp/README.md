# REFFT (C++)

Real-time timbre stamp for Pepper, running on a PocketBeagle 2 with a classic
Bela rev C cape (`ssh bbb`). The C++ successor of `../_main.pd`.

The field recording keeps its phase and noise; the drone supplies the
magnitude of every FFT bin. Noisy recordings take on the drone's harmonics as
a background layer. Per-bin, with no spectral-envelope smoothing, so it does
not behave like a vocoder.

## I/O

| | |
|---|---|
| Audio in L | drone (magnitudes) |
| Audio in R | field recording (carrier) |
| Audio out L | dry/wet mix |
| Audio out R | wet only |

Input gain is −3 dB (`settings.json`), set for modular level: a self-oscillating
filter sine peaks around −4 dBFS. The stock Pepper +10 dB clips modular
signals by about 10 dB. The codec goes down to −12 dB.

## Knobs / CV in

Layer A is live. Hold button 1 to reach layer B; knobs pick up (they don't
act until they reach the stored value) after every layer change.

| Knob | Layer A | Layer B |
|---|---|---|
| 1 | drone gain (0 to +12 dB) | output gain |
| 2 | field gain (0 to +12 dB) | mask attack (0–2 s) |
| 3 | **whiten**: product (0) → full magnitude transfer (1) | floor tracker speed (0.5–30 dB/s) |
| 4 | **contrast**: drone mask exponent 0.5–4 (1 at about 1/3) | event threshold (3–30 dB above floor) |
| 5 | **floor only**: stamp only the field's noise floor, pass events | dry source: field (0) → drone (1) |
| 6 | **smear**: mask release (0–5 s) | CV out smoothing (1 ms–0.5 s) |
| 7 | **tonality**: field phase (0) → drone phase (1) | — |
| 8 | wet/dry | — |

## Buttons

1. shift (hold) for layer B
2. freeze the drone mask
3. cycle FFT size 1024 / 2048 / 4096 / 8192 (wet path fades out and back in)
4. bypass

## LEDs

LEDs 1–8 are level meters, two LEDs each: in L (1–2), in R (3–4), out L
(5–6), out R (7–8). Brightness is PWM (1 kHz), on a dB scale from −48 to
0 dBFS: the first LED of a pair covers −48…−24 dB, the second −24…0 dB.
Input meters read the jacks before the gain knobs. For a second after a size
change they show the FFT size instead. LED 9 is freeze, LED 10 is shift
(steady) or bypass (blinking).

## CV out (0–1 → 0–5 V)

1. drone level (−60…0 dB)
2. field level
3. field noisiness (spectral flatness, 100 Hz–10 kHz)
4. field brightness (centroid, 50 Hz–10 kHz, log)
5. onset gate from the field (spectral flux, 20 ms)
6. event amount (share of field energy above its tracked floor)
7. wet level
8. drone brightness (centroid)

## Code

- `Stamp.{h,cpp}`: the per-frame spectral engine. Hann analysis/synthesis,
  overlap 4, magnitudes normalised so a full-scale sine reads 1.0 at every
  FFT size.
- `Panel.h`: Pepper knobs (layers, pickup), buttons (debounced; the state at
  boot counts as released), LEDs.
- `render.cpp`: I/O, parameter mapping, features to CV. The spectral work runs
  in an auxiliary task (priority 90) on another core; it overlap-adds one hop
  ahead of the audio read position. Latency is FFT size + hop (58 ms at 2048).

## Build and run

```sh
rsync -a --exclude build ./ bbb:Bela/projects/refft-cpp/
ssh bbb 'make -C ~/Bela PROJECT=refft-cpp run'
```

Measured at FFT 2048: the spectral task uses about 2% of one core, the audio thread about 11%.
