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
| 1 | drone gain (off → 0 dB at centre → +12 dB) | output gain (same curve) |
| 2 | field gain (off → 0 dB at centre → +12 dB) | mask attack (0–2 s) |
| 3 | **whiten**: product (0) → full magnitude transfer (1) | floor tracker speed (0.5–30.5 dB/s) |
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

## Scope

The IDE scope has four channels: in L and in R (raw, before the gain knobs),
out L and out R (as written to the outputs). Its FFT mode gives the spectrum
of each.

## CV out (0–1 → 0–5 V)

1. drone level (−60…0 dB)
2. field level
3. field noisiness (spectral flatness, 100 Hz–10 kHz)
4. field brightness (centroid, 50 Hz–10 kHz, log)
5. onset gate from the field (spectral flux, 20 ms)
6. event amount (share of field energy above its tracked floor)
7. wet level
8. drone brightness (centroid)

## How the stamp works

This section follows one frame through `Stamp::processFrame()` (`Stamp.cpp`)
and the code around it in `render.cpp`. Symbols: `D` is the drone spectrum
(input L), `C` the carrier spectrum (the field recording, input R), `k` a bin
index, `N` the FFT size, `hop = N/4`.

### The idea in one line

For every bin, the output takes its **magnitude from the drone** and its
**phase from the field recording**:

    out[k] = |D[k]| · C[k] / |C[k]|

`C/|C|` is a unit-length phasor: the field recording's phase with its level
removed ("whitened"). The drone's harmonics come out with the random,
constantly moving phase of the noise, so they sound breathy, grainy and part
of the recording rather than played on top of it. Everything else in the
engine is a way to move away from this pure form.

Why it isn't a vocoder: a vocoder smooths the modulator into a coarse
spectral envelope (a few bands) and imposes that on the carrier. Here each
bin is kept separate, so the drone's individual partials (21.5 Hz bins at
N = 2048) are imposed, not its envelope. The fine harmonic structure goes
through; the noise only provides texture.

### 1. Framing and overlap (render.cpp)

- The audio thread writes both inputs (after the gain knobs) into circular
  buffers `gIn[0]` (drone) and `gIn[1]` (field).
- Every `hop` samples it marks the frame end `gFrameEnd` and schedules
  `stampTask` on another core. The task copies the last `N` samples of
  each input.
- The task's output frame (`N` samples) is **added** into `gOut` starting
  `hop` samples after the frame end. The audio thread reads `gOut` at its
  write position and zeroes it behind itself.
- So input sample `i` reaches the output `N + hop` samples later: 58 ms at
  2048, 29 ms at 1024, 232 ms at 8192. The task has one hop of time
  (11.6 ms at 2048) to finish; if it is still busy when the next hop comes,
  that frame is skipped and counted ("frames skipped").
- The dry path (out L's dry component) is **not** delayed, so dry and wet
  are offset by that latency.

### 2. Analysis

- Both frames are multiplied by a periodic Hann window `w[n]` and
  transformed (`Fft`, NEON backend).
- Magnitudes are scaled by `magNorm = 2 / Σw`, so a full-scale sine reads
  1.0 (0 dB) in its bin at any FFT size. All thresholds below are in this
  scale.
- How noise lands in this scale: white noise of RMS `σ` gives bins of
  average magnitude about `2σ·√(1.5/N)`, i.e. 0.054·σ at N = 2048. A field
  recording at −35 dBFS RMS therefore has bins around −60 dB. Doubling `N`
  lowers noise bins by 3 dB, while a sine stays put. Keep this in mind for
  the whiten threshold.

### 3. The drone mask: contrast, smoothing, freeze

The drone magnitude `md = |D[k]|` becomes a per-bin **mask**:

1. **Contrast** (knob A4, exponent `c = 0.5·8^v`, 0.5…4, 1 at about 1/3):

       md = peak · (md / peak)^c

   `peak` is the loudest drone bin in the frame, which stays where it is.
   `c > 1` pushes quieter bins down (only the strongest partials survive,
   cleaner and more tonal); `c < 1` lifts them (the drone's noise and
   sidebands come through, denser).
2. **Smoothing** per bin, once per frame, one-pole with separate times for
   rising and falling:

       mask[k] += coef · (md − mask[k])     coef = attack if md > mask[k], else release

   Attack is knob B2 (τ = 2v² s), release is the **smear** knob A6
   (τ = 5v² s). The coefficients are computed from the hop length, so the
   times are the same at every FFT size. Long release keeps partials ringing
   after the drone moves: spectral sustain.
3. **Freeze** (button 2) stops updating `mask`. The current spectrum is held
   indefinitely and the drone input no longer matters until you unfreeze.

### 4. Whitening the carrier: the whiten knob

The core of the stamp. `mc = |C[k]|`, threshold `T = 10^((−6 − 94·v)/20)`,
so knob A3 moves `T` from −6 dB (v = 0) down to −100 dB (v = 1):

    stamped = mask[k] · 1            if mc > T    (carrier level discarded)
    stamped = mask[k] · mc / T       if mc ≤ T    (carrier level kept)

This is the formula `mask · |C| · (1/|C|)` with the whitening gain `1/|C|`
capped at `1/T`:

- **Bins above `T`** are fully whitened: their output level is the mask
  alone. The noise's own fluctuation is removed, and the result is the
  smoothest, most "drone-like" texture.
- **Bins below `T`** keep their relative level: the output is the
  **spectral product** `mask · mc`, boosted by `1/T`. The noise's
  bin-to-bin and frame-to-frame fluctuation shows up as grain, and quiet
  parts of the recording stay quiet (a squelch).

Working through the extremes:

- **v = 0** (`T` = −6 dB): every noise bin is below `T`, so the output is
  the pure product `2 · mask · mc`. With your field recording around −60 dB
  per bin, that's very quiet and very grainy, because the output follows
  the recording's level bin by bin.
- **v = 1** (`T` = −100 dB): every bin is above `T`, so the output is
  `mask` with noise phase: full magnitude transfer, as loud as the drone,
  regardless of how quiet the recording is.
- **In between**: the knob position where `T` meets the recording's noise
  level is the crossover. For bins around −60 dB that's `v ≈ 0.57`. Below
  it the recording's dynamics shape the output; above it they are flattened
  out. Because noise bins drop 3 dB per FFT size step, the crossover moves
  a little when you change size.

The whiten knob therefore also acts as a level control for the wet path at
low settings.

### 5. Floor tracking and "floor only"

Each carrier bin has a noise-floor estimate `fl`:

    if mc < fl:  fl = mc                       (follow drops at once)
    else:        fl = fl · rise                (creep up by B3 dB/s, 0.5…30.5)

This is a minimum tracker: it sits at the low edge of the noise
fluctuation, not at its average. Typical noise frames read a few dB above
it, so the event threshold needs to sit well above 0 dB.

How far a bin is above its floor, `above = 20·log10(mc / fl)`, becomes an
**event** amount, a soft gate 12 dB wide centred on the event threshold `E`
(knob B4, 3…30 dB, about 12 dB at its default):

    event = clamp((above − (E − 6)) / 12, 0, 1)

The **floor only** knob (A5) blends each bin between stamped and untouched:

    w   = 1 − floorOnly · event
    mag = stamped · w + mc · (1 − w)

At `floorOnly = 0` every bin is stamped. At `floorOnly = 1`, bins where
something stands out of the noise (a bird, a footstep, a gust) keep the
recording's own magnitude, and because they also keep its phase, those
sounds pass through the wet path unchanged (delayed by the latency). Only
the steady noise bed is replaced by the drone. The rise speed (B3) decides
how quickly the floor catches up after a quiet moment: slow makes long
events stay "events" longer, fast absorbs them into the floor.

### 6. Phase and tonality

The carrier phasor `u = C[k] / |C[k]|` (unit length) normally gives the
output its phase. The **tonality** knob (A7, `t`) blends it with the drone's
phasor `d = D[k] / |D[k]|` and renormalises:

    b = (1 − t)·u + t·d,   phase = b / |b|

- `t = 0`: noise phase, the breathy stamp described above.
- `t = 1`: drone phase. The output is then the drone itself, shaped by the
  mask and by the carrier's level where whiten lets it through: the
  recording "plays" the drone instead of colouring the noise.
- In between, partials become partly coherent: pitched but unstable. This
  is a vector blend, not an angle interpolation, so where `u` and `d` point
  in nearly opposite directions the result swings abruptly. That's part of
  the sound at middle settings.

### 7. Resynthesis

    out[k] = phase · mag · outScale       (DC set to 0; DC and Nyquist bins kept real)

- Inverse FFT, multiply by the same Hann window (synthesis window), and
  overlap-add four frames per hop.
- `outScale = 1 / (magNorm · ifftGain) · hop / Σw²` undoes the magnitude
  normalisation, the backend's inverse-FFT gain (measured at setup) and the
  sum of the squared windows at overlap 4. A steady signal at full transfer
  therefore comes out at the level of the mask: a −6 dB drone partial gives a
  −6 dB output partial.

### 8. Mix and output (render.cpp)

- **Wet/dry** (A8) is an equal-power crossfade: `wet · sin(v·π/2)` +
  `dry · cos(v·π/2)`.
- The dry signal is itself a blend between field and drone (**dry source**,
  B5, also equal-power), so out L can be the stamped texture over the
  original recording or over the drone.
- **Bypass** (button 4) fades out L to the dry signal and out R to silence
  over about 10 ms.
- **Output gain** (B1) applies to both outputs, `4v²`, unity at centre.
- **FFT size** (button 3) fades the wet path out over 20 ms, waits for the
  task to finish, switches size (which resets the mask, floor and flux
  state), clears the overlap buffer and fades back in.

### 9. Analysis features (CV outs)

Computed in the same per-bin loop, over 100 Hz…10 kHz, from the field
recording unless noted:

- **Flatness**: geometric over arithmetic mean of bin power. Pure white
  noise reads about 0.56, not 1, because the power of a noise bin varies
  frame to frame; tones and silence read lower.
- **Centroid**: magnitude-weighted mean frequency, mapped on a log scale
  50 Hz…10 kHz. The drone gets one too (CV 8).
- **Flux**: the sum of bin magnitude increases since the last frame,
  divided by the total magnitude. The onset gate (CV 5) fires for 20 ms
  when flux exceeds twice its running average plus 0.02.
- **Event amount**: the magnitude-weighted average of `event` from step 5,
  i.e. how much of the recording currently stands out of its floor.
- **Levels**: RMS of each input frame (after the gain knobs) and the
  envelope of the wet output, on a −60…0 dB scale.

All CV outs except the gate are smoothed by the CV smoothing knob (B6).

### Things to try while working through it

- Sine drone (your self-oscillating filter) into a noisy field recording,
  whiten at 1, contrast 1, everything else at 0: you should hear a single
  breathy partial at the sine's pitch. Then lower whiten and listen for the
  point where the recording's grain starts to come through (around the
  crossover in step 4).
- Same setup, raise tonality: the partial goes from breath to tone.
- A recording with birds or voices, floor only at 1: the noise bed becomes
  drone, the events stay. Sweep the event threshold (B4) to decide what
  counts as an event.
- Freeze a rich drone moment, then change the drone: the frozen spectrum
  keeps playing through the recording.
- Compare FFT sizes on the same material: 1024 is faster and smears
  partials across more bins (43 Hz each); 8192 resolves close partials (5 Hz
  bins) but blurs the timing.

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
