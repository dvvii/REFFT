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
| Audio out L | the sampler, clean (see Sampler) |
| Audio out R | the stamp: dry/wet mix |

To stamp a recording from the sampler, patch out L into in R. Nothing is
routed internally, so the stamp sees exactly what any other source would.

Input gain is −3 dB (`settings.json`), set for modular level: a self-oscillating
filter sine peaks around −4 dBFS. The stock Pepper +10 dB clips modular
signals by about 10 dB. The codec goes down to −12 dB.

## Knobs / CV in

Layer A is live. Hold button 1 to reach layer B; knobs pick up (they don't
act until they reach the stored value) after every layer change.

The knobs on this Pepper don't reach the top of the analog range: at full
clockwise they read 0.87–0.96 depending on the knob. `Panel::kKnobTop` holds
each knob's measured top, and readings are scaled by it so every knob spans
0–1. Measure again (full CW, with the GUI's raw knob positions or
`analogRead`) if the module or its knobs change.

| Knob | Layer A | Layer B |
|---|---|---|
| 1 | drone gain (off → 0 dB at centre → +12 dB) | out R gain (off, then −24…+24 dB; 0 dB at centre) |
| 2 | field gain (off → 0 dB at centre → +12 dB) | mask attack (0–2 s) |
| 3 | **whiten**: product (0) → full magnitude transfer (1) | floor tracker speed (0.5–30.5 dB/s) |
| 4 | **contrast**: drone mask exponent 0.5–4 (1 at about 1/3) | event threshold (3–30 dB above floor) |
| 5 | **floor only**: stamp only the field's noise floor, pass events | dry source: field (0) → drone (1) |
| 6 | **smear**: mask release (0–5 s) | CV out smoothing (1 ms–0.5 s) |
| 7 | **tonality**: field phase (0) → drone phase (1) | event band edge (100 Hz–12.8 kHz, 2 kHz) |
| 8 | wet/dry | event band edge (same range, 10 kHz) |

## Buttons

1. shift (hold) for layer B
2. freeze the drone mask
3. cycle FFT size 1024 / 2048 / 4096 / 8192 (wet path fades out and back in)
4. bypass

## LEDs

LEDs 1–8 are level meters, two LEDs each: in L (1–2), in R (3–4), out L
(sampler, 5–6), out R (stamp, 7–8). Brightness is PWM (1 kHz), on a dB scale from −48 to
0 dBFS: the first LED of a pair covers −48…−24 dB, the second −24…0 dB.
Input meters read the jacks before the gain knobs. For a second after a size
change they show the FFT size instead. LED 9 is freeze, LED 10 is shift
(steady) or bypass (blinking).

## Scope

The IDE scope has four channels: in L and in R (raw, before the gain knobs),
out L (sampler) and out R (stamp), as written to the outputs. Its FFT mode gives the spectrum
of each.

## Sampler (browser GUI)

A built-in player for field recordings of any length, streamed from the SD
card, controlled from the Bela GUI at `http://bela.local/gui` while the
project runs. It plays on out L only; patch out L into in R to stamp it.

- **Files**: audio files (wav, aif, flac, ogg, mp3, caf, w64) in the project
  folder or its `samples/` folder. Upload them with the IDE, or
  `rsync -a recordings/ bbb:Bela/projects/refft-cpp/samples/`, then press
  rescan. A file starts playing as soon as it is picked.
- **Waveform**: drag an edge to move it, drag elsewhere to draw a new region,
  click to move the playhead, double-click for the whole file. The first time
  a file is picked it is read through once for its waveform, loudness and
  peak (about 10–20 s for 45 minutes); the result is cached in `.cache/`.
- **Transport**: play/stop (space), back to the region start (enter), loop
  (`l`; off = one shot), normalise (see below; the file's level and the gain
  applied are shown next to its name, and normalisation glides in once the
  file has been read through), and for multichannel files mix / L / R (keeps
  the playhead).
- **Sliders**: gain −36…+24 dB (a trim on top of normalise), crossfade
  2 ms–2 s. Drag, shift-drag for fine steps, wheel to nudge, double-click to
  reset.
- **Touch and pencil** (iPad, phone): input comes from Pointer Events on the
  canvas, so mouse, finger and Apple Pencil behave alike. A tap is a click,
  a double tap a double-click, and a drag works as with the mouse. Swiping
  the file list scrolls it, and a file loads on a tap, so a swipe doesn't
  load the file under the finger. The page itself doesn't scroll or zoom.
- **Meters**: in L, in R, out L (sampler), out R (stamp). Disk underruns, if any, are
  shown next to the file name.
- **Panel** (display only): buttons 1–4 (layer, freeze, FFT size with its
  latency and the stamp's whiten make-up gain, bypass) and the stored value of every knob on both layers, in the
  units of the table above. The live layer is lit, and the knob being turned
  is yellow. A knob waiting for pickup reads "turn to …", with an orange tick
  at the physical knob's position.
- **CV outs**: the eight outputs as 0–5 V bars, labelled as in the CV out
  list below.
- **Spectrum**: in R in 42 bands (100 Hz–12.8 kHz), each lit yellow by how
  much of it counts as events, with the event band (B7, B8) marked. Use it
  to set the band around the birds and leave out the frogs.

**Normalise and the limiter.** Field recordings peak at 0 dBFS from a few
bumps while their ambience sits 25–55 dB lower, so scaling to the peak does
nothing. Normalise instead measures the median of 100 ms RMS levels (the
ambience) and brings it to −26 dBFS, unless that would push the loudest 0.1%
of blocks more than 12 dB into the limiter; then it gives less gain (a quiet
bed with loud calls). The gain is clamped to −20…+40 dB. A lookahead peak
limiter (1.5 ms lookahead, 100 ms release, −1 dBFS ceiling) on out L catches
what this and the trim push up; the GUI shows when it is reducing.

Playback is at the original speed (files at another sample rate are
resampled with cubic interpolation). Two voices each stream through a 3 s
ring buffer that a disk thread keeps full. The idle voice waits filled at the
region start, so a loop point is an instant equal-power crossfade, capped at
half the region so short loops still crossfade. A seek or a new file
re-arms the idle voice there and crossfades once it holds 4096 frames,
typically within a few ms. Streaming one file takes 176 kB/s from the SD
card and about 1 MB of memory.

Settings (file, region, play state, loop, sliders) are
saved to `sampler.cfg` in the project folder two seconds after the last
change, and restored at boot, so a startup project plays its recording
without a browser. Delete the file to start from defaults.

## CV out (0–1 → 0–5 V)

1. drone level (−60…0 dB)
2. field level
3. field noisiness (spectral flatness, 100 Hz–10 kHz)
4. field brightness (centroid, 100 Hz–4 kHz, log)
5. onset gate from the field's event band (spectral flux, 20 ms)
6. event amount (share of the event band's energy above its tracked floor)
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
- The dry path (out R's dry component) is **not** delayed, so dry and wet
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

Left alone, this made whiten a level control for the wet path as well:
with a quiet field the product end sat 40–80 dB below the transfer end.
Static compensation (step 7) takes most of that out, so whiten and
contrast change the character of the stamp rather than its loudness.

### 5. Floor tracking and "floor only"

Each carrier bin has a smoothed level `lv` (its power through a 50 ms
one-pole, square-rooted) and a noise-floor estimate `fl` of that level:

    if lv < fl:  fl = lv                       (follow drops at once)
    else:        fl = fl · rise                (creep up by B3 dB/s, 0.5…30.5)

This is a minimum tracker: it sits at the low edge of the noise
fluctuation, not at its average. The smoothing narrows that fluctuation;
tracking the raw magnitude instead put the floor at the deepest dips of
each noise bin, 15–20 dB under its average, so plain noise read as events.
Typical noise frames still read a few dB above the floor, so the event
threshold needs to sit well above 0 dB.

Only bins in the **event band** (B7 and B8 set its edges, either way round;
default 2–10 kHz) can be events, with half-octave tapers outside it. In the
Calanoa recordings the frogs sit around 350–500 Hz and the birds above
2 kHz, so the default band leaves the frog chorus to the floor.

How far a bin is above its floor, `above = 20·log10(mc / fl)`, becomes an
**event** amount, a soft gate 12 dB wide centred on the event threshold `E`
(knob B4, 3…30 dB, about 12 dB at its default), weighted by the band:

    event = band(f) · clamp((above − (E − 6)) / 12, 0, 1)

The **floor only** knob (A5) blends each bin between stamped and untouched:

    w   = 1 − floorOnly · event
    mag = stamped · w + mc · (1 − w)

At `floorOnly = 0` every bin is stamped. At `floorOnly = 1`, bins in the
event band where something stands out of the noise (a bird call) keep the
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
- **Level compensation** is static, with nothing that follows the carrier
  from frame to frame. An earlier version scaled each frame to the carrier's
  power smoothed over 0.4 s. With dense birdsong (the 5am recording) that
  pumped: every call pulled the drone up and down with it. Two fixed gains
  replace it:
  - **Contrast** (A4) scales the sharpened mask back to the drone's own
    power, so sharpening partials doesn't also turn the stamp down.
  - **Whiten make-up**: bins below the whiten threshold are a spectral
    product (carrier × drone), so their level depends on the carrier's.
    They get `threshold − ref − 6` dB of make-up (0…60 dB, shown in the GUI
    next to the FFT size). `ref` is the carrier's mean bin power, averaged
    in dB over 10 s, so one bird call barely moves it. A carrier at its
    usual level then lands 6 dB under the drone.

  The wet level therefore follows the drone (A1), not the recording. In a
  model using the 5am recording and a −15 dBFS drone, it stayed between −16
  and −23 dBFS across the whole A3 × A4 range, at carrier levels of both
  −26 and −45 dBFS. The swing of the 100 ms level with whiten up (the drone
  alone) fell from 3.5 dB under level matching to 0.3–1.5 dB. With whiten
  down, the stamp follows the birds as the recording does, and their peaks
  go to the out R limiter.

### 8. Mix and output (render.cpp)

- **Wet/dry** (A8) is an equal-power crossfade: `wet · sin(v·π/2)` +
  `dry · cos(v·π/2)`.
- The dry signal is itself a blend between field and drone (**dry source**,
  B5, also equal-power), so out R can be the stamped texture over the
  original recording or over the drone; fully wet it is the stamp alone.
- **Bypass** (button 4) fades out R to the dry signal over about 10 ms.
- **Output gain** (B1) applies to out R: off at the bottom, then −24…+24 dB
  linear in dB, 0 dB at centre. The stamp sits near the drone's level, and
  the limiter holds the peaks, so this knob takes it up to modular level.
- **Limiter**: out R then goes through the same lookahead limiter as the
  sampler (`Limiter.h`, −1 dBFS, 1.5 ms), so high gain settings don't clip.
  The GUI shows any limiting next to the out R meter. Out L (the sampler)
  has its own gain in the GUI.
- **FFT size** (button 3) fades the wet path out over 20 ms, waits for the
  task to finish, switches size (which resets the mask, floor and flux
  state), clears the overlap buffer and fades back in.

### 9. Analysis features (CV outs)

Computed in the same per-bin loop, over 100 Hz…10 kHz, from the field
recording unless noted:

- **Flatness**: geometric over arithmetic mean of bin power. Pure white
  noise reads about 0.56, not 1, because the power of a noise bin varies
  frame to frame; tones and silence read lower.
- **Centroid**: the power-weighted mean of log frequency, mapped 100 Hz…
  4 kHz (where these recordings sit; it measures over 100 Hz…10 kHz). Pink noise reads mid-scale, white noise near the top, brown noise
  near the bottom. (A magnitude-weighted mean of linear frequency put
  almost everything near the top, as most bins are high.) The drone gets
  one too (CV 8).
- **Flux**: the sum of bin magnitude increases since the last frame, in the
  event band, divided by the band's total magnitude. The onset gate (CV 5) fires for 20 ms
  when flux exceeds twice its running average plus 0.02.
- **Event amount**: the magnitude-weighted average of `event` from step 5
  over the event band, i.e. how much of the band currently stands out of
  its floor.
- **Levels**: RMS of each input frame (after the gain knobs) and the
  envelope of the wet output, on a −60…0 dB scale.

All CV outs except the gate are smoothed by the CV smoothing knob (B6).
Below −60 dB, where the level CVs read 0, noisiness, brightness, the onset
gate and the event amount fade to 0 over 6 dB (drone brightness follows the
drone's level): at that point they would only describe the codec's noise.

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
- `Sampler.{h,cpp}`: the field-recording player. A disk thread fills the
  two voices' ring buffers (libsndfile); a loader thread scans and opens
  files, analyses them (peak, overview, cached) and saves `sampler.cfg`. The
  audio thread requests a stream position by bumping a generation number and
  only touches atomics and the rings: no allocation, locks or I/O.
- `sketch.js`: the p5.js GUI. `render.cpp` sends it a status buffer every
  33 ms plus the file list, waveform overview and loader messages; it sends
  back `{address, value}` control messages (`onGuiControl`). A value being
  edited shows the local copy for 600 ms so the echo doesn't fight the drag;
  when the status names a file list or clip the page doesn't have (it
  connected late, or the project restarted), the page asks for `/sync`.
- `Limiter.h`/`.cpp`: the lookahead peak limiter on both outputs.
- `Panel.h`: Pepper knobs (layers, pickup), buttons (debounced; the state at
  boot counts as released), LEDs.
- `render.cpp`: I/O, parameter mapping, features to CV. The spectral work runs
  in an auxiliary task (priority 90) on another core; it overlap-adds one hop
  ahead of the audio read position. Latency is FFT size + hop (58 ms at 2048).

## Build and run

```sh
rsync -a --exclude build --exclude sampler.cfg --exclude samples ./ bbb:Bela/projects/refft-cpp/
ssh bbb 'make -C ~/Bela PROJECT=refft-cpp run'
```

Measured at FFT 2048: the spectral task uses about 2% of one core, the audio thread about 11%.
