# REFFT offline

The stamp on the desktop: the board's own `cpp/Stamp.cpp` and
`cpp/Limiter.cpp`, compiled against a portable stand-in for Bela's `Fft`
(`include/libraries/Fft/Fft.h`), driven through a copy of `render.cpp`'s
signal path (`engine.h`). Three tools:

- `refft-render`: renders a drone and a field (files or test signals) to a
  WAV file with any knob settings, sweeps or ramps, and prints levels and
  features as it goes. For listening at the desk and A/B comparisons.
- `refft-check`: the predictions in `cpp/LISTENING.md`, measured. Each
  line prints the number next to what it expects; `obs` lines measure
  behaviour without a verdict.
- `refft-gen`: writes the listening kit: field signals for the board's
  sampler and drones for offline renders.

```sh
make            # builds into build/
make check      # runs refft-check (under 10 s); exit status = failures
make kit        # writes kit/sampler/ and kit/drones/
```

Needs only a C++14 compiler. WAV in and out only; convert anything else
first (`sox in.flac out.wav`). `build/`, `kit/` and `renders/` are not
tracked.

## How close it is to the board

Same: `Stamp.cpp` and `Limiter.cpp` unchanged; the input gains, circular
buffers, hop scheduling, overlap-add, size-change fade, mix, bypass, out R
gain and limiter, sample for sample. Files at other rates are resampled with
the sampler's cubic interpolation.

Different:

- The knob mappings and the mix are **copied** from `render()`, not shared.
  A change to them in `render.cpp` has to be made in `engine.h` too; each
  copied block there names what it mirrors.
- The stamp task runs inline instead of on another core. Its output lands
  one hop later either way, so this only differs if the board skips frames
  (it prints "frames skipped" when it does).
- The FFT is a plain radix-2 in double precision, not NE10. Results differ
  in the last bits, not audibly.
- Knobs are set directly: no 5 ms smoothing, no pickup. A stepped sweep
  changes a value at the step.
- No codec. The board's inputs see the codec's hiss; `--adc-noise -95` adds
  white noise at an assumed level. Its real level on this Pepper hasn't been
  measured.

## refft-render

```sh
build/refft-render -d DRONE -f FIELD -o OUT.wav [options]
```

`DRONE` and `FIELD` are WAV files or test signals: `sine:220`, `saw:110@-12`
(peak dBFS, default −6), `steps:220,330,440` (1 s each), `white:-40`,
`pink:-40` (RMS dBFS), `birds`, `whistle`, `gaps`, `silence`.

Knobs are positions 0–1, as on the panel and in the GUI, by name or as
A0–A7 / B0–B7:

| A | name | B | name |
| --- | --- | --- | --- |
| A0 | `drone-gain` | B0 | `out-gain` |
| A1 | `field-gain` | B1 | `attack` |
| A2 | `whiten` | B2 | `floor-speed` |
| A3 | `contrast` | B3 | `event-thresh` |
| A4 | `floor-only` | B4 | `dry-source` |
| A5 | `smear` | B5 | `cv-smooth` |
| A6 | `tonality` | B6 | `event-low` |
| A7 | `wet` | B7 | `event-high` |

Unset knobs sit at the board's boot defaults (wet 50%, whiten 0.7, contrast
0.33, everything else as in `render.cpp` `setup()`).

- `--set KNOB=V` sets a knob for the whole render.
- `--sweep KNOB=V,V,...` steps through values, `--step` seconds each
  (default 4), with a click at each step and an Audacity label file
  (`OUT.txt`). Also sweeps `size` (1024…8192), `freeze` and `bypass` (0/1).
- `--ramp KNOB=A:B` moves a knob linearly over the whole render.
- `--size`, `--freeze-at SEC`, `--seconds`, `--tap out|wet|dry`,
  `--adc-noise DB`, `--drone-ch`/`--field-ch` (default: the channels' mean).

The report has one row per step (measured over its last three quarters) or
per 2 s: input levels, the stamp alone (wet), out R, the most limiting, and
the field's flatness, brightness and event amount.

Examples:

```sh
# whiten from product to full transfer, on the synthetic field
build/refft-render -d kit/drones/drone-saw110.wav -f kit/sampler/test-birds.wav \
    -o renders/whiten.wav --set wet=1 --sweep whiten=0,0.25,0.5,0.75,1

# a real recording, the drone from a file, tonality swept slowly
build/refft-render -d ~/drones/organ.wav -f ~/field/5am.wav -o renders/tonality.wav \
    --set wet=1 --ramp tonality=0:1 --seconds 60
```
