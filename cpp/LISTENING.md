# Learning the stamp by ear

A course in listening to REFFT, one stage of the algorithm at a time, with
test signals whose result can be predicted before it is heard. Each lesson
says what to set, what should happen and what the desktop model measured.
When the ear agrees, that stage is understood; when it doesn't, either the
code or its explanation is wrong, and both are worth knowing. The field
recordings and drones come last, once each knob has a sound of its own.

The stages are the ones in `README.md`, "How the stamp works"; each lesson
names its section. The numbers come from `offline/` (`make check`), which
runs the board's `Stamp.cpp` on the desktop; see `offline/README.md`.

## Setting up

**Patch.** Modular sine or saw into **in L** (drone). Sampler **out L**
into **in R** (field). **Out R** to the monitors. Keep the monitors low
for lesson 2b.

**The kit.** On the Mac, in `offline/`: `make kit`, then copy the field
signals to the board and press rescan in the GUI:

```sh
rsync -a offline/kit/sampler/ bbb:Bela/projects/refft-cpp/samples/
```

| File | What it is |
| --- | --- |
| `test-white-20/40/60`, `test-pink-20/40/60` | noise at −20, −40, −60 dBFS RMS |
| `test-gaps` | pink noise at −30 dBFS, 1 s on, 1 s of digital silence |
| `test-whistle` | a pink bed at −40, with a 4 kHz tone at −25 dBFS for 3 s every 8 s |
| `test-birds` | a pink bed at −40, chirps between 2.5 and 6 kHz (inside the event band), 450 Hz pulse trains (below it) |

**Sampler.** Normalise **off** and gain 0 dB, so the files keep their
levels relative to each other. The level that reaches in R also depends on
the out L → in R round trip, so read it on the GUI's in R meter rather than
trusting the file's number.

**The clean position.** Most lessons start here and change one knob. It
takes the stamp down to its core: full transfer, no shaping, no smoothing,
the wet path alone.

| Knob | Clean position | GUI reads |
| --- | --- | --- |
| A0 drone gain, A1 field gain | centre | +0.0 dB |
| A2 whiten | full | 100% |
| A3 contrast | about a third | ^1.00 |
| A4 floor only, A5 smear, A6 tonality | off | 0%, 0 ms, 0% |
| A7 wet | full | 100% |
| B0 out gain | centre | +0.0 dB |
| B1 attack | off | 0 ms |
| B2 floor speed, B3 event threshold | boot defaults | 3.2 dB/s, 11.9 dB |
| B4 dry source | off | field |
| B6, B7 event band | boot defaults | 2.0 kHz, 10.0 kHz |

Layer B knobs only act once they reach their stored value (pickup); the GUI
shows "turn to …" until they do.

**At the desk.** Every lesson has a render that does the same thing
offline. Run them from `offline/` after `make`, and listen on headphones;
each sweep step starts with a click, and `renders/NAME.txt` labels the steps
in Audacity.

---

## 0. Levels: is it working at all?

*README: "The idea in one line", 7. Resynthesis.*

Sine drone at a mid pitch, about −6 dBFS on the in L meter. Field
`test-white-40`. Clean position.

- You hear one breathy, airy tone at the sine's pitch: the drone's
  magnitude with the noise's phase. Nothing else of the noise.
- The out R meter sits **about 1.5 dB under in L** (model: −1.4 dB).
  Consecutive frames share three quarters of their noise, so their phases
  mostly agree and the overlap-add loses little.
- Bypass (button 4) gives the noise back.

```sh
build/refft-render -d sine:1000 -f kit/sampler/test-white-40.wav -o renders/l0.wav \
    --set whiten=1 --set contrast=0.3333 --set wet=1 --seconds 10
```

Heard:

## 1. One partial, four FFT sizes

*README: 2. Analysis, "Things to try".*

Same setup. Sweep the sine's pitch on the modular, then step the FFT size
(button 3; the LEDs show the size for a second).

- The breathy partial follows the pitch exactly; it is a narrow band of
  noise centred on it.
- **1024**: wide and rough, a hiss with a pitch in it. **8192**: almost a
  whistle, and the response to the pitch knob lags visibly. The band's width
  halves with each step (model, RMS width: 31, 16, 8, 4 Hz).
- The delay grows with size: 29, 58, 116, 232 ms.

```sh
build/refft-render -d sine:1000 -f kit/sampler/test-white-40.wav -o renders/l1.wav \
    --set whiten=1 --set contrast=0.3333 --set wet=1 --sweep size=1024,2048,4096,8192
```

Heard:

## 2. Whiten: transfer or product

*README: 4. Whitening the carrier.*

Sine or saw drone. Field `test-pink-20`, then `-40`, then `-60`. Sweep A2
from full down to 0 on each.

- **Full**: the three files sound the same, and at the same level
  (model: −10.5 dBFS on all three). The noise's own level is discarded.
- **Towards 0**: the noise's grain comes through; the tone becomes rough,
  fluttering, "made of" the recording, and it gets quieter. Below the
  threshold the stamp is drone × field, so it follows the field dB for dB:
  at whiten 0 the three files come out 20 dB apart (model: −49, −69,
  −89 dBFS). Whiten is a level control as well as a texture control.
- **Where it turns**: the crossover sits where the threshold meets the
  field's bins, lower on the knob for quieter files. Listen for the point
  on each file where the level starts to drop.
- **Dynamics**: at whiten 0, switch from `test-pink-20` to `-40` while
  playing. The stamp drops 20 dB and stays there. At full whiten nothing
  happens.

```sh
build/refft-render -d saw:110 -f kit/sampler/test-pink-40.wav -o renders/l2.wav \
    --set contrast=0.3333 --set wet=1 --sweep whiten=0,0.25,0.5,0.75,1 --step 6
```

Heard:

## 2b. When the field stops

*README: 4. Whitening the carrier.*

Field `test-gaps`: a second of noise, a second of silence.

- **Whiten full**: during the gaps the sampler sends silence, but in R
  still sees the codec's hiss, and full transfer gives any bin above
  −100 dB the drone's full level. Expect **the drone to keep playing
  through the gaps**, nearly as loud. On the board in R sits at −72 dBFS
  with the sampler stopped, and the model puts the stamp of that hiss only
  2 dB under the drone. Field gain (A1) fully down makes in R exactly zero,
  and then the stamp is silent.
- **Whiten 0.7**: the gaps drop about 25 dB (model, board hiss).
- **Whiten 0**: the gaps go silent; the product follows the field down.
- **Stop the sampler and start it again**: the stamp comes back at its own
  level at once, at any whiten.

Heard:

## 3. Contrast

*README: 3. The drone mask (contrast).*

Saw drone. Field `test-white-40`. Clean position, then sweep A3.

- **^0.5** (bottom): dense, buzzy, the saw's whole spectrum plus its noise
  (model: 88 harmonics within 30 dB of the strongest).
- **^1**: the saw as it is (26).
- **^4** (top): almost one partial left, the strongest (2). Somewhere
  between, the harmonics drop out one by one, weakest first.
- The level stays put (model: within 2 dB across the range).

```sh
build/refft-render -d kit/drones/drone-saw110.wav -f kit/sampler/test-white-40.wav -o renders/l3.wav \
    --set whiten=1 --set wet=1 --sweep contrast=0,0.33,0.66,1
```

Heard:

## 4. Smear, attack, freeze

*README: 3. The drone mask (smoothing, freeze).*

A sine stepping between notes: a sequencer into the oscillator, or the
pitch knob turned in steps. Field `test-white-40`. Clean position.

- **Smear** (A5) up: each note keeps ringing after the next one starts,
  so the notes pile into a chord. At 0.6 (1.8 s on the GUI) a note fades
  about 5 dB per second (model: 9.5 dB over 2 s). At 0 a note stops with
  its step (65 dB down).
- **Attack** (B1) up: notes swell in. At 0.6 (0.72 s) a note is at
  two-thirds of its level after 0.72 s (model: −4.0 dB).
- **Freeze** (button 2): the current spectrum holds; new notes no longer
  come through at all (model: 64 dB down), and the drone input doesn't
  matter until you unfreeze.

```sh
build/refft-render -d kit/drones/drone-steps.wav -f kit/sampler/test-white-40.wav -o renders/l4.wav \
    --set whiten=1 --set contrast=0.3333 --set wet=1 --sweep smear=0,0.3,0.6,0.9 --step 8
build/refft-render -d kit/drones/drone-steps.wav -f kit/sampler/test-white-40.wav -o renders/l4f.wav \
    --set whiten=1 --set contrast=0.3333 --set wet=1 --freeze-at 2.5 --seconds 12
```

Heard:

## 5. Tonality

*README: 6. Phase and tonality.*

Sine, then saw drone. Field `test-white-40`. Clean position, then sweep A6.

- **0**: breath. **Full**: tone. Middle settings: pitched but unstable,
  with flutter where the two phases point in opposite directions.
- **Full, with the rest clean**: the output **is the drone**, 58 ms late.
  The model nulls the two to −74 dB, so this is the identity test of the
  whole framing and overlap-add. The level rises by the 1.5 dB lost in
  lesson 0.
- Then bring whiten down with tonality full: the drone now plays with the
  recording's dynamics. The recording "plays" the drone instead of
  colouring the noise.

```sh
build/refft-render -d kit/drones/drone-saw110.wav -f kit/sampler/test-white-40.wav -o renders/l5.wav \
    --set whiten=1 --set contrast=0.3333 --set wet=1 --sweep tonality=0,0.25,0.5,0.75,1
```

Heard:

## 6. Floor only and events

*README: 5. Floor tracking and "floor only".*

Saw or sine drone. Field `test-birds`, then `test-whistle`. Clean
position, then A4 floor only full. Watch the GUI spectrum: yellow bands are
events.

- **Birds**: the chirps pass through untouched (58 ms late) while the bed
  and the frog pulses (450 Hz, below the event band) turn into drone.
- **Event threshold** (B3) on `test-birds`: at the default 12 dB some of the
  bed's grain still passes, because noise peaks often sit 6 dB over the
  tracked floor (model: the bed only 15 dB under). At 19 dB: 29 dB under. At
  30 dB the bed is gone, and the chirps still pass.
- **Floor speed** (B2) on `test-whistle`: at full (30.5 dB/s) the floor
  climbs into the long tone, and it fades into the drone about 1.3 s after
  it starts. At the default it passes whole. At the slowest the floor sinks
  to the noise's deepest dips, and nearly the whole bed counts as events:
  floor only then does almost nothing.
- **Event band** (B6, B7) down to 100 Hz–1 kHz: the chirps no longer pass,
  and the frogs do.

```sh
build/refft-render -d kit/drones/drone-saw110.wav -f kit/sampler/test-birds.wav -o renders/l6.wav \
    --set whiten=1 --set contrast=0.3333 --set wet=1 --set floor-only=1 \
    --sweep event-thresh=0.33,0.6,1 --step 10
build/refft-render -d kit/drones/drone-saw110.wav -f kit/sampler/test-whistle.wav -o renders/l6b.wav \
    --set whiten=1 --set contrast=0.3333 --set wet=1 --set floor-only=1 \
    --sweep floor-speed=0,0.3,1 --step 16
```

Heard:

## 7. The mix and the delay

*README: 1. Framing, 8. Mix and output.*

Field `test-birds`, any drone, wet (A7) at centre, dry source (B4) on
field. Step the FFT size.

- The chirps flam: the dry ones, then the stamped or passed ones, 29, 58,
  116 or 232 ms later. At 8192 it is a clear echo.
- Dry source towards drone: the dry part becomes the drone, so out R is
  stamp over drone instead of stamp over recording.

Heard:

## 8. CV outs (optional)

*README: CV out, 9. Analysis features.*

Field `test-birds`: CV 4 (onset gate) fires on chirps, CV 5 (event amount)
rises with them, CV 3 (brightness) stays put on the steady bed. Field
`test-white-40` against `test-pink-40`: brightness near the top against
mid-scale.

Heard:

---

## Then the real material

Same order, on a drone and a recording you know: levels, size, whiten,
contrast, smoothing, tonality, floor only. Keep a line per setting worth
remembering: the knob positions as the GUI shows them, what it did, and the
lesson it belongs to. For an offline A/B of a recording, render it with
`--sweep` over the knob in question.

## Findings from the model

These came out of `make check` while writing this guide.

1. **A make-up gain turned silences into hiss at full level** (removed).
   Below full whiten it raised the product to the drone's level against
   the carrier's level over 10 s. On the board, with the sampler stopped,
   it climbed to 45 dB in a minute and lifted the codec's hiss to nearly
   full level; when the recording came back, out R's limiter took up to
   43 dB off and released over 17 s. The stamp now follows the recording's
   level below the whiten threshold instead (lesson 2).
2. **Full whiten keeps the drone alive over the codec's hiss** (lesson 2b).
   The drone doesn't stop when the recording does, unless field gain is
   fully down or whiten comes down.
3. **The noise-phase stamp is about 1.5 dB under the drone**, and exactly at
   its level with tonality full. README step 7's "a −6 dB drone partial gives
   a −6 dB output partial" holds for the drone's phase.
4. **The default event threshold leaks a little bed**, about 15 dB under the
   field's own, and the slowest floor speed turns most of the bed into
   events (lesson 6).
5. **FFT 1024 and low fundamentals.** The stamp zeroes the DC bin. At 1024 a
   110 Hz fundamental leaks slightly into it, so the identity test only nulls
   to −53 dB instead of −74. Inaudible; noted so the number doesn't surprise.
