# HAPTIK — implementation plan

A build plan for a coding agent. This is the *what to build, in what order, and
how to know each step works*. For the *why* (the physics, the musical intent),
read the companion design spec; this document does not re-justify decisions, it
executes them. Where they conflict, this plan wins for v1 scope.

Module: a scanned-synthesis oscillator. A mass-spring ring evolves slowly
(haptic band); a phase pointer scans its shape at audio rate; the shape is the
waveform. Pitch = scan rate, decoupled from the dynamics.

---

## 0. Scope

**In, v1:** ring topology only; symplectic-Euler dynamics; scan readout with
linear interpolation; four excitation shapes; TRIG re-excite; EXT IN force
injection; FREEZE; OUT + MOTION outputs.

**Deferred to v2 (leave hooks, don't build):** BOUNDARY string mode; polyphony;
cubic scan interpolation; dynamics divider for CPU; switchable DC blocker as a
panel control (a fixed internal one is fine in v1).

**Non-goals:** presets, custom panel art (a placeholder SVG of correct width is
sufficient to build and test).

---

## 1. Files

Follow the standard one-module layout. Slug `Haptik` everywhere (plugin.json
module slug == `createModel("Haptik")` == `extern Model* modelHaptik`).

```
src/Haptik.cpp     # all module + widget code
plugin.hpp         # add: extern Model* modelHaptik;
plugin.cpp         # add: p->addModel(modelHaptik);
res/Haptik.svg     # placeholder panel, ~12 HP wide
plugin.json        # add the module entry, tags ["Oscillator","Physical Modeling"]
```

---

## 2. Module interface

`MAX_N = 256`. State arrays sized to `MAX_N`.

### Params (`enum ParamId`)

| Symbol | config range / default | snap | Notes |
| --- | --- | --- | --- |
| N_PARAM | 8 .. 128, default 64 | yes | masses; also table length |
| PITCH_PARAM | -4 .. 4 oct, default 0 | no | display Hz: base 2, mult `dsp::FREQ_C4` |
| RATE_PARAM | log Hz 0.05 .. 30, default 3 | no | evolution speed → k_center |
| COUPLE_PARAM | 0 .. 0.9, default 0.3 | no | neighbour stiffness k_spring (hard cap 0.9 for stability) |
| DAMP_PARAM | 0 .. 1, default 0.35 | no | 0 = lossless/drone |
| INJECT_PARAM | 0 .. 1, default 0.6 | no | excitation + EXT IN gain |
| EXCITE_PARAM | 0 .. 3, default 1 | yes | 0 impulse, 1 bump, 2 noise, 3 continuous drive |
| FREEZE_PARAM | switch 0/1, default 0 | — | configSwitch {"Run","Freeze"} |
| RATE_ATT, COUPLE_ATT, DAMP_ATT, INJECT_ATT | -1 .. 1, default 0 | no | attenuverters |

### Inputs (`enum InputId`)

| Symbol | Notes |
| --- | --- |
| VOCT_INPUT | 1V/oct, summed with PITCH_PARAM, no attenuverter |
| RATE_INPUT, COUPLE_INPUT, DAMP_INPUT, INJECT_INPUT | ±5V CV, depth `cv * att * 0.1` |
| TRIG_INPUT | SchmittTrigger; rising edge re-excites with current EXCITE shape |
| EXT_INPUT | audio-rate force injected at driver mass |

### Outputs (`enum OutputId`)

| Symbol | Notes |
| --- | --- |
| OUT_OUTPUT | scanned waveform, soft-clipped to ±5V |
| MOTION_OUTPUT | y[0] as sub-audio CV, ±5V |

### Persistent state (members)

```
float y[MAX_N] = {}, v[MAX_N] = {};   // displacement, velocity
float scanPhase = 0.f;
int   last_N = -1;                     // forces reinit on first process()
int   driverIdx = 0;
dsp::SchmittTrigger trig;
dsp::TRCFilter<float> dcBlock;         // fixed internal DC blocker on OUT
```

Tunable constants (declare as `static constexpr`, tune by ear at M7):
`DAMP_MAX_HZ = 800.f`, `OUT_GAIN = 1.0f`, `MOTION_GAIN = 4.f`,
`EXT_GAIN = 0.2f`, `BUMP_FRAC = 0.125f` (bump width = N·frac).

---

## 3. Per-sample math (the kernel)

All in normalised units: mass = 1, dt = 1 per sample. Recompute coefficients
from `args.sampleRate` every sample — this is what makes sample-rate changes
graceful with no `onSampleRateChange` handler needed (see §5).

```
// macros -> physics
fEvo   = RATE (log Hz, +CV·att·0.1, clamp 0.05..30)
wc     = 2π·fEvo / fs
kCtr   = wc·wc
kSpr   = clamp(COUPLE +CV·att·0.1, 0, 0.9)
gamma  = exp( -(DAMP +CV·att·0.1) · DAMP_MAX_HZ / fs )   // velocity multiplier, ≤1
amt    = clamp(INJECT +CV·att·0.1, 0, 1)
freeze = FREEZE > 0.5

// reinit on N change (also first call): zero y[],v[]; scanPhase=0;
// driverIdx = N/4; optionally seed one bump so it sounds on load (see §6 decision)

// excitation
if trig.process(TRIG voltage): applyExcite(EXCITE, amt)        // see §4
drive = EXT connected ? EXT·EXT_GAIN·amt : 0
if EXCITE==3 && !freeze: drive += amt · 0.05 · (2·rand-1)      // continuous keep-alive

// dynamics: symplectic Euler, two passes (skip if freeze)
if !freeze:
  for i in 0..N-1:
    lap  = y[(i-1+N)%N] - 2·y[i] + y[(i+1)%N]
    a    = kSpr·lap - kCtr·y[i] + (i==driverIdx ? drive : 0)
    v[i] = (v[i] + a) · gamma
  for i in 0..N-1:
    y[i] += v[i]

// scan readout (always)
scanPhase += pitchHz / fs;  scanPhase -= floor(scanPhase)
p   = scanPhase · N
i0  = min((int)p, N-1)                 // guard against p==N at phase rounding
f   = p - i0
s   = y[i0] + f·(y[(i0+1)%N] - y[i0])

// outputs
dcBlock.setCutoff(20.f / fs);  dcBlock.process(s)              // ~20 Hz HP
OUT    = 5·tanh(dcBlock.highpass() · OUT_GAIN)
MOTION = clamp(y[0]·MOTION_GAIN, -5, 5)
```

`pitchHz = dsp::FREQ_C4 · 2^(PITCH_PARAM + VOCT)`.

Two-pass ordering is required: pass 1 reads only `y[]` (unchanged) to form all
accelerations from one snapshot; pass 2 reads only `v[]`.

---

## 4. Excitation shapes (`applyExcite(shape, amt)`)

- **0 impulse:** `y[driverIdx] += amt` (single mass kicked).
- **1 bump:** add a Hann window of half-width `BUMP_FRAC·N` centred on
  `driverIdx`, peak `amt`, wrapping modulo N.
- **2 noise:** `for all i: y[i] += amt·(2·rand-1)`.
- **3 drive:** no impulse here; handled as continuous force in the kernel.

Excitation writes displacement `y[]` (a "pluck"). Optionally expose
strike-vs-pluck later by writing `v[]` instead; not in v1.

---

## 5. Correctness checklist (verify before done)

- **Stability is structural:** ω_max = sqrt(kCtr + 4·kSpr); with kSpr ≤ 0.9 and
  kCtr tiny, ω_max < 2, so symplectic Euler is stable. The COUPLE clamp at 0.9
  is the guarantee — do not raise it. Add a debug assert that `y[]` stays finite
  while sweeping COUPLE 0→0.9 at DAMP=0.
- **N change:** reinit path zeroes state and resets `driverIdx`; a click on N
  change is acceptable (matches GENDYN).
- **Index safety:** `i0 = min((int)(scanPhase·N), N-1)`; neighbour index
  `(i0+1)%N`; Laplacian uses `(i-1+N)%N`. No negative modulo.
- **Sample-rate change:** no cached SR-dependent state — `kCtr`, `gamma`,
  `pitchHz`, and the DC cutoff are all recomputed per sample from `fs`. So an
  engine SR change is graceful and needs no handler. (If you later cache any of
  these, you must add `onSampleRateChange`.)
- **State persistence:** params save automatically. Lattice state (`y`,`v`,
  `scanPhase`) is intentionally NOT persisted via `dataToJson` — a reloaded
  patch re-seeds; document this as deliberate.
- **No audio-thread allocation / locks / IO** in `process()`.
- **Denormals:** the DAMP-driven decay can drive `v[]` toward denormal in long
  undamped tails; the `+kCtr` restoring term and continuous drive usually keep
  it lively, but if profiling shows denormal stalls, add `+= 1e-20f` to the
  velocity update.
- **DC:** internal 20 Hz high-pass on OUT (the scanned mean wanders). MOTION is
  deliberately NOT high-passed — it's meant to be sub-audio.

---

## 6. Build milestones (each ends with a check)

**M1 — Skeleton loads.** Enums, all `config*`, empty `process` writing 0 V.
Module appears in Rack, no crash. ✔ loads and can be patched.

**M2 — Static scan.** Skip dynamics. On reinit, fill `y[]` with one sine period
across the ring. Implement scan readout + pitch. ✔ steady tone; pitch tracks
V/OCT (at PITCH=0,VOCT=0 the fundamental is ≈261.6 Hz — measure period =
fs/freq); scope shows a clean sine.

**M3 — Dynamics live.** Add the symplectic step with kCtr/kSpr, DAMP off, seed a
bump on reinit. ✔ the tone visibly/audibly morphs; sweeping COUPLE 0→0.9 never
produces NaN/inf or runaway level (assert finite).

**M4 — Damping + excitation + FREEZE.** Add gamma, the four EXCITE shapes, TRIG,
FREEZE. ✔ EXCITE=1 plucks and settles; DAMP=0 + EXCITE=2 drones; FREEZE holds
the current waveform as a static tone that still tracks pitch.

**M5 — EXT IN.** Inject external audio as force at the driver. ✔ patch a VCO into
EXT IN with DAMP mid → the ring resonates the input (formant/string-ish).

**M6 — MOTION + output stage.** y[0]→MOTION; tanh + DC blocker on OUT. ✔ MOTION
is a usable slow LFO; OUT has no DC offset on a low undamped patch; transients
don't clip hard.

**M7 — Review + tune.** Run §5 checklist. Tune `DAMP_MAX_HZ`, `OUT_GAIN`,
`MOTION_GAIN`, `EXT_GAIN`, `BUMP_FRAC` by ear against the starting points below.
✔ defaults sound good cold.

---

## 7. Default voicing / smoke-test patches

- **Plucked, settling:** EXCITE=1, DAMP≈0.35, COUPLE≈0.3, RATE≈3 Hz, trigger.
- **Drone:** DAMP=0, COUPLE≈0.6, EXCITE=2 (or hold TRIG).
- **Frozen wavetable:** excite, settle, FREEZE on, play V/OCT.
- **Driven resonator:** osc → EXT IN, DAMP≈0.4, modulate COUPLE.

These double as the M4/M5 acceptance patches.

---

## 8. Open decisions (flag to the human, don't guess)

1. **Sound on load:** seed a default bump at reinit (makes noise immediately,
   like a normal VCO) vs. silent until TRIG. Plan assumes *seed a bump*; confirm.
2. **Driver position:** `driverIdx = N/4` (asymmetric, excites more modes) vs.
   centre vs. exposed as a param. Plan assumes N/4.
3. **`MAX_N`/N ceiling:** 128 playable max, 256 array. Raise if CPU allows and
   smoother tables are wanted.
4. **DC blocker:** fixed internal (v1) vs. switchable panel control (v2). Plan
   assumes fixed internal at 20 Hz.
5. **Tunable constants** in §2 are first guesses; M7 sets them by ear.
