# Haptik

A scanned-synthesis oscillator for VCV Rack 2.

A ring of masses connected by springs is set ringing, and a phase pointer scans
the ring's shape at audio rate — that shape *is* the waveform. The scan rate
(pitch) is independent of the spring dynamics, so you can **freeze** the shape
and play it as a wavetable, drive it externally so it resonates like a string,
or let it ring and evolve while it sounds. The coupled ring itself runs at audio
rate, so the voice sits between a wavetable oscillator and a Karplus–Strong-style
resonator.

## Controls

| Control | Range | Purpose |
| --- | --- | --- |
| **N** | 8–128 | number of masses / table length |
| **PITCH** | ±4 oct | scan rate (audio pitch); base C4 |
| **RATE** | 0.05–30 Hz | centering force — the ring's global "breathing" rate (subtle relative to COUPLE) |
| **COUPLE** | 0–0.9 | neighbour stiffness; the main timbral control (hard-capped at 0.9 for stability) |
| **DAMP** | 0–1 | energy loss; 0 = lossless drone |
| **INJECT** | 0–1 | excitation amount + EXT IN gain |
| **EXCITE** | impulse / bump / noise / drive | excitation shape applied on TRIG |
| **DRIVER** | 0–100% | where excitation / EXT IN enters the ring (default 25%) |
| **FREEZE** | Run / Freeze | hold the current waveform as a static tone |

CV inputs (RATE, COUPLE, DAMP, INJECT) each have an attenuverter. **V/OCT** sums
with PITCH. **TRIG** re-excites with the current shape. **EXT IN** injects
external audio as a force at the driver mass (its level scales with INJECT).

Outputs: **OUT** (scanned waveform, soft-clipped to ±5 V, internally DC-blocked
at ~20 Hz) and **MOTION** (the driver-region mass displacement as a ±5 V CV;
audio-rate in this build, deliberately not high-passed).

## Patches

`tools/make_patches.py` writes four smoke-test patches into `patches/` (and
copies them into the Windows Rack patches folder if present):

- **haptik_1_plucked** — plucked and settling
- **haptik_2_drone** — lossless drone (DAMP = 0)
- **haptik_3_frozen** — frozen wavetable; play it with V/OCT
- **haptik_4_driven** — VCO → EXT IN, the ring as a resonator

## Notes

- Lattice state (displacement / velocity / scan phase) is intentionally **not**
  saved with the patch — a reloaded patch re-seeds with a bump. Params persist.
- Stability is structural: with COUPLE ≤ 0.9 the symplectic-Euler step cannot
  blow up. **Do not raise that cap.** `tools/stability_test.cpp` verifies the
  no-blow-up invariant and pitch accuracy offline:
  `g++ -O2 -o /tmp/t tools/stability_test.cpp && /tmp/t` (exit 0 = pass).

## Build

```bash
# Linux
make RACK_DIR=~/Rack2-SDK/Rack-SDK dist

# Windows cross-compile (from WSL)
RACK_DIR=~/Rack2-SDK-win/Rack-SDK \
  CC=x86_64-w64-mingw32-gcc-posix CXX=x86_64-w64-mingw32-g++-posix \
  STRIP=x86_64-w64-mingw32-strip MACHINE=x86_64-w64-mingw32 make dist
```
