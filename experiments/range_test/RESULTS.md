# PFM-1 range / false-positive experiment results

Harness: `experiments/range_test/run_range_experiment.py`
Validation: `experiments/range_test/validate_fixes.py` (whitelist + current detector)
Photos: copied from Downloads → `experiments/range_test/images/` (originals untouched)
Branch: `erim/perception`

## Detectors

| Path | What |
|------|------|
| **Stock shape** | `ShapeMineDetector` as shipped — chromatic-blue proposal, accepted on `matchShapes` + silhouette IoU |
| **Diag shape** | Harness-only baseline: Canny + `matchShapes`, no area-ratio gate |
| **Blue shape** | Harness-only baseline: HSV blue mask → filled contour → `matchShapes` |
| **Tag** | Pipeline **AprilTag `tag36h11` only**, mine IDs gated to `{0, 12}` |
| **ArUco** | OpenCV probe (diagnostic for fake paper codes) |

**Calibration:** no phone intrinsics in repo. Synthetic ~65° HFOV pinhole. **Absolute ranging is uncalibrated.**

---

## Current results (after the shape-detector rework)

### A) Detection vs distance (Part A)

| Dist | Stock shape | AprilTag (with_tag) |
|------|-------------|---------------------|
| 0.30 m | **2/2** | 0/1 |
| 0.50 m | **2/2** | 0/1 |
| 1.00 m | **2/2** | 0/1 |
| 1.50 m | **2/2** | 0/1 |
| 2.00 m | **2/2** | 0/1 |

Stock confidence on hits: **0.72 – 0.94**.

### B) Angles (Part B)

All four scenes hit (45°, low angle, in-plane rotation, rotated mine), confidence **0.86 – 0.91**.
B1 (45° tilt) merges into one large blob — detection fires but localization is weaker.

### C) False positives (Part C)

| Set | Stock | Notes |
|-----|-------|-------|
| C1 clutter+mine | 3/3 detect | Mine found; extra blue clutter blobs also survive as lower-ranked candidates |
| C2 mine+fake code | 3/3 detect | Fake code itself not registered as a mine |
| C3 **fake code only** | **0/2 — no detections** | Hard requirement met |

### D) Ranging

Mean **est/true ≈ 2.15**, mean error **+0.71 m**, ratios ~0.7–4.8.
**Not usable** until phone `fx` is calibrated. `pfm_physical_span_m=0.120` remains unvalidated.

---

## Key findings

1. **Gray-only Canny + `matchShapes` does not work on these photos.** Real edges fragment; bridging
   morphology floods the frame into a single contour. The original area-ratio gate (template
   ~90k px vs edge fragments ~1–3k px) rejected everything, but removing it alone is not enough.
2. **The old "diag (no ratio)" numbers were contaminated.** Many of its top scores were full-frame
   border contours (`span_px ≈ 1280`), not the mine — that overstated how recoverable the edge-only
   shape signal was.
3. **Working path:** chromatic-blue **proposal** (B-dominant + narrow hue/sat, which rejects green
   grass far better than naive HSV) → filled contours → **accept and rank on `matchShapes` +
   silhouette IoU**. Color never registers a mine on its own; that is what previously broke C3.
4. **AprilTag decode: still zero** on every `with_tag` photo. 1-inch tags at these distances with
   phone-camera blur are below the reliable decode threshold. Family is confirmed `tag36h11` —
   this is a size/resolution limit, not a wrong-family problem.

## Honest limitations

- Detection depends on the mine being **saturated blue plastic**. If the arena mines differ in
  color, the proposal stage must be re-tuned or replaced.
- Classical Hu-moment matching stays brittle under strong perspective and clutter. On a few frames
  (A4, A5) a second blue blob can outrank the mine, though the mine is still returned in `n ≥ 2`.
- **No photos beyond 2 m** — the high-altitude coarse-search ceiling is uncharacterized.
- No phone camera calibration in the repo, so all depth estimates are relative.

## Next data to collect

1. Photos at **3 m, 5 m, and survey altitude** to find the real detection ceiling.
2. **Phone/Pi camera calibration** so span-based ranging can be validated.
3. Larger tags, or accept that tag decode only works at close range and let shape carry wide-area search.
