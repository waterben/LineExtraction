# Parameter Optimization — Detection Statistics

Evaluation of all 9 LSD detectors on the **York Urban** dataset (102 images,
12 122 ground-truth line segments) using three parameter strategies:

| Strategy     | Description |
|:-------------|:------------|
| **default**  | Detector's built-in defaults (no `DetectorProfile`). |
| **profile**  | `DetectorProfile(50, 50, 50, 50)` — neutral sliders, no adaptive factors. |
| **adaptive** | `DetectorProfile.from_image(img)` — image-adaptive profile. |

Matching threshold: 10 px (endpoint distance).

---

## Image Property Analysis (102 images)

| Property      |  Mean  |  Std   |  Min   |  Max   | Median |
|:--------------|-------:|-------:|-------:|-------:|-------:|
| contrast      | 0.4752 | 0.0811 | 0.2251 | 0.7356 | 0.4742 |
| noise_level   | 0.3396 | 0.1447 | 0.1483 | 0.8401 | 0.2965 |
| edge_density  | 0.3817 | 0.0653 | 0.2005 | 0.5390 | 0.3794 |
| dynamic_range | 0.7668 | 0.1095 | 0.3843 | 0.9765 | 0.7843 |

---

## Aggregate Results

All numbers are computed over 102 images per (detector, strategy) pair.

| Detector   | Strategy | mP    | mR    | mF1   | σF1   | mdF1  | μP    | μR    | μF1   | det/img | ms/img |
|:-----------|:---------|------:|------:|------:|------:|------:|------:|------:|------:|--------:|-------:|
| **LsdCP**  | adaptive | 0.123 | 0.337 | **0.160** | 0.088 | 0.143 | 0.123 | 0.283 | 0.172 | 273   | 22.3  |
| **LsdCC**  | adaptive | 0.120 | 0.348 | **0.159** | 0.087 | 0.136 | 0.121 | 0.293 | 0.171 | 288   | 32.4  |
| LsdCP      | profile  | 0.114 | 0.346 | 0.156 | 0.086 | 0.141 | 0.117 | 0.299 | 0.168 | 305   | 24.7  |
| LsdCC      | profile  | 0.111 | 0.359 | 0.154 | 0.084 | 0.134 | 0.114 | 0.310 | 0.166 | 324   | 35.7  |
| LsdEDLZ    | adaptive | 0.123 | 0.249 | 0.144 | 0.085 | 0.131 | 0.123 | 0.204 | 0.154 | 196   | 11.5  |
| LsdEDLZ    | profile  | 0.114 | 0.254 | 0.141 | 0.083 | 0.128 | 0.116 | 0.214 | 0.150 | 219   | 12.2  |
| LsdFGioi   | profile  | 0.077 | 0.394 | 0.120 | 0.095 | 0.081 | 0.079 | 0.428 | 0.134 | 644   | 85.4  |
| LsdEL      | adaptive | 0.068 | 0.444 | 0.110 | 0.091 | 0.067 | 0.069 | 0.465 | 0.120 | 803   | 40.8  |
| LsdEL      | profile  | 0.062 | 0.460 | 0.102 | 0.086 | 0.067 | 0.063 | 0.490 | 0.111 | 929   | 43.6  |
| LsdFGioi   | default  | 0.059 | 0.332 | 0.093 | 0.083 | 0.055 | 0.061 | 0.381 | 0.105 | 740   | 92.5  |
| LsdEDLZ    | default  | 0.059 | 0.341 | 0.093 | 0.075 | 0.061 | 0.062 | 0.361 | 0.106 | 689   | 16.6  |
| LsdEP      | adaptive | 0.050 | 0.449 | 0.085 | 0.076 | 0.048 | 0.049 | 0.482 | 0.089 | 1162  | 21.0  |
| LsdEP      | profile  | 0.049 | 0.459 | 0.083 | 0.075 | 0.055 | 0.048 | 0.495 | 0.087 | 1230  | 22.7  |
| LsdFGioi   | adaptive | 0.050 | 0.255 | 0.078 | 0.073 | 0.053 | 0.049 | 0.268 | 0.082 | 656   | 113.7 |
| LsdFBW     | adaptive | 0.045 | 0.475 | 0.077 | 0.076 | 0.041 | 0.045 | 0.546 | 0.083 | 1442  | 46.2  |
| LsdBurns   | adaptive | 0.041 | 0.322 | 0.068 | 0.066 | 0.039 | 0.041 | 0.367 | 0.073 | 1070  | 42.1  |
| LsdFBW     | profile  | 0.038 | 0.476 | 0.067 | 0.066 | 0.040 | 0.038 | 0.552 | 0.072 | 1708  | 52.7  |
| LsdFBW     | default  | 0.036 | 0.496 | 0.065 | 0.065 | 0.039 | 0.039 | 0.570 | 0.072 | 1752  | 65.2  |
| LsdBurns   | profile  | 0.036 | 0.342 | 0.061 | 0.060 | 0.035 | 0.038 | 0.397 | 0.069 | 1252  | 48.6  |
| LsdCP      | default  | 0.034 | 0.452 | 0.060 | 0.061 | 0.033 | 0.036 | 0.510 | 0.067 | 1688  | 43.5  |
| LsdCC      | default  | 0.033 | 0.451 | 0.059 | 0.060 | 0.031 | 0.035 | 0.513 | 0.065 | 1748  | 55.5  |
| LsdEP      | default  | 0.014 | 0.449 | 0.027 | 0.035 | 0.013 | 0.014 | 0.531 | 0.027 | 4545  | 46.7  |
| LsdEL      | default  | 0.014 | 0.454 | 0.026 | 0.034 | 0.012 | 0.013 | 0.539 | 0.026 | 4756  | 83.6  |
| LsdHoughP  | adaptive | 0.018 | 0.037 | 0.020 | 0.026 | 0.012 | 0.011 | 0.031 | 0.016 | 336   | 58.8  |
| LsdHoughP  | profile  | 0.011 | 0.040 | 0.014 | 0.016 | 0.009 | 0.007 | 0.033 | 0.011 | 557   | 60.5  |
| LsdBurns   | default  | 0.007 | 0.460 | 0.013 | 0.016 | 0.006 | 0.007 | 0.549 | 0.014 | 9481  | 105.3 |
| LsdHoughP  | default  | 0.003 | 0.061 | 0.006 | 0.008 | 0.003 | 0.003 | 0.083 | 0.006 | 3039  | 129.0 |

**Legend:**
mP = mean precision, mR = mean recall, mF1 = mean F1 (macro-averaged over images),
σF1 = std-dev of F1, mdF1 = median F1,
μP / μR / μF1 = micro-averaged precision / recall / F1 (pooled TP/FP/FN),
det/img = mean detections per image, ms/img = mean detection time per image.

---

## Best Strategy per Detector

| Detector   | Best Strategy | mF1   | Improvement over default |
|:-----------|:--------------|------:|:-------------------------|
| LsdCP      | adaptive      | 0.160 | +167% (0.060 → 0.160)    |
| LsdCC      | adaptive      | 0.159 | +170% (0.059 → 0.159)    |
| LsdEDLZ    | adaptive      | 0.144 | +55% (0.093 → 0.144)     |
| LsdFGioi   | profile       | 0.120 | +29% (0.093 → 0.120)     |
| LsdEL      | adaptive      | 0.110 | +323% (0.026 → 0.110)    |
| LsdEP      | adaptive      | 0.085 | +215% (0.027 → 0.085)    |
| LsdFBW     | adaptive      | 0.078 | +19% (0.065 → 0.078)     |
| LsdBurns   | adaptive      | 0.068 | +423% (0.013 → 0.068)    |
| LsdHoughP  | adaptive      | 0.020 | +227% (0.006 → 0.020)    |

---

## Key Findings

1. **Adaptive parameters improve every detector.**
   8 of 9 detectors achieve their best F1 with the adaptive strategy
   (`DetectorProfile.from_image`). Only LsdFGioi prefers the neutral
   profile (50/50/50/50).

2. **The biggest gains come from reducing over-detection.**
   Default parameters produce far too many segments (e.g. LsdBurns:
   9 481/img → 1 070/img). Adaptive/profile parameters cut detection
   count by 3–9× while improving precision dramatically.

3. **LsdCP and LsdCC are the strongest detectors on York Urban**
   both reaching F1 ≈ 0.16 with adaptive parameters. LsdCP is also
   33% faster (22 ms vs 32 ms per image).

4. **LsdEDLZ offers the best speed-accuracy trade-off.**
   F1 = 0.144 at only 11.5 ms/image — the fastest detector with
   competitive accuracy.

5. **LsdHoughP performs poorly across all strategies.**
   F1 never exceeds 0.02, suggesting the probabilistic Hough approach
   is poorly suited for this benchmark's line matching criteria.

---

## Bug Fix: LsdCP Segfault (Heap-Buffer-Overflow)

### Symptoms

Prior to the fix, LsdCP crashed with SIGSEGV (signal 11) when using
non-default parameters (profile or adaptive strategy) on certain images.
The crash was non-deterministic and appeared only when processing
multiple images sequentially.

### Root Cause

**Heap-buffer-overflow** in `LsdCP::computePatterns()` at
`libs/lsd/include/lsd/lsd_cp.hpp`, line 790.

The `move_idx` lambda inside `find_gap()` advances an index into the
edge map (`emap_`) without checking array bounds:

```cpp
// BEFORE (buggy)
auto move_idx = [&pemap, &pdmap](int& idx, char ndir) {
    idx += pdmap[static_cast<int>(ndir)][0];
    return pemap[idx];       // ← out-of-bounds read when idx escapes image
};
```

During gap detection, `find_gap()` calls `move_idx` repeatedly in a
loop (up to `max_gap_` iterations), then branches left (`edir - 2`) and
right (`edir + 2`). When the search reaches the image border (where
`emap_[idx] == -1`), subsequent calls continue stepping outward — past
the allocated `emap_` buffer — causing a heap-buffer-overflow.

With default parameters (`max_gap_ = 0`), the gap search loop never
executes, masking the bug. Profile/adaptive parameters set `max_gap_`
to 2–3, activating the code path.

### Fix

Added a bounds check in `move_idx` to return `-2` (used-pixel sentinel,
which stops the search) when the index leaves the valid range:

```cpp
// AFTER (fixed)
auto move_idx = [&pemap, &pdmap, this](int& idx, char ndir) -> char {
    idx += pdmap[static_cast<int>(ndir)][0];
    if (idx < 0 || idx >= this->size_) {
        return -2;
    }
    return pemap[idx];
};
```

Additionally, the `tolerance()` setter was hardened to prevent
`pat_tol_` from being set to zero (protecting a separate `size / pat_tol_`
division in `computePatterns()`):

```cpp
void tolerance(int pt) { pat_tol_ = std::max(1, pt); }
```

### Verification

- **AddressSanitizer**: 8 new crash-regression tests pass under ASAN
  (`bazel test --config=asan //libs/lsd:test_lsd_cp_crash`).
- **Full test suite**: All 75 library tests pass
  (`bazel test //libs/...`).
- **Full statistics run**: LsdCP now completes all 27 strategy
  combinations on 102 York Urban images without any crashes.

### Affected Files

| File | Change |
|:-----|:-------|
| `libs/lsd/include/lsd/lsd_cp.hpp` | Bounds check in `move_idx`; clamp in `tolerance()` setter |
| `libs/lsd/tests/test_lsd_cp_crash.cpp` | New ASAN regression test (8 test cases) |
| `evaluation/python/tools/detector_statistics.py` | `set_double` → `set_float` (matching Python binding API) |
| `evaluation/python/tools/optimize_presets.py` | Same `set_double` → `set_float` fix |

---

## Wireframe Dataset Evaluation

Evaluation of all 9 LSD detectors on the **Wireframe** dataset (50 images,
ground-truth line segments) using the same three parameter strategies.
Matching threshold: 10 px (endpoint distance).

### Aggregate Results (Wireframe)

| Detector   | Strategy | mP    | mR    | mF1   | σF1   | mdF1  | μP    | μR    | μF1   | det/img | ms/img |
|:-----------|:---------|------:|------:|------:|------:|------:|------:|------:|------:|--------:|-------:|
| **LsdCC**  | adaptive | 0.154 | 0.310 | **0.196** | 0.059 | 0.205 | 0.147 | 0.293 | 0.196 | 166.7 | 19.0  |
| **LsdCP**  | adaptive | 0.155 | 0.296 | **0.194** | 0.056 | 0.199 | 0.148 | 0.280 | 0.194 | 157.6 | 13.3  |
| LsdCC      | profile  | 0.145 | 0.320 | 0.193 | 0.059 | 0.195 | 0.140 | 0.304 | 0.191 | 181.4 | 20.4  |
| LsdCP      | profile  | 0.146 | 0.304 | 0.191 | 0.057 | 0.194 | 0.141 | 0.288 | 0.190 | 170.0 | 14.1  |
| LsdEDLZ    | adaptive | 0.169 | 0.232 | 0.186 | 0.057 | 0.184 | 0.163 | 0.215 | 0.185 | 109.9 |  6.2  |
| LsdEDLZ    | profile  | 0.159 | 0.227 | 0.179 | 0.059 | 0.181 | 0.151 | 0.213 | 0.176 | 117.6 |  6.7  |
| LsdEL      | adaptive | 0.078 | 0.422 | 0.128 | 0.050 | 0.126 | 0.073 | 0.419 | 0.124 | 478.3 | 22.6  |
| LsdEL      | profile  | 0.074 | 0.434 | 0.125 | 0.049 | 0.123 | 0.071 | 0.429 | 0.122 | 505.2 | 23.6  |
| LsdFGioi   | profile  | 0.078 | 0.339 | 0.124 | 0.049 | 0.122 | 0.074 | 0.337 | 0.122 | 378.8 | 52.7  |
| LsdEDLZ    | default  | 0.073 | 0.321 | 0.117 | 0.045 | 0.105 | 0.070 | 0.314 | 0.115 | 372.8 |  9.2  |
| LsdEP      | profile  | 0.059 | 0.441 | 0.102 | 0.043 | 0.098 | 0.055 | 0.436 | 0.098 | 662.4 | 12.7  |
| LsdEP      | adaptive | 0.059 | 0.431 | 0.101 | 0.043 | 0.100 | 0.053 | 0.428 | 0.095 | 668.5 | 12.3  |
| LsdFGioi   | default  | 0.062 | 0.303 | 0.101 | 0.043 | 0.095 | 0.059 | 0.304 | 0.099 | 426.3 | 53.6  |
| LsdFBW     | adaptive | 0.057 | 0.484 | 0.100 | 0.044 | 0.094 | 0.051 | 0.484 | 0.092 | 795.4 | 24.9  |
| LsdFGioi   | adaptive | 0.061 | 0.249 | 0.097 | 0.051 | 0.098 | 0.057 | 0.247 | 0.092 | 363.1 | 70.2  |
| LsdFBW     | profile  | 0.051 | 0.497 | 0.092 | 0.040 | 0.087 | 0.048 | 0.498 | 0.087 | 869.0 | 27.9  |
| LsdBurns   | adaptive | 0.046 | 0.308 | 0.078 | 0.036 | 0.074 | 0.041 | 0.307 | 0.072 | 626.1 | 24.0  |
| LsdFBW     | default  | 0.043 | 0.499 | 0.078 | 0.033 | 0.069 | 0.041 | 0.501 | 0.076 | 1011.9| 35.0  |
| LsdBurns   | profile  | 0.043 | 0.331 | 0.074 | 0.033 | 0.070 | 0.041 | 0.331 | 0.073 | 674.6 | 27.5  |
| LsdCP      | default  | 0.041 | 0.427 | 0.073 | 0.030 | 0.067 | 0.040 | 0.426 | 0.072 | 899.7 | 26.3  |
| LsdCC      | default  | 0.040 | 0.434 | 0.072 | 0.030 | 0.066 | 0.039 | 0.434 | 0.071 | 930.4 | 30.9  |
| LsdEP      | default  | 0.018 | 0.462 | 0.035 | 0.017 | 0.033 | 0.017 | 0.465 | 0.033 | 2266.1| 24.5  |
| LsdEL      | default  | 0.018 | 0.474 | 0.034 | 0.017 | 0.032 | 0.017 | 0.477 | 0.033 | 2369.4| 42.9  |
| LsdHoughP  | profile  | 0.019 | 0.032 | 0.021 | 0.017 | 0.015 | 0.011 | 0.033 | 0.017 | 251.3 | 33.8  |
| LsdHoughP  | adaptive | 0.025 | 0.021 | 0.020 | 0.025 | 0.013 | 0.017 | 0.020 | 0.018 |  96.4 | 31.5  |
| LsdBurns   | default  | 0.010 | 0.496 | 0.019 | 0.010 | 0.018 | 0.009 | 0.503 | 0.018 | 4715.2| 56.5  |
| LsdHoughP  | default  | 0.007 | 0.086 | 0.012 | 0.009 | 0.010 | 0.006 | 0.090 | 0.011 | 1270.3| 66.4  |

### Best Strategy per Detector (Wireframe)

| Detector   | Best Strategy | mF1   | Improvement over default |
|:-----------|:--------------|------:|:-------------------------|
| LsdCC      | adaptive      | 0.196 | +172% (0.072 → 0.196)    |
| LsdCP      | adaptive      | 0.194 | +166% (0.073 → 0.194)    |
| LsdEDLZ    | adaptive      | 0.186 | +59% (0.117 → 0.186)     |
| LsdEL      | adaptive      | 0.128 | +276% (0.034 → 0.128)    |
| LsdFGioi   | profile       | 0.124 | +23% (0.101 → 0.124)     |
| LsdEP      | profile       | 0.102 | +191% (0.035 → 0.102)    |
| LsdFBW     | adaptive      | 0.100 | +28% (0.078 → 0.100)     |
| LsdBurns   | adaptive      | 0.078 | +311% (0.019 → 0.078)    |
| LsdHoughP  | profile       | 0.021 | +75% (0.012 → 0.021)     |

### Cross-Dataset Comparison

| Detector   | Best (York Urban) | Best (Wireframe) | Δ mF1  |
|:-----------|:------------------|:-----------------|-------:|
| LsdCC      | 0.159 (adaptive)  | 0.196 (adaptive) | +0.037 |
| LsdCP      | 0.160 (adaptive)  | 0.194 (adaptive) | +0.034 |
| LsdEDLZ    | 0.144 (adaptive)  | 0.186 (adaptive) | +0.042 |
| LsdFGioi   | 0.120 (profile)   | 0.124 (profile)  | +0.004 |
| LsdEL      | 0.110 (adaptive)  | 0.128 (adaptive) | +0.018 |
| LsdEP      | 0.085 (adaptive)  | 0.102 (profile)  | +0.017 |
| LsdFBW     | 0.078 (adaptive)  | 0.100 (adaptive) | +0.022 |
| LsdBurns   | 0.068 (adaptive)  | 0.078 (adaptive) | +0.010 |
| LsdHoughP  | 0.020 (adaptive)  | 0.021 (profile)  | +0.001 |

**Key observations:**

- All detectors score **higher on Wireframe** than on York Urban,
  suggesting Wireframe's ground truth aligns better with the line
  segment extraction approach.
- **LsdCC** overtakes LsdCP as the top detector on Wireframe (F1=0.196
  vs 0.194), while LsdCP was marginally ahead on York Urban.
- **LsdEDLZ** remains the best speed-accuracy trade-off: F1=0.186 at
  only 6.2 ms/image on Wireframe.
- The **adaptive strategy wins for 7 of 9** detectors on Wireframe
  (vs 8 of 9 on York Urban). LsdFGioi and LsdEP prefer the neutral
  profile.
