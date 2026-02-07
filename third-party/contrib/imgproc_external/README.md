# External Image Processing Algorithms

This directory contains core algorithm implementations extracted from research papers, preserved under their **original copyrights**. These are **NOT** MIT-licensed.

## Contents

### `rcmg_core.hpp` — Robust Colour Morphological Gradient

| Field | Value |
|---|---|
| **Author** | Adrian N. Evans, University of Bath |
| **Copyright** | © 2005-2007 Adrian N. Evans. All rights reserved. |
| **License** | No open-source license granted |
| **Reference** | A.N. Evans and X.U. Liu, "A Morphological Gradient Approach to Color Edge Detection", IEEE Trans. Image Processing, 15(6), pp. 1454-1463, 2006 |
| **Modifications** | Ported from C to C++17 templates, OpenCV integration, extracted as standalone core (Benjamin Wassermann, 2016-2026) |

### `susan_core.hpp` — SUSAN Edge Detector

| Field | Value |
|---|---|
| **Author** | Stephen Smith, FMRIB, Oxford University |
| **Copyright** | © Crown Copyright (1995-1999), Defence Evaluation and Research Agency |
| **License** | Research purposes only. Must not be sold. Header must be kept intact. |
| **Patent** | UK Patent 2272285 (lapsed) |
| **Reference** | Smith, S.M. and Brady, J.M., "SUSAN - A New Approach to Low Level Image Processing", Int. Journal of Computer Vision, 23(1), pp. 45-78, 1997 |
| **Modifications** | Ported from C to C++17 templates, OpenCV integration, extracted as standalone core (Benjamin Wassermann, 2016-2026) |

## Usage Restrictions

⚠️ **This code is NOT freely usable for commercial purposes:**

- **RCMG:** "All rights reserved" — no redistribution or commercial license granted. Contact Adrian N. Evans, University of Bath for licensing.
- **SUSAN:** Crown Copyright, issued for research purposes only. Must not be sold. The UK patent has lapsed, but copyright restrictions remain.

## Integration

These headers are used by `libs/imgproc` through thin wrapper classes:

- `imgproc/rcmg.hpp` → `RCMGradient` wraps `rcmg_impl::RCMGCore`
- `imgproc/susan.hpp` → `SusanGradient` wraps `susan_impl::SusanCore`

The wrapper classes (MIT-licensed) provide the `Gradient<>` interface, option management, and lazy direction computation. The core algorithm logic remains under its original copyright.

### Bazel

```python
deps = ["//third-party/contrib/imgproc_external"]
```

### CMake

```cmake
target_link_libraries(my_target contrib_imgproc_external)
```
