# External LSD Implementations

## Overview

This library contains external implementations of Line Segment Detection (LSD) algorithms from various research sources. These implementations use different licenses and are kept separate from the main MIT-licensed codebase.

## License Information

⚠️ **Mixed Licenses** - This directory contains code under multiple licenses:

### AGPL v3.0 - LSD by Grompone von Gioi

- **Files:** `lsd_fgioi_impl.cpp`, `lsd_fgioi.hpp`
- **License:** GNU Affero General Public License v3.0
- **Implications:** AGPL is a strong copyleft license requiring source disclosure for network use

### BSD-style - EDLines Zero

- **Files:** `lsd_edlz_impl.cpp`, `lsd_edlz.hpp`
- **License:** BSD-style with academic citation requirement

### Academic License - KHT

- **Files:** `kht_impl.cpp`, `kht.h`
- **License:** Academic use with citation requirement

## Implementations

### 1. LSD - Line Segment Detector (Grompone von Gioi)

**Original Author:** Rafael Grompone von Gioi, Jérémie Jakubowicz, Jean-Michel Morel, Gregory Randall

**Reference:**

```
"LSD: A Fast Line Segment Detector with a False Detection Control"
IEEE Transactions on Pattern Analysis and Machine Intelligence
vol. 32, no. 4, pp. 722-732, April 2010
```

**License:** AGPL v3.0

**Implementation:**

- `lsd_fgioi_impl.cpp` - Core LSD algorithm
- `lsd_fgioi.hpp` - Interface wrapper

**Key Features:**

- Fast line segment detection with false detection control
- No parameter tuning required
- Sub-pixel accuracy
- Based on gradient analysis

**Citation Required:**

```bibtex
@article{lsd,
  title={LSD: A Fast Line Segment Detector with a False Detection Control},
  author={Grompone von Gioi, R. and Jakubowicz, J. and Morel, J.-M. and Randall, G.},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence},
  volume={32},
  number={4},
  pages={722--732},
  year={2010}
}
```

### 2. EDLines Zero

**Original Authors:** Cuneyt Akinlar, Cihan Topal

**Reference:**

```
"EDLines: A real-time line segment detector with a false detection control"
Pattern Recognition Letters, vol. 32, no. 13, pp. 1633-1642, 2011
```

**License:** BSD-style with citation requirement

**Implementation:**

- `lsd_edlz_impl.cpp` - EDLines Zero algorithm
- `lsd_edlz.hpp` - Interface wrapper

**Key Features:**

- Real-time performance
- Edge Drawing based approach
- Parameter-free operation
- Robust to noise

**Citation Required:**

```bibtex
@article{edlines,
  title={EDLines: A real-time line segment detector with a false detection control},
  author={Akinlar, C. and Topal, C.},
  journal={Pattern Recognition Letters},
  volume={32},
  number={13},
  pages={1633--1642},
  year={2011}
}
```

### 3. KHT - Kernel-based Hough Transform

**Original Authors:** Leandro A. F. Fernandes, Manuel M. Oliveira

**Reference:**

```
"Real-time line detection through an improved Hough transform voting scheme"
Pattern Recognition, vol. 41, no. 1, pp. 299-314, 2008
```

**License:** Academic use with citation requirement

**Implementation:**

- `kht_impl.cpp` - KHT algorithm
- `kht.h` - Interface definitions

**Key Features:**

- Improved voting scheme
- Real-time performance
- Memory efficient
- Cluster-based detection

**Citation Required:**

```bibtex
@article{kht,
  title={Real-time line detection through an improved Hough transform voting scheme},
  author={Fernandes, L. A. F. and Oliveira, M. M.},
  journal={Pattern Recognition},
  volume={41},
  number={1},
  pages={299--314},
  year={2008}
}
```

## Build Configuration

The library suppresses warnings common in research code:

- Old-style casts
- Type conversions
- Sign comparisons
- Effective C++ guideline violations

This is intentional to avoid modifying original algorithms.

## Integration

This library is used by:

- `libs/lsd` - Line Segment Detection library (algorithm implementations)

## Usage Notes

⚠️ **Important Legal Considerations:**

1. **AGPL License (lsd_fgioi):**
   - Requires source disclosure if used in network services
   - Strong copyleft - derived works must be AGPL
   - May not be suitable for proprietary/commercial use without separate licensing

2. **Academic Licenses (EDLines, KHT):**
   - Require citation in published research
   - Check original license files for commercial use restrictions

3. **Not Part of Main MIT License:**
   - These implementations are **NOT** covered by LineExtraction's MIT license
   - Each has its own licensing terms
   - Consult original authors for commercial licensing

## Commercial Use

For commercial applications:

1. **lsd_fgioi (AGPL):** Contact original authors for commercial license or avoid using
2. **EDLines/KHT:** Review original license terms, may require separate agreement

Consider using alternative implementations from `libs/lsd` that use MIT-compatible licenses.

## Original Source Repositories

- **LSD:** <http://www.ipol.im/pub/art/2012/gjmr-lsd/>
- **EDLines:** <http://ceng.anadolu.edu.tr/cv/EDLines/>
- **KHT:** <https://www.inf.ufrgs.br/~oliveira/pubs_files/RT_line_detection.html>

## Modifications

All implementations have been:

1. Wrapped in C++ classes for integration with LineExtraction
2. Adapted to use OpenCV data structures
3. Modified to fit project coding style (minimal changes to algorithms)

Original algorithm logic remains unchanged to preserve published results.

## Testing

These implementations are tested via:

- `libs/lsd/tests/test_lsd_implementations.cpp` - Comparative tests
- Validated against reference implementations
- Performance benchmarks in `evaluation/`

## Maintenance

**Do not modify algorithm implementations** without consulting original papers and authors. Changes may invalidate published results and benchmarks.

For bug fixes or improvements, consider:

1. Contacting original authors
2. Contributing back to original repositories
3. Documenting deviations from reference

---

**Note:** This library contains third-party code subject to different licensing terms than the main LineExtraction project. Ensure compliance with all applicable licenses before use, especially for commercial applications.
