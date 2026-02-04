# Third-Party Libraries

This directory contains third-party libraries and external code used by the LineExtraction project.

⚠️ **License Notice:** Code in this directory is subject to **different licenses** than the main project's MIT license.

## Contents

### [`contrib/`](contrib/README.md)

**Algorithm Implementations - Academic / AGPL / BSD Licenses**

Contains third-party algorithm implementations and auto-generated code:

- **`matlab_coder/`** - MATLAB Coder generated code (Academic License)
- **`lsd_external/`** - External LSD implementations (AGPL v3, BSD, Academic)

[📄 See contrib/README.md for details](contrib/README.md)

---

### [`qplot/`](qplot/README.md)

**QCustomPlot - GPL v3**

Interactive plotting widget for Qt applications.

- **License:** GNU General Public License v3.0
- **Author:** Emanuel Eichhammer
- **Version:** 1.3.0
- **Website:** <http://www.qcustomplot.com/>

**Used by:**

- `apps/line_analyzer/` - Line Analyzer GUI

⚠️ **GPL v3 Implications:**

- Applications using this must be GPL v3
- Commercial license available from author
- Strong copyleft - derivatives must be open-source

[📄 See qplot/README.md for details](qplot/README.md)

---

### [`qplot3d/`](qplot3d/README.md)

**QCustomPlot3D - GPL v3**

3D visualization extension for QCustomPlot.

- **License:** GNU General Public License v3.0
- **Based on:** QCustomPlot by Emanuel Eichhammer

**Used by:**

- `apps/line_analyzer/` - 3D visualizations (if enabled)

⚠️ **Same GPL v3 restrictions as QCustomPlot apply.**

[📄 See qplot3d/README.md for details](qplot3d/README.md)

## License Summary

| Library | License | Commercial Use | Source Disclosure | Copyleft |
|---------|---------|----------------|-------------------|----------|
| `contrib/matlab_coder` | Academic (MATLAB) | ❌ No | - | - |
| `contrib/lsd_external/lsd_fgioi` | **AGPL v3** | ⚠️ Complex | ✅ Yes | ✅ Strong |
| `contrib/lsd_external/edlines` | BSD-style | ✅ Yes | ❌ No | ❌ No |
| `contrib/lsd_external/kht` | Academic | ⚠️ Check | ❌ No | ❌ No |
| `qplot` | **GPL v3** | ⚠️ No* | ✅ Yes | ✅ Strong |
| `qplot3d` | **GPL v3** | ⚠️ No* | ✅ Yes | ✅ Strong |

*Commercial licenses available from original authors

## Impact on LineExtraction

### Core Libraries (MIT Licensed)

The main LineExtraction libraries in `libs/` remain **MIT licensed**:

- `libs/edge/`
- `libs/geometry/`
- `libs/imgproc/` (except MATLAB Coder parts)
- `libs/lsd/` (except external implementations)
- `libs/lfd/`
- `libs/utility/`
- `libs/eval/`

### Components with Restrictive Licenses

**GPL v3 Components:**

- QCustomPlot / QCustomPlot3D
- **Any application using these (e.g., Line Analyzer) becomes GPL v3**

**AGPL v3 Component:**

- LSD by Grompone von Gioi
- Requires source disclosure for network/service use

**Academic/Proprietary Restrictions:**

- MATLAB Coder generated code
- Some LSD external implementations

## For Commercial / Proprietary Use

### Option 1: Avoid Restrictive Components

Use only MIT-licensed core libraries:

```bash
# Build without Qt applications
bazel build //libs/...

# Build without contrib components
# (use alternative implementations in libs/)
```

### Option 2: Obtain Commercial Licenses

- **QCustomPlot:** <http://www.qcustomplot.com/> (commercial license available)
- **Qt5:** <https://www.qt.io/licensing/> (commercial license available)
- **MATLAB Coder:** Contact MathWorks for commercial terms
- **LSD AGPL:** Contact Grompone von Gioi for alternative licensing

### Option 3: GPL Compliance

Release your application under GPL v3 and comply with:

- Source code disclosure
- Same GPL license for derivatives
- Patent and anti-tivoization clauses

## Qt5 Framework Licensing

Many components require **Qt5**, which has multiple licensing options:

- **LGPL v3** (most common for open-source):
  - Allows dynamic linking in proprietary apps
  - Must provide Qt source or means to relink
  - Suitable for most open-source projects

- **GPL v2 / GPL v3**:
  - For GPL-licensed applications
  - Source code disclosure required

- **Commercial Qt License**:
  - For proprietary closed-source applications
  - Removes LGPL obligations
  - Purchase from Qt Company

**Note:** If using QCustomPlot (GPL v3), entire app is GPL v3 regardless of Qt license choice.

## Build System Integration

### Auto-Detection

Both Bazel and CMake automatically detect Qt5 availability:

```bash
# Bazel: detect features including Qt5
./tools/scripts/detect_bazel_features.sh

# CMake: auto-detects during configuration
cmake ..
```

### Selective Building

```bash
# Bazel: build only MIT-licensed components
bazel build //libs/...

# CMake: disable Qt applications
cmake -DBUILD_APPS=OFF ..
```

## References

- **Main Project License:** [../../README.md#license](../../README.md#license)
- **Contrib Libraries:** [contrib/README.md](contrib/README.md)
- **QCustomPlot:** [qplot/README.md](qplot/README.md)
- **GPL v3 License:** <https://www.gnu.org/licenses/gpl-3.0.html>
- **LGPL v3 License:** <https://www.gnu.org/licenses/lgpl-3.0.html>
- **Qt Licensing:** <https://www.qt.io/licensing/>

---

**Questions about licensing?** See individual README files in subdirectories or consult the original authors.
