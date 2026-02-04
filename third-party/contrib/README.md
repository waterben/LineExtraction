# Third-Party Contributions

This directory contains third-party code and auto-generated implementations that are **NOT** part of the main MIT-licensed LineExtraction codebase.

⚠️ **Important:** Code in this directory is subject to **different licenses** than the main project. Review individual subdirectory READMEs for licensing details before use, especially for commercial applications.

## Contents

### [`matlab_coder/`](matlab_coder/README.md)

**MATLAB Coder Generated Code - Academic License**

Auto-generated C++ code from MATLAB implementations of:

- Phase Congruency detection (Peter Kovesi)
- Log-Gabor filters
- Mathematical utility functions

**License:** Academic License (MATLAB Coder)
**Restrictions:** Academic use only, subject to MATLAB Coder license terms
**Used by:** `libs/imgproc` (phase congruency features)

[📄 Full Documentation](matlab_coder/README.md)

---

### [`lsd_external/`](lsd_external/README.md)

**External LSD Implementations - Mixed Licenses (AGPL, BSD, Academic)**

Research implementations of Line Segment Detection algorithms:

| Implementation | Authors | License | Notes |
|----------------|---------|---------|-------|
| **LSD** | Grompone von Gioi et al. | **AGPL v3.0** | ⚠️ Strong copyleft - source disclosure required |
| **EDLines Zero** | Akinlar & Topal | BSD-style | Citation required |
| **KHT** | Fernandes & Oliveira | Academic | Citation required |

**Used by:** `libs/lsd` (algorithm implementations)

[📄 Full Documentation](lsd_external/README.md)

## License Summary

| Component | License | Commercial Use | Source Disclosure | Citation Required |
|-----------|---------|----------------|-------------------|-------------------|
| `matlab_coder/` | Academic (MATLAB) | ❌ No | ⚠️ Depends | ⚠️ Check terms |
| `lsd_external/lsd_fgioi` | **AGPL v3.0** | ⚠️ Complex | ✅ Yes (network use) | ✅ Yes |
| `lsd_external/edlines` | BSD-style | ✅ Generally yes | ❌ No | ✅ Yes |
| `lsd_external/kht` | Academic | ⚠️ Check terms | ❌ No | ✅ Yes |

**Main LineExtraction:** MIT License ✅ (commercial-friendly, minimal restrictions)

## Integration

Each subdirectory contains:

- `BUILD.bazel` - Bazel build configuration
- `CMakeLists.txt` - CMake build configuration
- `README.md` - Detailed licensing, authors, and usage information
- `src/` - Implementation files
- `include/` - Header files

Libraries are built with relaxed compiler warnings (auto-generated/external code):

```cmake
-Wno-old-style-cast
-Wno-conversion
-Wno-sign-conversion
-Wno-effc++
```

## Build

### Bazel (Primary)

```bash
# Build all contrib libraries
bazel build //third-party/contrib/...

# Build individual libraries
bazel build //third-party/contrib/matlab_coder
bazel build //third-party/contrib/lsd_external
```

### CMake (Legacy)

```bash
cd build
cmake --build . --target contrib_matlab_coder
cmake --build . --target contrib_lsd_external
```

## Usage in Code

### Bazel

```python
# In BUILD.bazel
deps = [
    "//third-party/contrib/matlab_coder",
    "//third-party/contrib/lsd_external",
]
```

### CMake

```cmake
# In CMakeLists.txt
target_link_libraries(my_target
    contrib_matlab_coder
    contrib_lsd_external
)
```

## Commercial Use Warning

⚠️ **Before using in commercial or proprietary projects:**

1. **MATLAB Coder** (`matlab_coder/`):
   - Academic License restrictions apply
   - Contact MathWorks for commercial licensing
   - Consider alternative implementations

2. **AGPL Code** (`lsd_fgioi`):
   - Requires source code disclosure for network/service use
   - GPL-incompatible for many commercial applications
   - Contact original authors for commercial licensing
   - **Alternative:** Use other LSD implementations in `libs/lsd`

3. **Academic Licenses** (EDLines, KHT):
   - Review original license terms
   - Typically require citation in publications
   - May restrict commercial use without permission

**Safe Alternative:** The main LineExtraction codebase (outside `third-party/contrib/`) is MIT-licensed and suitable for commercial use.

## Contributions

**Do not add code to this directory** unless:

1. It is third-party code with clear licensing
2. It cannot be integrated under MIT license
3. There are compelling technical reasons to include it
4. Proper attribution and documentation are provided

Prefer MIT-compatible alternatives when possible.

## References

- Main Project: [LineExtraction README](../../README.md)
- MATLAB Coder: [matlab_coder/README.md](matlab_coder/README.md)
- LSD External: [lsd_external/README.md](lsd_external/README.md)
- License FAQ: <https://choosealicense.com/>

---

**Questions about licensing?** Consult individual README files or contact the original authors of third-party code.
