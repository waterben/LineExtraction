# Bazel Build Setup - Status und nächste Schritte

## ⚠️ Bekannte Probleme

### JPEG Linker-Fehler bei Tests

**Problem**: Alle Tests schlagen mit `undefined symbol: jpeg_nbits_table` fehl

```
symbol lookup error: .../libexternal_Slibjpeg_Uturbo+_Slibjpeg.so: undefined symbol: jpeg_nbits_table
```

**Ursache**: OpenCV 4.12.0 aus dem Bazel Central Registry (BCR) hat ein Problem mit der libjpeg-turbo Dependency. Das Symbol `jpeg_nbits_table` fehlt in der verlinkten libjpeg-turbo Version.

**Status**:

- ✅ Alle Libraries **kompilieren** erfolgreich
- ✅ Alle Binaries **kompilieren** erfolgreich
- ❌ Tests **laufen nicht** wegen JPEG linking issue

**Workaround Optionen**:

1. **OpenCV lokal installieren** und via `new_local_repository` einbinden (wie Qt5)
2. **Eigene OpenCV bazel rules** erstellen mit funktionierender libjpeg-turbo
3. **Warten** auf BCR fix für OpenCV

**Betroffene Tests**: Alle 54 Tests (alle verwenden OpenCV indirekt über die Libraries)

### Feature Flags

**FastNlMeansOperator**: Benötigt opencv2/photo Modul (nicht in BCR OpenCV verfügbar)

```bash
# Code ist conditional mit #ifdef HAVE_OPENCV_PHOTO guards versehen
# Kann aktiviert werden sobald OpenCV mit photo module verfügbar ist
bazel build --//bazel:enable_photo=true //libs/imgproc:lib_imgproc
```

**Betroffene Files**:

- `libs/imgproc/include/imgproc/image_operator.hpp`
- `libs/imgproc/tests/test_image_operator.cpp`
- `evaluation/thesis/image_denoise.cpp`

## 🆕 Bazel 8.4.2 Migration

**Status**: ✅ Konfiguration aktualisiert

- `.bazelversion` auf 8.4.2 erhöht
- Alle `glob()` Patterns mit `allow_empty=True` Parameter erweitert für strikte Bazel 8.x Validierung
- MODULE.bazel Dependency-Versionen an Bazel 8.x angepasst
- Warnings über dependency resolution (bazel_skylib 1.7.1, rules_cc 0.1.1, rules_python 0.40.0)

### Anpassungen für Bazel 8.x

- **Glob Patterns**: Alle BUILD.bazel Dateien verwenden nun `allow_empty=True`
- **Externe Dependencies**: Automatische version resolution durch bzlmod
- **OpenCV BUILD**: Header patterns erweitert für generated und source includes
- **Library Structure**: .c files zusätzlich zu .cpp files in geometry library

## ✅ Fertiggestellt

1. **MODULE.bazel** - Bzlmod-Konfiguration mit:
   - GoogleTest (1.14.0.bcr.1) über BCR
   - Eigen (3.4.0) über http_archive (BCR-Version hatte Checksum-Fehler)
   - pybind11 (2.11.1) über BCR
   - dlib (19.24) über http_archive
   - rules_cc (0.0.9 → 0.1.1 auto-resolved)
   - rules_python (0.31.0 → 0.40.0 auto-resolved)
   - bazel_skylib (1.5.0 → 1.7.1 auto-resolved)

2. **.bazelrc** - Konfiguration für:
   - GCC und Clang Compiler-Auswahl
   - Debug/Release Modi
   - Sanitizer (ASan, TSan)
   - Performance-Optimierungen

3. **Library BUILD-Dateien** - Für alle Core-Bibliotheken:
   - `//libs/utility` ✅ (inkl. src/*.h für OpenCV integration)
   - `//libs/geometry` ✅ (inkl. src/*.h und src/*.c für tr.c)
   - `//libs/imgproc` ✅ (inkl. src/*.h und impl/*.hpp)
   - `//libs/edge` ⚠️ (build OK, linking mit OpenCV fehlt)
   - `//libs/lsd` ✅ (inkl. impl/*.hpp)
   - `//libs/lfd` ⚠️ (build OK, linking mit OpenCV fehlt)
   - `//libs/eval` ⚠️ (build OK, linking mit OpenCV fehlt)

4. **Third-Party Bibliotheken**:
   - `//third-party/qplot` ✅
   - `//third-party/qplot3d` ✅
   - `//tools/bazel/third_party` - Bazel BUILD Dateien
     - eigen.BUILD ✅
     - dlib.BUILD ✅
     - opencv.BUILD ✅
     - qt5.BUILD ✅

5. **Example BUILD-Dateien**:
   - `//examples/edge` (bereit für Tests)
   - `//examples/lsd` (bereit für Tests)

6. **Evaluation BUILD-Dateien**:
   - `//evaluation/performance` (bereit für Tests)

7. **Tests**: Automatische Test-Target-Generierung für alle Bibliotheken

## ⚠️ Aktuelle Herausforderung

**OpenCV Integration**: Das Projekt verwendet `extern/managed_opencv`, welches über CMake gebaut wird.

### Problem - OpenCV Linking

Bazel benötigt die OpenCV-Header aus verschiedenen Quellen:

- Generierte Header: `managed_opencv-build/opencv2/...`
- Modul-Header: `managed_opencv/modules/*/include/...`

### Lösungsansätze

**Option 1**: OpenCV vollständig über CMake bauen (empfohlen)

```bash
# Vor dem Bazel-Build:
cmake -B build -S . && cmake --build build -j4
bazel build //...
```

**Option 2**: Bazel-only mit vorge bauten OpenCV

- OpenCV BUILD-Datei erweitern
- Symlinks oder filegroup für Header erstellen

**Option 3**: System-OpenCV verwenden

```bash
# Ubuntu/Debian
sudo apt install libopencv-dev

# MODULE.bazel anpassen:
# path = "/usr" statt managed_opencv
```

## 🎯 Quick Start (empfohlen)

### 1. CMake Build zuerst ausführen

```bash
cmake -B build -S .
cmake --build build --target lib_utility lib_geometry lib_imgproc lib_edge lib_lsd
```

### 2. Bazel Build

```bash
# Einzelne Bibliothek
bazel build //libs/geometry:lib_geometry

# Alle Libraries (die nicht OpenCV benötigen)
bazel build //libs/...

# Tests
bazel test //libs/geometry:test_camera
```

## 📝 Verwendung

### Libraries bauen

```bash
bazel build //libs/lsd:lib_lsd
bazel build //libs/lfd:lib_lfd
```

### Tests ausführen

```bash
# Alle Tests
bazel test //libs/...

# Spezifische Tests
bazel test //libs/geometry:test_camera
bazel test //libs/utility:test_value
```

### Examples bauen

```bash
bazel build //examples/edge:edge_test
bazel build //examples/lsd:lsd
```

### Mit verschiedenen Compilern

```bash
# Mit Clang
bazel build --config=clang //libs/...

# Debug-Build
bazel build --config=debug //libs/...
```

## 📂 Dateistruktur

```
LineExtraction/
├── MODULE.bazel              # Bzlmod Dependencies
├── .bazelrc                  # Build-Konfiguration
├── .bazelversion            # Bazel 8.4.2
├── BUILD.bazel               # Root targets
├── libs/*/BUILD.bazel        # Library BUILD-Dateien
├── apps/*/BUILD.bazel        # Application BUILD-Dateien
├── examples/*/BUILD.bazel    # Example BUILD-Dateien
├── third-party/              # Original third-party libs
│   ├── qplot/BUILD.bazel
│   └── qplot3d/BUILD.bazel
└── tools/
    ├── toolchains/BUILD.bazel
    └── bazel/
        ├── qt.bzl
        └── third_party/      # Bazel BUILD-Dateien für externe Dependencies
            ├── BUILD.bazel
            ├── eigen.BUILD
            ├── dlib.BUILD
            ├── opencv.BUILD
            └── qt5.BUILD
```

## 🔧 Nächste Schritte

1. **OpenCV vollständig integrieren** - siehe Option 1-3 oben
2. **Qt5 MOC/UIC Support** - für line_analyzer App
3. **Python Bindings** - mit pybind11
4. **CI/CD Integration** - GitHub Actions mit Bazel

## 💡 Tipps

- **Cache nutzen**: `--disk_cache=~/.cache/bazel/line_extraction` (bereits in .bazelrc)
- **Schnellere Builds**: `bazel build --jobs=4 //...`
- **Cache löschen**: `bazel clean --expunge`
- **Dependencies prüfen**: `bazel query 'deps(//libs/lsd:lib_lsd)'`

## 📚 Dokumentation

Siehe:

- `docs/BAZEL_BUILD.md` - Vollständige Build-Anleitung
- `.bazelrc` - Alle verfügbaren Konfigurationen
- `MODULE.bazel` - Dependency-Management

## ✨ Fazit

Die Bazel-Infrastruktur ist zu **80% fertig**. Die Hauptkomponenten (Libraries, Tests, Examples) sind konfiguriert. Die OpenCV-Integration benötigt noch Feinabstimmung, funktioniert aber mit einem vorherigen CMake-Build.

---

## Feature Flag System (November 9, 2025)

### Summary

Implemented CMake-like conditional compilation using Bazel feature flags. Qt5, CUDA, and OpenGL support can now be enabled/disabled at build time.

### New Files

- `bazel/BUILD.bazel` - Feature flag definitions
- `bazel/features.bzl` - Helper functions for conditional compilation
- `bazel/README.md` - Complete documentation

### Feature Flags

| Feature | Flag | Default |
|---------|------|---------|
| Qt5 | `--//bazel:enable_qt5` | `false` |
| CUDA | `--//bazel:enable_cuda` | `false` |
| OpenGL | `--//bazel:enable_opengl` | `false` |
| OpenCV Photo | `--//bazel:enable_photo` | `false` |

### Conditional Targets

- `//apps/line_analyzer` - Requires `--//bazel:enable_qt5=true --//bazel:enable_opengl=true`
- `//third-party/qplot` - Requires `--//bazel:enable_qt5=true`
- `//third-party/qplot3d` - Requires `--//bazel:enable_qt5=true --//bazel:enable_opengl=true`

### Usage Examples

```bash
# Build all (Qt5 disabled by default)
bazel build //...

# Build with Qt5
bazel build --//bazel:enable_qt5=true --//bazel:enable_opengl=true //apps/line_analyzer

# Exclude Qt5 targets
bazel test --build_tag_filters=-qt5 //...
```

### Configuration File

Create `.bazelrc.user` (git-ignored) for machine-specific settings:

```bash
build --//bazel:enable_qt5=false
build --build_tag_filters=-qt5
test --test_tag_filters=-qt5
```
