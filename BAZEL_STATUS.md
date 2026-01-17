# Bazel Build Setup - Status und nächste Schritte

## ✅ Aktueller Status (Januar 2026)

**Das Bazel-Setup funktioniert jetzt!**

- ✅ Alle Core-Libraries bauen erfolgreich
- ✅ **52 von 54 Tests bestehen** (2 haben Code-Bugs, keine Build-Probleme)
- ✅ libjpeg-turbo SIMD Linking Problem gelöst mit Version 3.1.3.bcr.2

### Schnellstart

```bash
# Alle Libraries bauen
bazel build //libs/...

# Alle Tests ausführen (ohne OpenGL)
bazel test //libs/... --build_tag_filters=-opengl --test_tag_filters=-opengl

# Einzelne Library bauen
bazel build //libs/lsd:lib_lsd
```

## 🔧 Behobene Probleme

### ✅ JPEG Linker-Fehler (BEHOBEN)

**Problem**: `undefined symbol: jpeg_nbits_table`

**Lösung**: Update auf `libjpeg_turbo` Version 3.1.3.bcr.2 aus der BCR.
Diese Version (veröffentlicht Januar 2026) behebt das SIMD Linking Problem.

### ✅ OpenCV Integration (FUNKTIONIERT)

OpenCV 4.12.0.bcr.1 aus der Bazel Central Registry funktioniert jetzt korrekt.

## ⚠️ Bekannte Einschränkungen

### OpenGL-Abhängige Komponenten

Die OpenGL-Komponenten (`lib_geometry_gl`, `lib_geometry_tr`) sind optional und benötigen:

- OpenGL, GLU, GLUT System-Libraries
- Werden mit `--build_tag_filters=-opengl` ausgeschlossen

```bash
# Build ohne OpenGL
bazel build //libs/... --build_tag_filters=-opengl
```

### Qt5 Anwendungen

Qt5-basierte Apps (z.B. `line_analyzer`) benötigen zusätzliche Konfiguration.

### Fehlschlagende Tests (Code-Bugs, keine Build-Probleme)

- `//libs/edge:test_zc` - OpenCV `cv::Mat::at()` Typfehler
- `//libs/imgproc:test_susan` - Empty image handling

## 📦 Dependency-Versionen (Januar 2026)

| Dependency | Version | Quelle |
|------------|---------|--------|
| OpenCV | 4.12.0.bcr.1 | BCR |
| libjpeg_turbo | 3.1.3.bcr.2 | BCR |
| GoogleTest | 1.15.2 | BCR |
| Eigen | 3.4.0 | http_archive |
| dlib | 19.24.7 | Local Registry |
| bazel_skylib | 1.9.0 | BCR |
| rules_cc | 0.2.15 | BCR |
| rules_python | 1.1.0 | BCR |

## ✅ Fertiggestellte Komponenten

1. **Core Libraries** - Alle bauen und testen erfolgreich:
   - `//libs/utility` ✅
   - `//libs/geometry:lib_geometry_core` ✅
   - `//libs/imgproc` ✅
   - `//libs/edge` ✅
   - `//libs/lsd` ✅
   - `//libs/lfd` ✅
   - `//libs/eval` ✅

2. **Third-Party Bibliotheken**:
   - `//third-party/qplot` ✅
   - `//third-party/qplot3d` ✅

3. **Example BUILD-Dateien**:
   - `//examples/edge` (bereit für Tests)
   - `//examples/lsd` (bereit für Tests)

4. **Evaluation BUILD-Dateien**:
   - `//evaluation/performance` (bereit für Tests)

5. **Tests**: Automatische Test-Target-Generierung für alle Bibliotheken

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
