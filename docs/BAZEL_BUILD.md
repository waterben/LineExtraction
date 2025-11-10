# Bazel Build System für LineExtraction

Dieses Projekt unterstützt nun sowohl CMake als auch Bazel als Build-Systeme.

## Voraussetzungen

- Bazel 8.4.2 oder höher (siehe `.bazelversion`)
- GCC oder Clang
- System-installierte Bibliotheken:
  - OpenCV 4.x (in `/usr/local`)
  - Qt5 (in `/usr`)
  - OpenGL/GLU/GLUT

## Schnellstart

### Alles bauen

```bash
# Mit Standard-Konfiguration (GCC, Release)
bazel build //...

# Mit Clang
bazel build --config=clang //...

# Debug-Build mit GCC
bazel build --config=gcc --config=debug //...
```

### Einzelne Targets bauen

```bash
# Eine Bibliothek bauen
bazel build //libs/utility:lib_utility
bazel build //libs/lsd:lib_lsd

# Eine Anwendung bauen
bazel build //apps/line_analyzer:app_line_analyzer

# Ein Example bauen
bazel build //examples/lsd:lsd
bazel build //examples/edge:edge_test
```

### Tests ausführen

```bash
# Alle Tests ausführen
bazel test //...

# Spezifische Tests
bazel test //libs/geometry:test_camera
bazel test //libs/utility:test_value

# Tests mit Debug-Ausgabe
bazel test --test_output=all //libs/...
```

### Binaries ausführen

```bash
# Nach dem Build
bazel run //examples/lsd:lsd

# Mit Argumenten
bazel run //apps/line_analyzer:app_line_analyzer -- --help
```

## Konfigurationsoptionen

### Compiler-Auswahl

```bash
# GCC verwenden (Standard)
bazel build --config=gcc //...

# Clang verwenden
bazel build --config=clang //...
```

### Build-Modi

```bash
# Release-Modus (Standard): -O3, optimiert
bazel build --config=release //...

# Debug-Modus: -g, -O0, keine Optimierung
bazel build --config=debug //...

# Erweiterte Debug-Features
bazel build --config=debugging_feature //...
```

### Sanitizer

```bash
# Address Sanitizer
bazel build --config=asan //libs/utility:test_value
bazel run --config=asan //libs/utility:test_value

# Thread Sanitizer
bazel build --config=tsan //...
```

## Projektstruktur

```
LineExtraction/
├── MODULE.bazel              # Bzlmod-Konfiguration mit Dependencies
├── .bazelrc                  # Bazel-Konfiguration
├── .bazelversion             # Bazel-Version (7.4.1)
├── BUILD.bazel               # Root BUILD-Datei
├── libs/                     # Bibliotheken
│   ├── utility/BUILD.bazel
│   ├── geometry/BUILD.bazel
│   ├── imgproc/BUILD.bazel
│   ├── edge/BUILD.bazel
│   ├── lsd/BUILD.bazel
│   ├── lfd/BUILD.bazel
│   └── eval/BUILD.bazel
├── apps/                     # Anwendungen
│   └── line_analyzer/BUILD.bazel
├── examples/                 # Beispiele
│   ├── edge/BUILD.bazel
│   └── lsd/BUILD.bazel
├── evaluation/               # Evaluation
│   └── performance/BUILD.bazel
├── third-party/              # Third-party Bibliotheken (original)
│   ├── qplot/BUILD.bazel
│   └── qplot3d/BUILD.bazel
└── tools/
    ├── toolchains/BUILD.bazel # Toolchain-Definitionen
    └── bazel/
        ├── qt.bzl            # Qt-Utilities
        └── third_party/      # Bazel BUILD-Dateien für externe Dependencies
            ├── BUILD.bazel
            ├── eigen.BUILD   # Eigen Linear Algebra
            ├── dlib.BUILD    # dlib Computer Vision
            ├── opencv.BUILD  # OpenCV (managed build)
            └── qt5.BUILD     # Qt5 System-Integration
```

## Dependencies

Die folgenden Dependencies werden über Bzlmod (`MODULE.bazel`) verwaltet:

- **GoogleTest** (1.14.0) - C++ Testing Framework
- **Eigen** (3.4.0) - Lineare Algebra
- **pybind11** (2.11.1) - Python Bindings
- **dlib** (19.24) - Computer Vision Library

System-installierte Bibliotheken:

- **OpenCV** - über `tools/bazel/third_party/opencv.BUILD`
- **Qt5** - über `tools/bazel/third_party/qt5.BUILD`

## Toolchain-Konfiguration

Das Projekt unterstützt konfigurierbare Toolchains für GCC und Clang.
Die Toolchain-Definitionen befinden sich in `tools/toolchains/BUILD.bazel`.

## Troubleshooting

### OpenCV nicht gefunden

Passe den Pfad in `MODULE.bazel` an:

```python
local_repo(
    name = "opencv",
    path = "/usr/local",  # oder /usr, oder dein OpenCV-Installationspfad
)
```

### Qt5 nicht gefunden

Prüfe, ob Qt5 installiert ist und passe ggf. die Pfade in `tools/bazel/third_party/qt5.BUILD` an:

```bash
# Qt5 Pakete prüfen
dpkg -l | grep qt5

# Qt5 Pfade finden
find /usr -name "Qt5Core" -type d
```

### Build-Fehler bei Tests

Stelle sicher, dass alle Test-Abhängigkeiten vorhanden sind:

```bash
# Nur Libraries bauen, ohne Tests
bazel build //libs/utility:lib_utility --build_tests_only=false
```

## Benutzerdefinierte Konfiguration

Erstelle eine `.bazelrc.user` Datei im Projekt-Root für benutzerdefinierte Einstellungen:

```bash
# .bazelrc.user
build --jobs=8
build --config=clang
test --test_verbose_timeout_warnings
```

Diese Datei wird automatisch geladen, wenn sie existiert.

## Performance-Tipps

```bash
# Disk-Cache verwenden (Standard aktiviert)
build --disk_cache=~/.cache/bazel/line_extraction

# Parallele Jobs begrenzen
bazel build --jobs=4 //...

# Inkrementelle Builds beschleunigen
bazel build --keep_going //...
```

## Vergleich mit CMake

| Feature | CMake | Bazel |
|---------|-------|-------|
| Konfiguration | `cmake -B build` | Über `.bazelrc` |
| Build | `cmake --build build` | `bazel build //...` |
| Tests | `ctest` | `bazel test //...` |
| Install | `cmake --install` | N/A (direkter Run) |
| Clean | `rm -rf build` | `bazel clean` |

Beide Build-Systeme können parallel verwendet werden.
