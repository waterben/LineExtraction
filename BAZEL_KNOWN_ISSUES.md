# Bekannte Bazel Build Probleme

## 🔴 JPEG Linker-Fehler (KRITISCH)

### Symptom

Alle Tests schlagen beim Ausführen mit folgendem Fehler fehl:

```
symbol lookup error: .../libexternal_Slibjpeg_Uturbo+_Slibjpeg.so:
    undefined symbol: jpeg_nbits_table
```

### Betroffene Targets

- ✅ **Kompilierung**: Alle Libraries und Binaries bauen erfolgreich
- ❌ **Tests**: Alle 54 Tests schlagen beim Ausführen fehl
- ❌ **Binaries**: Vermutlich auch Runtime-Fehler bei Verwendung von OpenCV Bildformaten

### Root Cause ✅ IDENTIFIZIERT

**Bazel Central Registry libjpeg-turbo 3.1.2 hat einen Linking-Bug**: Pre-compiled
ASM-Objekt-Dateien (`.o`) werden nicht korrekt in die finale `libjpeg.so` gelinkt.

#### Technische Details

1. **Symbol-Definition**
   - `jpeg_nbits_table` ist in `simd/x86_64/jchuff-sse2.asm` definiert
   - Diese ASM-Datei wird zu `jchuff-sse2.o` assembliert
   - Das Objekt sollte Teil der `:simd_x86_64` Library sein

2. **Conditional Compilation Logik**
   - In `src/jpeg_nbits.c` wird das Symbol nur kompiliert wenn `INCLUDE_JPEG_NBITS_TABLE` gesetzt ist
   - Dieses Makro ist **undefined** wenn `WITH_SIMD` auf x86/x64 aktiviert ist
   - Der Kommentar erklärt explizit: *"When building for x86[-64] with the SIMD extensions enabled, the C Huffman encoders can reuse jpeg_nbits_table from the SSE2 baseline Huffman encoder"*

3. **Was passieren sollte**

   ```
   src/jpeg_nbits.c → NICHT kompiliert (SIMD-Pfad)
   simd/x86_64/jchuff-sse2.asm → jchuff-sse2.o → enthält jpeg_nbits_table
   jchuff-sse2.o → gelinkt in simd_x86_64 → gelinkt in libjpeg.so
   ```

4. **Was tatsächlich passiert**

   ```
   src/jpeg_nbits.c → NICHT kompiliert (SIMD-Pfad) ✅ korrekt
   simd/x86_64/jchuff-sse2.asm → jchuff-sse2.o ✅ korrekt
   jchuff-sse2.o hat das Symbol (mit nm verifiziert) ✅ korrekt
   jchuff-sse2.o → NICHT in libjpeg.so gelinkt ❌ BUG!
   ```

5. **Beweis**

   ```bash
   # Symbol existiert in ASM-Objekt:
   $ nm bazel-bin/external/libjpeg_turbo+/simd/x86_64/jchuff-sse2.o | grep jpeg_nbits_table
   0000000000008040 R jpeg_nbits_table  # ✅ als read-only data definiert

   # Symbol ist undefined in finaler .so:
   $ nm -D bazel-bin/external/libjpeg_turbo+/libjpeg.so | grep jpeg_nbits_table
                    U jpeg_nbits_table  # ❌ undefined!

   # Nur C-Source in der .so (ASM fehlt):
   $ readelf -s bazel-bin/external/libjpeg_turbo+/libjpeg.so | grep jchuff
   24: 0000000000000000  0 FILE LOCAL DEFAULT ABS jchuff.c  # Nur C, kein jchuff-sse2.o
   ```

#### Warum das passiert

Das libjpeg-turbo BCR `BUILD.bazel` deklariert pre-compiled `.o` Files im `srcs`-Attribut:

```python
cc_library(
    name = "simd_x86_64",
    srcs = [
        "simd/x86_64/jchuff-sse2.o",  # Pre-compiled object
        ...
    ],
)
```

Bazels `cc_library` Rule hat jedoch Probleme mit pre-compiled Objects im `srcs`-Feld
beim Bauen von Shared Libraries. Die Objekte werden möglicherweise nicht korrekt in
den finalen Link-Befehl aufgenommen.

### Build vs. Runtime

```bash
# ✅ BUILD FUNKTIONIERT
bazel build //...        # Erfolgreich
bazel build //libs/...   # Erfolgreich
bazel build //examples/...  # Erfolgreich

# ❌ TESTS SCHLAGEN FEHL
bazel test //...         # 54/54 tests failed
bazel test //libs/...    # Alle tests failed
```

### Workarounds

#### Option 1: Lokale OpenCV Installation (EMPFOHLEN)

```bash
# System OpenCV installieren
sudo apt-get install libopencv-dev libopencv-contrib-dev

# MODULE.bazel: OpenCV BCR dependency entfernen/kommentieren
# bazel_dep(name = "opencv", version = "4.12.0")

# WORKSPACE.bazel: Lokale installation hinzufügen
new_local_repository(
    name = "opencv",
    build_file = "//third-party:opencv.BUILD",
    path = "/usr",
)
```

**Vorteile**:

- ✅ Funktioniert sofort
- ✅ Alle module verfügbar (photo, contrib, etc.)
- ✅ Gleiche Version wie CMake Build

**Nachteile**:

- ⚠️ Nicht hermetic (System-Abhängigkeit)
- ⚠️ Schwieriger für andere Entwickler

#### Option 2: Eigene OpenCV Bazel Rules

Verwende `rules_foreign_cc` um OpenCV aus Source zu bauen:

```python
# MODULE.bazel
bazel_dep(name = "rules_foreign_cc", version = "0.10.1")

# BUILD.bazel
cmake(
    name = "opencv",
    lib_source = "@opencv_source//:all",
    out_static_libs = ["libopencv_core.a", ...],
)
```

**Vorteile**:

- ✅ Hermetic build
- ✅ Volle Kontrolle über Optionen

**Nachteile**:

- ⏱️ Sehr lange Build-Zeit
- 🔧 Komplex zu konfigurieren

#### Option 3: Warten auf BCR Fix

Issue beim OpenCV BCR repository melden und auf Fix warten.

### Temporäre Lösung

Für development kann CMake weiterhin verwendet werden:

```bash
# CMake Build (funktioniert normal)
cd build
cmake ..
make -j$(nproc)
make test  # ✅ Tests laufen

# Bazel Build (für code structure/linting)
bazel build //...  # ✅ Kompiliert
# Tests müssen mit CMake ausgeführt werden
```

## ⚠️ Fehlende OpenCV Module

### opencv2/photo Module

Das `photo` module (für FastNlMeansOperator) ist nicht im BCR OpenCV enthalten.

**Status**: Code ist mit `#ifdef HAVE_OPENCV_PHOTO` guards versehen

**Betroffene Files**:

- `libs/imgproc/include/imgproc/image_operator.hpp`
- `libs/imgproc/tests/test_image_operator.cpp`
- `evaluation/thesis/image_denoise.cpp`

**Aktivierung** (sobald OpenCV mit photo verfügbar):

```bash
bazel build --//bazel:enable_photo=true //...
```

## 📊 Test Status Summary

```
Kompilierung:  ✅ 100% erfolgreich
Build:         ✅ 100% erfolgreich
Tests:         ❌   0% erfolgreich (JPEG linking issue)
```

**Fazit**: Der Bazel Build ist technisch korrekt konfiguriert. Das Runtime-Problem
ist ein Linking-Issue zwischen OpenCV BCR und libjpeg-turbo BCR. Die genaue Root
Cause muss noch identifiziert werden.
