# LineExtraction Examples

Demonstration programs showcasing the capabilities of the LineExtraction library. Each subdirectory focuses on a specific library module and contains standalone example binaries.

## Directory Structure

| Directory | Library | Description |
|-----------|---------|-------------|
| [edge/](edge/) | `libs/edge` | Edge detection, NMS, zero-crossing, and edge linking |
| [geometry/](geometry/) | `libs/geometry` | Geometric primitives, camera models, stereo vision |
| [imgproc/](imgproc/) | `libs/imgproc` | Gradient operators, FFT, steerable filters, pyramids |
| [lfd/](lfd/) | `libs/lfd` | Line feature descriptor matching (stereo, video, motion) |
| [lsd/](lsd/) | `libs/lsd` | Line Segment Detection algorithm variants |
| [other/](other/) | — | Miscellaneous demos (Hough transform) |
| [qt/](qt/) | — | Qt-based visualization (currently empty) |
| [thesis/](thesis/) | `libs/eval` | Figure generation for thesis chapters |

## Building

### Bazel (Recommended)

```bash
# Build all examples
bazel build //examples/...

# Build a specific category
bazel build //examples/edge:all
bazel build //examples/lsd:all

# Run a specific example
bazel run //examples/lsd:lsd
bazel run //examples/edge:edge_test
```

### CMake (Legacy)

```bash
cd build && cmake .. && cmake --build . -j$(nproc)
```

## Usage

Most examples accept an optional image path as command-line argument. Without arguments, the built-in test image (`windmill.jpg`) is used:

```bash
bazel run //examples/edge:nms_test              # default test image
bazel run //examples/edge:nms_test -- image.jpg # custom image
```

Test images are resolved via the `TestImages` utility:

```cpp
#include <utility/test_images.hpp>

int main(int argc, char** argv) {
    lsfm::TestImages::init(argv[0]);
    std::string file = argc >= 2 ? argv[1] : lsfm::TestImages::windmill();
}
```

## Project Layout

Each example subdirectory follows the same layout:

```
<category>/
  BUILD.bazel       # Bazel build targets
  CMakeLists.txt    # Legacy CMake build
  README.md         # Documentation
  src/              # Source files (.cpp)
```
