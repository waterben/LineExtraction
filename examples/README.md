# LineExtraction Examples

Demonstration programs showcasing the capabilities of the LineExtraction library. Each example demonstrates specific features and algorithms from the core libraries.

## Directory Structure

| Directory | Description |
|-----------|-------------|
| [edge/](edge/) | Edge detection, NMS, and edge linking algorithms |
| [geometry/](geometry/) | Geometric primitives, camera models, stereo vision |
| [imgproc/](imgproc/) | Image processing filters and transforms |
| [lfd/](lfd/) | Line Feature Descriptor matching |
| [lsd/](lsd/) | Line Segment Detection algorithms |
| [other/](other/) | Miscellaneous utilities and external library demos |
| [qt/](qt/) | Qt-based visualization examples |
| [thesis/](thesis/) | Examples for thesis figure generation |

See the README.md in each subdirectory for detailed information about specific examples.

## Building Examples

### Bazel (Recommended)

```bash
# Build all examples
bazel build //examples/...

# Build specific category
bazel build //examples/edge:all
bazel build //examples/lsd:all

# Run specific example
bazel run //examples/lsd:lsd -- /path/to/image.jpg
```

### CMake (Legacy)

```bash
mkdir build && cd build
cmake ..
make
./bin/lsd /path/to/image.jpg
```

## Usage

Most examples accept command-line arguments:

```bash
./example_name                    # Run with default test image
./example_name /path/to/image.jpg # Run with custom image
```

Test images are provided in `resources/datasets/`. The `TestImages` utility class provides access to standard test images:

```cpp
#include <utility/test_images.hpp>

int main(int argc, char** argv) {
    lsfm::TestImages::init(argv[0]);
    std::string filename = argc >= 2 ? argv[1] : lsfm::TestImages::windmill();
    // ...
}
```

## Quick Links

- **Full Documentation:** See main [README.md](../README.md) for complete documentation
- **Edge Examples:** [edge/README.md](edge/README.md)
- **Geometry Examples:** [geometry/README.md](geometry/README.md)
- **Image Processing Examples:** [imgproc/README.md](imgproc/README.md)
- **Line Feature Descriptors:** [lfd/README.md](lfd/README.md)
- **Line Segment Detection:** [lsd/README.md](lsd/README.md)
- **Other Examples:** [other/README.md](other/README.md)
- **Qt Examples:** [qt/README.md](qt/README.md)
- **Thesis Examples:** [thesis/README.md](thesis/README.md)
