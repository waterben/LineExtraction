# Utility Library

Core utility classes and helper functions used throughout the LineExtraction project.

## Overview

The `utility` library provides foundational components:

- **Value Management**: Type-safe variant values and runtime parameter configuration
- **Options Parsing**: Command-line argument parsing for applications
- **Image Utilities**: Test image resolution, response visualization
- **Math Helpers**: Numeric limits, MATLAB-like functions

## Components

### Value & ValueManager

Type-safe variant class and runtime parameter management.

```cpp
#include <utility/value.hpp>
#include <utility/value_manager.hpp>

// Value supports float, int, bool, string
lsfm::Value v1(3.14);        // FLOAT
lsfm::Value v2(42);          // INT
lsfm::Value v3(true);        // BOOL
lsfm::Value v4("hello");     // STRING

// ValueManager for algorithm parameters
class MyAlgorithm : public lsfm::ValueManager {
public:
    MyAlgorithm() {
        add("threshold", [this](const lsfm::Value& v) {
            if (v.type()) threshold_ = v.getDouble();
            return threshold_;
        }, "Detection threshold");
    }
private:
    double threshold_ = 0.5;
};
```

### Options Parser

Lightweight command-line argument parsing.

```cpp
#include <utility/options.hpp>

std::string input, output;
bool verbose = false;

utility::Options opts;
opts.add_string("input", 'i', "Input file", input, true);
opts.add_string("output", 'o', "Output file", output, false, "out.txt");
opts.add_switch("verbose", 'v', "Verbose output", verbose);

auto remaining = opts.parse(argc, argv);
```

### TestImages

Cross-build-system test image resolution.

```cpp
#include <utility/test_images.hpp>

int main(int argc, char** argv) {
    lsfm::TestImages::init(argv[0]);

    // Works with both Bazel and CMake
    std::string path = lsfm::TestImages::get("windmill.jpg");
    std::string bsds = lsfm::TestImages::bsds500("100007.jpg");
    auto [left, right] = lsfm::TestImages::stereoPair("Adirondack");
}
```

### StringTable

2D string table with CSV export.

```cpp
#include <utility/string_table.hpp>

lsfm::StringTable table(3, 2);
table(0, 0) = "Name";  table(0, 1) = "Score";
table(1, 0) = "Test1"; table(1, 1) = "95";
table(2, 0) = "Test2"; table(2, 1) = "87";

table.saveCSV("results.csv");
std::cout << table;  // Pretty print
```

## Headers

| Header | Description |
|--------|-------------|
| `value.hpp` | Type-safe variant class |
| `value_manager.hpp` | Runtime parameter management |
| `option_manager.hpp` | Legacy option interface |
| `options.hpp` | Command-line argument parser |
| `test_images.hpp` | Test image path resolution |
| `string_table.hpp` | 2D string table with CSV export |
| `range.hpp` | Simple range template |
| `limit.hpp` | Numeric limit constants |
| `format.hpp` | Printf-style string formatting |
| `response_convert.hpp` | Image response visualization |
| `high_prio.hpp` | Process priority control |
| `console_app.hpp` | Console application base class |
| `camera_utilities.hpp` | Stereo calibration utilities |
| `matlab_helpers.hpp` | MATLAB-like matrix functions |
| `matlab_helpers_gui.hpp` | GUI utilities (requires highgui) |

## GUI Components

The utility library provides optional GUI components in a separate library to avoid forcing GUI dependencies on headless builds.

### lib_utility vs lib_utility_gui

**`lib_utility`** (core, no GUI dependencies):

- All utilities except `showMat()`
- Safe for headless/CI builds
- Dependencies: OpenCV (core, imgproc, calib3d), Eigen

**`lib_utility_gui`** (GUI utilities):

- Contains `showMat()` for displaying matrices in windows
- Dependencies: `lib_utility`, OpenCV highgui
- Used only by GUI applications/examples

### Usage in GUI Applications

```cpp
#include <utility/matlab_helpers.hpp>      // Core utilities (no GUI)
#include <utility/matlab_helpers_gui.hpp>  // GUI utilities (requires highgui)

using namespace lsfm;

cv::Mat image = cv::imread("test.png", cv::IMREAD_GRAYSCALE);
showMat("Image", image, IMG_NORM_AUTO);  // Display in window
```

**Bazel:**

```python
cc_binary(
    name = "my_gui_app",
    srcs = ["app.cpp"],
    deps = [
        "//libs/utility:lib_utility",      # Core utilities
        "//libs/utility:lib_utility_gui",  # GUI utilities
    ],
)
```

**CMake:**

```cmake
target_link_libraries(my_gui_app
    lib_utility
    le::opencv  # Includes highgui
)
```

### Design Rationale

Separating GUI code prevents forcing all downstream libraries to link against:

- OpenCV highgui
- GUI backends (GTK, X11, etc.)
- Display system libraries

This ensures:

- ✅ Headless/CI builds work without X11
- ✅ Minimal dependencies for core libraries
- ✅ Docker containers don't need GUI libraries
- ✅ Explicit intent when GUI is needed

**Migration:** If you use `showMat()` in examples/applications, include `matlab_helpers_gui.hpp` and add `lib_utility_gui` dependency.

## Build

```bash
# Bazel
bazel build //libs/utility:lib_utility        # Core library
bazel build //libs/utility:lib_utility_gui    # GUI utilities
bazel test //libs/utility:test_utility

# CMake
cmake --build build --target lib_utility
```

## Dependencies

- OpenCV (core, imgproc, calib3d)
- OpenCV highgui (only for lib_utility_gui)
- Eigen3
- C++17 standard library

## See Also

- [Examples](../../examples/README.md) - Usage examples
- [eval](../eval/README.md) - Uses TestImages for evaluation
- [Main README](../../README.md) - Project overview
