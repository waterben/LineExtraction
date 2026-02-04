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

## Build

```bash
# Bazel
bazel build //libs/utility:lib_utility
bazel test //libs/utility:test_utility

# CMake
cmake --build build --target lib_utility
```

## Dependencies

- OpenCV (core, imgproc)
- C++17 standard library

## See Also

- [Examples](../../examples/README.md) - Usage examples
- [eval](../eval/README.md) - Uses TestImages for evaluation
- [Main README](../../README.md) - Project overview
