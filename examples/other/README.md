# Miscellaneous Examples

Other examples demonstrating comparison with standard OpenCV algorithms.

[← Back to Examples](../README.md)

## Examples

| Bazel Target | Source | Description |
|---|---|---|
| `test_other_houghlines` | [houghlines.cpp](src/houghlines.cpp) | OpenCV Hough line detection: standard vs probabilistic Hough Transform |

### C++ (Rerun Visualization)

| Bazel Target | Source | Description |
|---|---|---|
| `rerun_cpp_demo` | [rerun_demo.cpp](src/rerun_demo.cpp) | Rerun C++ SDK demo — gradient image, animated star line strips, grid keypoints |

### Python (Rerun Visualization)

| Bazel Target | Source | Description |
|---|---|---|
| `rerun_general_demo` | [rerun_general_demo.py](python/rerun_general_demo.py) | General [Rerun.io](https://rerun.io/) demo with synthetic data — images, line strips, points, timelines |

## Building & Running

```bash
bazel build //examples/other:all
bazel run //examples/other:test_other_houghlines
```

### Rerun Demos

> **First:** Start the Rerun viewer before running any demo.
> See [docs/RERUN.md](../../docs/RERUN.md) for platform-specific setup (WSL, Docker, headless).

```bash
# --- Quick start (web viewer — works on all platforms) ---
rerun --serve &
# Open http://localhost:9090 in your browser

# --- C++ demo ---
bazel run //examples/other:rerun_cpp_demo

# --- Python general demo ---

# Native viewer (requires display)
bazel run //examples/other/python:rerun_general_demo

# Web viewer (WSL / headless — open printed URL in browser)
bazel run //examples/other/python:rerun_general_demo -- --serve

# Save to .rrd file (no viewer needed)
bazel run //examples/other/python:rerun_general_demo -- --save /tmp/general.rrd
```

## Related Libraries

- [libs/geometry](../../libs/geometry/) — Line representation and drawing
- [libs/utility](../../libs/utility/) — Test images and helpers
