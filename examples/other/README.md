# Miscellaneous Examples

Other examples demonstrating comparison with standard OpenCV algorithms.

[← Back to Examples](../README.md)

## Examples

| Bazel Target | Source | Description |
|---|---|---|
| `test_other_houghlines` | [houghlines.cpp](src/houghlines.cpp) | OpenCV Hough line detection: standard vs probabilistic Hough Transform |

### Python (Rerun Visualization)

| Bazel Target | Source | Description |
|---|---|---|
| `rerun_general_demo` | [rerun_general_demo.py](python/rerun_general_demo.py) | General [Rerun.io](https://rerun.io/) demo with synthetic data — images, line strips, points, timelines |

## Building & Running

```bash
bazel build //examples/other:all
bazel run //examples/other:test_other_houghlines

# --- Rerun general demo (Python) ---

# Native viewer
bazel run //examples/other/python:rerun_general_demo

# Web viewer (WSL / headless — open printed URL in browser)
bazel run //examples/other/python:rerun_general_demo -- --serve

# Save to .rrd file
bazel run //examples/other/python:rerun_general_demo -- --save /tmp/general.rrd
```

## Related Libraries

- [libs/geometry](../../libs/geometry/) — Line representation and drawing
- [libs/utility](../../libs/utility/) — Test images and helpers
