# Miscellaneous Examples

Other examples demonstrating comparison with standard OpenCV algorithms.

[← Back to Examples](../README.md)

## Examples

| Bazel Target | Source | Description |
|---|---|---|
| `test_other_houghlines` | [houghlines.cpp](src/houghlines.cpp) | OpenCV Hough line detection: standard vs probabilistic Hough Transform |

## Building & Running

```bash
bazel build //examples/other:all
bazel run //examples/other:test_other_houghlines
```

## Related Libraries

- [libs/geometry](../../libs/geometry/) — Line representation and drawing
- [libs/utility](../../libs/utility/) — Test images and helpers
