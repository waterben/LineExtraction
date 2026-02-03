# Line Segment Detection Examples

Examples demonstrating various Line Segment Detection (LSD) algorithms and their applications.

[← Back to Examples](../README.md)

## Examples

| Example | Description | Usage |
|---------|-------------|-------|
| [lsd.cpp](lsd.cpp) | Comprehensive LSD algorithm comparison | `./lsd [image_path]` |
| [lsd_cc_test.cpp](lsd_cc_test.cpp) | Connected component-based LSD | `./lsd_cc_test [image_path]` |
| [lsd_edge.cpp](lsd_edge.cpp) | Edge-based LSD with pattern detection | `./lsd_edge [image_path]` |
| [lsd_graph.cpp](lsd_graph.cpp) | Graph-based line segment detection | `./lsd_graph [image_path]` |
| [lsd_nfa.cpp](lsd_nfa.cpp) | NFA-validated line detection | `./lsd_nfa [image_path]` |
| [lsd_video.cpp](lsd_video.cpp) | Real-time LSD on video | `./lsd_video [video_path]` |

### Algorithm Variants in lsd.cpp

- **LsdCC:** Connected component-based
- **LsdCP:** Control point-based
- **LsdEL:** Edge linking
- **LsdEP:** Edge pattern
- **LsdBurns:** Burns' algorithm
- **LsdFGIOI:** Fast LSD (a contrario)
- **LsdHCV:** Hierarchical chaining validation

## Building

**Bazel:**
```bash
bazel build //examples/lsd:all
bazel run //examples/lsd:lsd -- /path/to/image.jpg
bazel run //examples/lsd:lsd_video  # uses webcam
```

**CMake:**
```bash
make  # from build directory
```

## Key Algorithms

- **LSD (Line Segment Detector):** Fast line segment detection
- **NFA Validation:** Statistical significance testing via a contrario framework
- **Connected Components:** Region-based line detection
- **Edge Linking:** Chaining edge pixels into line segments
- **Graph-based:** Network of connected line segments
- **Burns' Algorithm:** Hierarchical line fitting

## Related Libraries

- [libs/lsd](../../libs/lsd/) - Line Segment Detection library
- [libs/edge](../../libs/edge/) - Edge detection library
