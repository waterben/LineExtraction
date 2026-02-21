# Line Segment Detection Examples

Examples demonstrating various Line Segment Detection (LSD) algorithms and their applications.

[← Back to Examples](../README.md)

## Examples

| Bazel Target | Source | Description |
|---|---|---|
| `lsd` | [lsd.cpp](src/lsd.cpp) | Comprehensive comparison of all LSD algorithm variants (see below) |
| `lsd_cc_test` | [lsd_cc_test.cpp](src/lsd_cc_test.cpp) | Connected component-based LSD with edge map analysis |
| `lsd_edge` | [lsd_edge.cpp](src/lsd_edge.cpp) | Edge-based LSD using edge linking (EL) and edge pattern (EP) strategies |
| `lsd_graph` | [lsd_graph.cpp](src/lsd_graph.cpp) | Graph-based line segment detection with junction handling |
| `lsd_nfa` | [lsd_nfa.cpp](src/lsd_nfa.cpp) | NFA-validated line detection — a contrario false positive rejection |
| `lsd_video` | [lsd_video.cpp](src/lsd_video.cpp) | Real-time LSD on webcam or video file |

### Python (Rerun Visualization)

| Bazel Target | Source | Description |
|---|---|---|
| `rerun_lsd_demo` | [rerun_lsd_demo.py](python/rerun_lsd_demo.py) | Interactive LSD visualization with [Rerun.io](https://rerun.io/) — line segments, endpoints, auxiliary data layers |

### Algorithm Variants (in `lsd.cpp`)

| Variant | Approach |
|---------|----------|
| **LsdCC** | Connected component labeling on edge maps |
| **LsdCP** | Control point-based line fitting |
| **LsdEL** | Edge linking — chain edge pixels into segments |
| **LsdEP** | Edge pattern — pattern-based segment grouping |
| **LsdBurns** | Burns' algorithm — hierarchical line fitting |
| **LsdFGIOI** | Grompone's fast LSD with a contrario validation |
| **LsdHCV** | Hierarchical chaining with validation |

## Building & Running

```bash
# Build all LSD examples
bazel build //examples/lsd:all

# Run algorithm comparison on an image
bazel run //examples/lsd:lsd

# Run with custom image
bazel run //examples/lsd:lsd -- /path/to/image.jpg

# Real-time video (webcam by default)
bazel run //examples/lsd:lsd_video

# --- Rerun visualization (Python) ---

# Run with a specific detector (native viewer)
bazel run //examples/lsd/python:rerun_lsd_demo -- --detector CC

# Run all detectors (compare on timeline)
bazel run //examples/lsd/python:rerun_lsd_demo -- --all

# Web viewer (WSL / headless — open printed URL in browser)
bazel run //examples/lsd/python:rerun_lsd_demo -- --serve --detector CC

# Save to .rrd file for later viewing
bazel run //examples/lsd/python:rerun_lsd_demo -- --save /tmp/lsd.rrd
```

## Key Concepts

- **Line Segment Detection:** Extracting straight line segments from images
- **NFA Validation:** Statistical test based on the Helmholtz principle — reject segments that could arise by chance
- **Connected Components:** Group co-directional edge pixels into line support regions
- **Edge Linking:** Chain edge pixels along consistent gradient directions
- **Edge Patterns:** Robust grouping strategy that handles junctions and corners

## Related Libraries

- [libs/lsd](../../libs/lsd/) — Line Segment Detection implementations
- [libs/edge](../../libs/edge/) — Edge detection, NMS, and linking (used as input stage)
