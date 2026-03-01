# Resources

Test images, evaluation datasets, ground truth annotations, and optimized detector presets used throughout the LineExtraction project.

## Directory Structure

```
resources/
├── BUILD.bazel              # Bazel filegroups
├── windmill.jpg             # Default test image
├── example_lines.png        # Synthetic image with known line segments
├── example_challenge.png    # Challenge test image
├── presets/
│   └── lsd_presets.json     # Optimized detector parameter presets
└── datasets/
    ├── BUILD.bazel          # Bazel filegroups for all datasets
    ├── ground_truth/        # CSV ground truth annotations
    │   ├── york_urban_gt.csv        # 12,122 segments from 102 images
    │   ├── wireframe_gt.csv         # 34,287 segments from ~5,000 images
    │   ├── example_gt.csv           # GT for example_lines.png
    │   ├── example_challenge_gt.csv # GT for example_challenge.png
    │   ├── example_gt.txt           # Legacy format (no header)
    │   └── example_challenge_gt.txt # Legacy format (no header)
    ├── noise/               # Synthetic noise test images
    ├── BSDS500/             # Auto-downloaded by Bazel (~50 MB)
    ├── MDB/                 # Middlebury stereo (manual setup)
    ├── YorkUrban/           # York Urban DB (setup script)
    └── Wireframe/           # Wireframe dataset (setup script)
```

## Test Images

| Image | Description | Used By |
|-------|-------------|---------|
| `windmill.jpg` | Default test image for examples | Most examples, CLI defaults |
| `example_lines.png` | Synthetic image with known line geometry | Accuracy tests, tutorials |
| `example_challenge.png` | More complex synthetic test image | Challenge evaluations |

## Datasets

### BSDS500 (Berkeley Segmentation Dataset)

**Auto-managed by Bazel** — no manual setup needed.

- **Source:** [github.com/BIDS/BSDS500](https://github.com/BIDS/BSDS500)
- **Size:** ~50 MB (auto-downloaded on first build)
- **Content:** 500 natural images with human-segmented boundaries
- **Use:** Performance benchmarking (gradient, NMS, LSD timing)
- **Bazel target:** `@bsds500//:all`

### York Urban Line Segment Database

102 images (640×480) with manually annotated line segments. Primary dataset for parameter optimization and accuracy evaluation.

- **Source:** [Elder Lab, York University](https://www.elderlab.yorku.ca/resources/) (Denis, Elder & Estrada, ECCV 2008)
- **Images:** 102 urban outdoor scenes
- **Ground truth:** 12,122 annotated line segments

**Setup:**

```bash
./tools/scripts/setup_york_urban.sh
```

This downloads the dataset (~30 MB), extracts images to `resources/datasets/YorkUrban/images/`, and converts `.mat` annotations to `resources/datasets/ground_truth/york_urban_gt.csv`.

### Wireframe Dataset

Large-scale wireframe annotation dataset for structural understanding.

- **Source:** [github.com/huangkuns/wireframe](https://github.com/huangkuns/wireframe) (Huang et al., CVPR 2018)
- **Images:** 5,462 images (462 test + 5,000 train)
- **Ground truth:** 34,287 annotated line segments

**Setup:**

```bash
# Test split only (462 images, recommended for evaluation)
./tools/scripts/setup_wireframe.sh

# Or specific split
./tools/scripts/setup_wireframe.sh --split test    # 462 images
./tools/scripts/setup_wireframe.sh --split train   # 5,000 images
./tools/scripts/setup_wireframe.sh --split all     # 5,462 images

# Manual download fallback (if auto-download fails)
./tools/scripts/setup_wireframe.sh --file /path/to/wireframe.tar.gz
```

### Middlebury Stereo (MDB)

Multi-resolution stereo pairs for stereo evaluation.

- **Source:** [Middlebury Stereo Evaluation](https://vision.middlebury.edu/stereo/)
- **Resolutions:** Quarter (Q), Half (H), Full (F)
- **Setup:** `./tools/scripts/setup_mdb_dataset.sh`

### Noise Test Images

Synthetic images with controlled Gaussian noise levels (σ = 10, 20, 30, 40, 50) for noise robustness evaluation:

```
noise/
├── bike.png               # Clean
├── bike_noise10.png       # σ=10
├── bike_noise20.png       # σ=20
├── ...
├── circle.png             # Clean
├── circle_noise10.png     # σ=10
├── ...
├── im0_gray.png           # Clean stereo pair
└── im0_noise_gray.png     # Noisy stereo pair
```

## Ground Truth Format

All ground truth files use a unified CSV format compatible with `GroundTruthLoader`:

```csv
image_name,x1,y1,x2,y2
photo.jpg,50.89,20.23,299.47,20.23
photo.jpg,299.47,20.23,209.71,160.07
```

| Column | Type | Description |
|--------|------|-------------|
| `image_name` | string | Source image filename (basename only) |
| `x1`, `y1` | float | Start point coordinates (pixels, sub-pixel precision) |
| `x2`, `y2` | float | End point coordinates (pixels, sub-pixel precision) |

Multiple segments per image are represented as multiple rows with the same `image_name`.

The legacy `.txt` format uses the same columns without a header row and without the `image_name` column (one file per image).

### Using Ground Truth in C++

```cpp
#include <algorithm/ground_truth.hpp>

// Load from CSV
auto entries = lsfm::GroundTruthLoader::load_csv("york_urban_gt.csv");

// Each entry: { image_name, vector<LineSegment<double>> }
for (const auto& entry : entries) {
    std::cout << entry.image_name << ": "
              << entry.segments.size() << " segments\n";
}
```

### Using Ground Truth in Python

```python
import le_algorithm as alg

entries = alg.GroundTruthLoader.load_csv("york_urban_gt.csv")
for entry in entries:
    print(f"{entry.image_name}: {len(entry.segments)} segments")
```

## Presets

Optimized detector parameter presets in `presets/lsd_presets.json`.

### What Are Presets?

Presets are pre-computed optimal parameter configurations for all 9 LSD detectors, generated by running automated parameter search over ground truth datasets. Each detector has three profiles:

| Profile | Optimization Target | Description |
|---------|---------------------|-------------|
| **Fast** | Precision | Fewer but more accurate detections |
| **Balanced** | F1 score | Best trade-off between precision and recall |
| **Accurate** | Recall | Maximum detection coverage |

### Preset File Structure

```json
{
  "metadata": {
    "ground_truth_files": ["resources/datasets/ground_truth/york_urban_gt.csv"],
    "num_images": 102,
    "num_gt_segments": 12122,
    "num_samples": 10,
    "match_threshold": 5.0,
    "seed": 42
  },
  "detectors": {
    "LsdCC": {
      "fast":     { "params": { ... }, "score": 0.094 },
      "balanced": { "params": { ... }, "score": 0.140 },
      "accurate": { "params": { ... }, "score": 0.491 }
    },
    ...
  }
}
```

### Generating New Presets

Use the preset optimization tool to generate presets for your own datasets:

```bash
# Activate Python environment
source .venv/bin/activate

# Generate presets using York Urban ground truth
python evaluation/python/tools/optimize_presets.py \
    --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \
    --image-dir resources/datasets/YorkUrban/images \
    --output resources/presets/lsd_presets.json \
    --samples 300

# Optimize specific detectors only
python evaluation/python/tools/optimize_presets.py \
    --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \
    --image-dir resources/datasets/YorkUrban/images \
    --detectors LsdCC LsdFGioi LsdEDLZ \
    --samples 100
```

## Adding New Datasets

To add a new image dataset for evaluation:

1. **Place images** in `resources/datasets/<DatasetName>/images/`

2. **Create ground truth annotations** in CSV format:

   ```csv
   image_name,x1,y1,x2,y2
   img001.jpg,10.0,20.0,100.0,20.0
   img001.jpg,50.0,10.0,50.0,90.0
   ```

3. **Convert existing annotations** (if available in other formats):

   ```bash
   python tools/scripts/convert_ground_truth.py \
       --format <york_urban|wireframe> \
       --input /path/to/annotations \
       --output resources/datasets/ground_truth/my_dataset_gt.csv
   ```

4. **Add a Bazel filegroup** in `resources/datasets/BUILD.bazel`:

   ```python
   filegroup(
       name = "my_dataset",
       srcs = glob(["MyDataset/images/*.{jpg,png}"]),
       visibility = ["//visibility:public"],
   )
   ```

5. **Run preset optimization** on the new dataset (optional):

   ```bash
   python evaluation/python/tools/optimize_presets.py \
       --ground-truth resources/datasets/ground_truth/my_dataset_gt.csv \
       --image-dir resources/datasets/MyDataset/images
   ```

### Where to Get More Data

| Dataset | Images | Description | URL |
|---------|--------|-------------|-----|
| York Urban | 102 | Urban scenes with line annotations | [elderlab.yorku.ca](https://www.elderlab.yorku.ca/resources/) |
| Wireframe | 5,462 | Indoor/outdoor wireframe annotations | [github.com/huangkuns/wireframe](https://github.com/huangkuns/wireframe) |
| BSDS500 | 500 | Natural images with segmentation | [github.com/BIDS/BSDS500](https://github.com/BIDS/BSDS500) |
| Middlebury | varies | Stereo pairs with depth ground truth | [vision.middlebury.edu/stereo](https://vision.middlebury.edu/stereo/) |
| ETH3D | varies | High-res multi-view stereo | [eth3d.net](https://www.eth3d.net/) |
| ScanNet | 1,513 scenes | Indoor RGB-D scans | [scannet.org](http://www.scannet.org/) |
| Holicity | 6,500 | City-scale holistic 3D structure | [holicity.io](https://holicity.io/) |

## Bazel Targets

| Target | Contents |
|--------|----------|
| `//resources:windmill` | Default test image |
| `//resources:example_lines` | Synthetic test image |
| `//resources:example_challenge` | Challenge test image |
| `//resources:presets` | Detector parameter presets JSON |
| `//resources/datasets:bsds500` | BSDS500 (auto-downloaded) |
| `//resources/datasets:york_urban` | York Urban images |
| `//resources/datasets:wireframe` | Wireframe images |
| `//resources/datasets:mdb_q` / `:mdb_h` / `:mdb_f` | Middlebury stereo |
| `//resources/datasets:ground_truth` | All GT CSV/TXT files |
| `//resources/datasets:noise` | Noise test images |
| `//resources/datasets:all` | All datasets combined |

## See Also

- [Algorithm Library](../libs/algorithm/README.md) — `GroundTruthLoader`, `AccuracyMeasure`, `ParamOptimizer`
- [Evaluation](../evaluation/README.md) — Performance benchmarks using these datasets
- [Examples](../examples/README.md) — Standalone programs using test images
- [Jupyter Tutorials](../examples/notebooks/) — Interactive Python tutorials
