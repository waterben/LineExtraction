# Evaluation and Performance Benchmarking Library

Framework for evaluating line and edge detection algorithms with comprehensive performance metrics, ground truth comparison, and statistical analysis tools.

## Overview

The `eval` library provides infrastructure for:

- **Algorithm evaluation**: Comparing detection results against ground truth
- **Performance measurement**: Precision, recall, F-measure, timing
- **Data management**: Loading, storing, and organizing benchmark datasets
- **Task execution**: Standardized evaluation workflows
- **Statistical analysis**: Results aggregation and reporting

All code resides in the `lsfm` namespace.

## Key Components

### Core Evaluation Framework

#### Performance Measures

**Performance Measure** (`performance_measure.hpp`):

```cpp
class PerformanceMeasure {
  // Compute metrics for detection evaluation
  // - Precision: TP / (TP + FP)
  // - Recall: TP / (TP + FN)
  // - F-measure: Harmonic mean of precision and recall
  // - Timing: Algorithm execution time
};
```

**Features:**

- Per-image metrics computation
- Aggregate statistics
- Threshold-dependent evaluation
- Statistical confidence intervals

#### OpenCV Integration

**CV Measure** (`cv_measure.hpp`):

- OpenCV matrix-based measurements
- Image correlation metrics
- Morphological analysis
- Pixel-level comparisons

### Data Management

#### Data Provider Framework

**Base Data Provider** (`data_provider.hpp`):

```cpp
class DataProvider {
  virtual const cv::Mat& getImage(size_t id) = 0;
  virtual const LineSegmentVector& getGroundTruth(size_t id) = 0;
  virtual const ImageData& getImageData(size_t id) = 0;
};
```

**CV Data Provider** (`cv_data_provider.hpp`):

- OpenCV-based image loading
- Directory scanning
- Format conversion
- Caching support

#### Results Management

**Results** (`results.hpp`):

```cpp
struct Results {
  // Per-image results
  std::vector<PerformanceMeasure> metrics;

  // Aggregate statistics
  double mean_precision;
  double mean_recall;
  double mean_f_measure;
  double total_time;
};
```

**Features:**

- Structured result storage
- Serialization support
- Result aggregation
- Statistical summaries

### Task Framework

#### Input Task

**Input Task** (`input_task.hpp`):

- Load image data from dataset
- Manage dataset iteration
- Handle preprocessing
- Configure data sources

#### Measurement Task

**Measure Task** (`measure_task.hpp`):

```cpp
class MeasureTask {
  virtual void execute(
    const cv::Mat& image,
    const LineSegmentVector& ground_truth,
    Results& results) = 0;
};
```

**Features:**

- Run detection algorithm
- Compare with ground truth
- Compute metrics
- Store intermediate results

#### Application Framework

**Evaluation Application** (`eval_app.hpp`):

```cpp
class EvalApp {
  // Complete evaluation workflow
  // - Load data
  // - Run detection
  // - Compute metrics
  // - Store results
  // - Generate reports
};
```

### Utility Components

#### Run Files Management

**Run Files** (`runfiles.hpp`):

- Store evaluation configurations
- Manage result files
- Track algorithm parameters
- Version control

#### Generic Measures

**Measure** (`measure.hpp`):

- Generic metric computation
- Extensible measure interface
- Multiple distance metrics

## Architecture

### Evaluation Workflow

```
1. Input Task
   ├─ Load dataset
   ├─ Iterate images
   └─ Prepare data

2. Detection Algorithm
   ├─ Run detector
   └─ Extract features

3. Measurement Task
   ├─ Compare with ground truth
   ├─ Compute metrics
   └─ Store results

4. Results Aggregation
   ├─ Compute statistics
   ├─ Generate reports
   └─ Export data
```

### Design Patterns

1. **Task Pattern**
   - Encapsulated evaluation steps
   - Composable tasks
   - Configuration management

2. **Provider Pattern**
   - Data abstraction
   - Format independence
   - Extensible data sources

3. **Measurement Framework**
   - Flexible metric definition
   - Statistical analysis
   - Threshold analysis

## Core Classes

### Hierarchies

```
DataProvider (abstract)
  └─ CVDataProvider (OpenCV-based)

Measure (abstract)
  ├─ PerformanceMeasure
  ├─ CVMeasure
  └─ Custom measures

Task (abstract)
  ├─ InputTask
  ├─ MeasureTask
  └─ Custom tasks

EvalApp (application)
  ├─ Coordinate tasks
  ├─ Manage workflow
  └─ Generate results
```

## Usage Examples

### Basic Evaluation

```cpp
#include <eval/eval_app.hpp>
#include <eval/cv_data_provider.hpp>

// Create data provider
auto provider = std::make_unique<CVDataProvider>("dataset/");

// Create evaluation app
EvalApp evaluator;
evaluator.setDataProvider(std::move(provider));

// Run evaluation
Results results = evaluator.evaluate();

// Print statistics
std::cout << "Mean Precision: " << results.mean_precision << std::endl;
std::cout << "Mean Recall: " << results.mean_recall << std::endl;
std::cout << "Mean F-measure: " << results.mean_f_measure << std::endl;
```

### Custom Measurement Task

```cpp
#include <eval/measure_task.hpp>

class CustomMeasureTask : public MeasureTask {
  void execute(const cv::Mat& image,
               const LineSegmentVector& ground_truth,
               Results& results) override {
    // Run custom detection
    auto detected = myDetector.detect(image);

    // Compute metrics
    auto metrics = PerformanceMeasure::compute(
      detected, ground_truth);

    results.metrics.push_back(metrics);
  }
};
```

### Batch Evaluation

```cpp
// Evaluate multiple algorithms
for (const auto& algo : algorithms) {
  EvalApp evaluator;
  evaluator.setDetector(algo);

  Results results = evaluator.evaluate();
  report(algo.name(), results);
}
```

## Metrics

### Standard Performance Metrics

- **Precision**: True Positives / (True Positives + False Positives)
- **Recall**: True Positives / (True Positives + False Negatives)
- **F-measure**: 2 *(Precision* Recall) / (Precision + Recall)

### Computational Metrics

- **Execution Time**: Algorithm runtime in milliseconds
- **Memory Usage**: Peak memory consumption (optional)
- **Throughput**: Images processed per second

### Matching Criteria

- **Distance threshold**: Maximum allowed distance for match
- **Overlap ratio**: Minimum overlap for line segments
- **Angular tolerance**: Maximum angle difference

## Dataset Format

Expected directory structure:

```
dataset/
├─ images/
│  ├─ image_001.png
│  ├─ image_002.png
│  └─ ...
└─ ground_truth/
   ├─ image_001_gt.txt
   ├─ image_002_gt.txt
   └─ ...
```

Ground truth format: Line segments as (x1, y1, x2, y2) per line

## Configuration

Most components are configurable via ValueManager:

```cpp
// Example configuration
evaluator.valueMinOverlap(Value(0.5));
evaluator.valueDistanceThreshold(Value(1.0));
evaluator.valueMatchingStrategy(Value("greedy"));
```

## Extensibility

The library is designed for extension:

1. **Custom Measures**: Implement `Measure` interface
2. **Custom Tasks**: Implement `Task` or specific task types
3. **Custom Providers**: Implement `DataProvider` interface
4. **Custom Applications**: Extend `EvalApp`

## Dependencies

- **OpenCV**: Image loading, matrix operations, display
- **Geometry library**: Line, LineSegment types
- **Edge library**: Edge detection components
- **LSD library**: Line segment detection
- **Utility library**: ValueManager, data structures

## Performance Considerations

1. **Caching**: Data providers cache loaded images
2. **Lazy Computation**: Metrics computed on demand
3. **Parallel Evaluation**: Image batches can be processed in parallel
4. **Memory Management**: Results aggregation without storing all images

## Integration with Other Libraries

- **With Edge Library**: Evaluate edge detection quality
- **With LSD Library**: Compare line segment detectors
- **With LFD Library**: Evaluate feature descriptor performance
- **Benchmark Data**: BSDS500, BSD, custom datasets

## See Also

- [Edge Detection Library](../edge/README.md) - Edge detection algorithms
- [LSD Library](../lsd/README.md) - Line segment detection methods
- [LFD Library](../lfd/README.md) - Feature matching and descriptors
- [Utility Library](../utility/README.md) - TestImages and core utilities
- [Evaluation Examples](../../evaluation/README.md) - Performance benchmarks
- [Main README](../../README.md) - Project overview
