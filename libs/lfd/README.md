# Line Feature Detector (LFD) Library

Comprehensive feature descriptor and matching library for line and point features with support for multiple visual tasks including stereo matching, motion estimation, and descriptor computation.

## Overview

The `lfd` library provides a unified framework for:

- **Line feature descriptors**: LBD (Line Band Descriptor), LR (Left-Right), generic descriptors
- **Feature matching**: Line and point feature matching across images
- **Stereo vision**: Line and point matching for stereo pairs
- **Motion estimation**: Feature matching for motion/optical flow
- **Filtering**: Geometric and optical flow constraints for feature validation

All code resides in the `lsfm` namespace.

## Key Components

### Feature Descriptors

#### Line-Based Descriptors

**LBD (Line Band Descriptor)** (`FeatureDescriptorLBD.hpp`):

- SIFT-like descriptor for line features
- Multiple band configurations
- Rotation normalization support
- Template-based for flexible types

**LR Descriptor** (`LRDescriptor.hpp`):

- Left-Right symmetric descriptor
- For line matching verification
- Reduced computational cost
- Good for stereo validation

**Generic Descriptors** (`GenericDescriptor.hpp`):

- Flexible descriptor framework
- Custom feature extraction
- Multiple distance metrics
- Type-agnostic design

#### Point Feature Descriptors

**Feature Descriptors** (`FeatureDescriptor.hpp`):

- Standard point feature descriptors
- Compatible with keypoint detectors
- OpenCV integration

### Feature Matching

#### Line Matching

**Line Matcher** (`lineMatching.hpp`):

```cpp
// Basic line-to-line matching
class LineMatching;

// Features:
// - Descriptor distance-based matching
// - Bidirectional verification
// - Confidence scoring
// - Multiple match strategies
```

**Pairwise Line Matcher** (`PairwiseLineMatcher.hpp`):

- Matches between two images
- Geometric constraint support
- Distance ratio testing
- Cross-checking

**Feature Matcher** (`FeatureMatcher.hpp`):

- Generic matcher for any feature type
- Configurable distance metrics
- Match filtering and validation

#### Point Matching

**Point Matchers** (Various):

- Stereo point matching
- Motion point matching
- Pairwise matching strategies

### Filtering & Constraints

#### Line Filtering

**Stereo Line Filter** (`StereoLineFilter.hpp`):

- Epipolar constraint validation
- Stereo line compatibility checking
- Disparity consistency

**Motion Line Filter** (`MotionLineFilter.hpp`):

- Optical flow constraints
- Motion consistency validation
- Temporal tracking support

**Global Rotation Filter** (`GlobalRotationFilter.hpp`):

- Global rotation estimation
- Outlier rejection
- Consistency checking

#### Point Filtering

**Stereo Point Filter** (`StereoPointFilter.hpp`):

- Epipolar constraint for points
- Stereo consistency

**Motion Point Filter** (`MotionPointFilter.hpp`):

- Optical flow consistency
- Motion validation

### Feature Tools

**Feature Tools** (`FeatureTools.hpp`):

- Utility functions for feature processing
- Descriptor computation helpers
- Match analysis and statistics

**Feature Filter Base** (`FeatureFilter.hpp`):

- Abstract filter interface
- Common filtering operations

### Matcher Implementations

**Stereo Matchers** (`StereoLineMatcher.hpp`, `StereoPointMatcher.hpp`):

```cpp
class StereoLineMatcher {
  // Match lines between stereo pairs
  // Enforces epipolar constraints
  // Returns validated correspondences
};
```

**Motion Matchers** (`MotionLineMatcher.hpp`, `MotionPointMatcher.hpp`):

```cpp
class MotionLineMatcher {
  // Match lines in sequential frames
  // Enforces motion consistency
  // Supports optical flow constraints
};
```

**Motion Descriptors** (`MotionDescriptor.hpp`):

- Temporal feature descriptors
- Motion-aware matching
- Sequence analysis support

## Architecture

### Design Patterns

1. **Template-Based Architecture**
   - Flexible type support (float, double)
   - Compile-time specialization
   - Header-only implementation

2. **Matcher Interface**
   - Unified matching framework
   - Pluggable filters and constraints
   - Configurable via ValueManager

3. **Descriptor Framework**
   - Generic descriptor representation
   - Multiple distance metrics
   - Efficient comparison

### Class Hierarchy

```
FeatureDescriptor (base)
  ├─ FeatureDescriptorLBD (line-specific)
  ├─ LRDescriptor (symmetric)
  └─ GenericDescriptor (extensible)

FeatureMatcher (base matching)
  ├─ LineMatching (line-specific)
  ├─ PairwiseLineMatcher (stereo/pairwise)
  └─ StereoLineMatcher (epipolar-constrained)

FeatureFilter (base filtering)
  ├─ StereoLineFilter (stereo constraints)
  └─ MotionLineFilter (motion constraints)
```

## Usage Examples

### Basic Line Matching

```cpp
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/PairwiseLineMatcher.hpp>

// Compute descriptors
auto desc1 = computeLBD(lines1, image1);
auto desc2 = computeLBD(lines2, image2);

// Match
PairwiseLineMatcher matcher;
auto matches = matcher.match(desc1, desc2);
```

### Stereo Line Matching

```cpp
#include <lfd/StereoLineMatcher.hpp>
#include <lfd/StereoLineFilter.hpp>

// Create matchers with epipolar constraint
StereoLineMatcher stereo_matcher;
stereo_matcher.setEssentialMatrix(E);  // Epipolar geometry

// Match and filter
auto stereo_matches = stereo_matcher.matchStereo(lines_left, lines_right);
```

### Motion-Based Matching

```cpp
#include <lfd/MotionLineMatcher.hpp>

// Create motion matcher
MotionLineMatcher motion_matcher;
motion_matcher.setOpticalFlowField(flow);

// Match consecutive frames
auto motion_matches = motion_matcher.matchMotion(lines_frame1, lines_frame2);
```

## Type Support

The library supports multiple template instantiations:

```cpp
// Single precision
lsfm::FeatureDescriptorLBD<float> desc_f;
lsfm::StereoLineMatcher<float> matcher_f;

// Double precision
lsfm::FeatureDescriptorLBD<double> desc_d;
lsfm::StereoLineMatcher<double> matcher_d;

// Custom types
lsfm::GenericDescriptor<CustomType> custom_desc;
```

## Configuration

Most components support configuration via the ValueManager interface:

```cpp
// Example configurations
matcher.valueMinDistance(Value(0.5));  // Minimum match distance
matcher.valueMaxRatio(Value(0.8));     // Distance ratio threshold
filter.valueEpipolarThreshold(Value(1.0));  // Epipolar constraint tolerance
```

## Dependencies

- **OpenCV**: cv::Mat, cv::Point, etc. for image data
- **Geometry library**: Vec2, Vec3, Line, LineSegment types
- **Utility library**: ValueManager for configuration

## Performance Considerations

1. **Descriptor Computation**: Compute once, reuse for multiple matchings
2. **Match Filtering**: Apply geometric constraints early for pruning
3. **Memory**: Descriptors stored efficiently as packed binary or floating-point
4. **Parallelization**: Matching loop can be parallelized for large feature sets

## See Also

- [Edge Detection Library](../edge/README.md) - For line feature extraction
- [LSD Library](../lsd/README.md) - For line segment detection
- [Geometry Library](../geometry/README.md) - For geometric primitives
