/// @file rerun_demo.cpp
/// @brief Minimal Rerun C++ SDK demo for the LineExtraction project.
///
/// Logs a synthetic gradient image and animated 2D line segments to the Rerun
/// viewer, demonstrating how the C++ SDK integrates with the lsfm project.
///
/// Usage::
///   # Start the Rerun viewer first:
///   rerun
///
///   # Then build and run:
///   bazel run //examples/other:rerun_cpp_demo

#include <rerun/archetypes/image.hpp>
#include <rerun/archetypes/line_strips2d.hpp>
#include <rerun/archetypes/points2d.hpp>
#include <rerun/archetypes/text_log.hpp>

#include <cmath>
#include <cstdint>
#include <rerun.hpp>
#include <vector>

namespace {

/// Pi constant (avoid relying on POSIX M_PI extension)
static constexpr double kPi = 3.14159265358979323846;

/// Generate a flat row-major grayscale gradient image with a circular highlight.
/// @param width  Image width in pixels.
/// @param height Image height in pixels.
/// @return Flat uint8 pixel buffer (single channel, row-major).
std::vector<uint8_t> make_gradient_image(int width, int height) {
  std::vector<uint8_t> pixels(static_cast<size_t>(width * height));
  const double cx = width * 0.5;
  const double cy = height * 0.5;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double base = static_cast<double>(x) / width * 200.0;
      double r = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
      double highlight = std::max(0.0, 120.0 - r * 0.5);
      double val = std::min(255.0, base + highlight);
      pixels[static_cast<size_t>(y * width + x)] = static_cast<uint8_t>(val);
    }
  }
  return pixels;
}

/// Build a star pattern as a list of 2D line strips.
/// @param cx           Center X coordinate.
/// @param cy           Center Y coordinate.
/// @param radius       Outer radius.
/// @param angle_offset Rotation offset in radians.
/// @param n_rays       Number of rays.
/// @return Vector of LineStrip2D, one per ray.
std::vector<rerun::LineStrip2D> make_star_strips(
    float cx, float cy, float radius, double angle_offset, int n_rays = 12) {
  std::vector<rerun::LineStrip2D> strips;
  strips.reserve(static_cast<size_t>(n_rays));
  for (int i = 0; i < n_rays; ++i) {
    double angle = 2.0 * kPi * i / n_rays + angle_offset;
    float x_end = cx + radius * static_cast<float>(std::cos(angle));
    float y_end = cy + radius * static_cast<float>(std::sin(angle));
    strips.push_back(rerun::LineStrip2D{{rerun::Vec2D{cx, cy}, rerun::Vec2D{x_end, y_end}}});
  }
  return strips;
}

/// Build a regular grid of 2D positions (simulating keypoint detections).
/// @param width  Image width.
/// @param height Image height.
/// @param step   Grid step size in pixels.
/// @return Vector of Position2D.
std::vector<rerun::Position2D> make_grid_points(int width, int height, int step = 80) {
  std::vector<rerun::Position2D> pts;
  for (int y = step; y < height; y += step) {
    for (int x = step; x < width; x += step) {
      pts.push_back(rerun::Position2D{static_cast<float>(x), static_cast<float>(y)});
    }
  }
  return pts;
}

}  // namespace

int main() {
  // Connect to a running Rerun viewer (default gRPC port 9876).
  // Start the viewer with `rerun` before running this demo.
  // Alternatively use rec.spawn() to launch the viewer automatically.
  const auto rec = rerun::RecordingStream{"lsfm_cpp_demo"};
  rec.connect_grpc().exit_on_failure();

  constexpr int kWidth = 640;
  constexpr int kHeight = 480;
  constexpr int kFrames = 60;

  // ---- Static background (logged once at frame 0) ----
  rec.set_time_sequence("frame", 0);

  // Gradient image - demonstrates image logging with greyscale pixel buffer
  auto pixels = make_gradient_image(kWidth, kHeight);
  rec.log("image/gradient",
          rerun::archetypes::Image::from_grayscale8(
              pixels, rerun::WidthHeight{static_cast<uint32_t>(kWidth), static_cast<uint32_t>(kHeight)}));

  // Grid keypoints (static)
  rec.log("detections/keypoints", rerun::archetypes::Points2D(make_grid_points(kWidth, kHeight))
                                      .with_radii({3.0f})
                                      .with_colors({rerun::Color(50, 200, 100, 180)}));

  rec.log("log", rerun::archetypes::TextLog("C++ Rerun demo started - logging " + std::to_string(kFrames) + " frames")
                     .with_level(rerun::components::TextLogLevel::Info));

  // ---- Animated frames ----
  for (int frame = 0; frame < kFrames; ++frame) {
    rec.set_time_sequence("frame", frame);

    // Rotating star pattern (simulating animated line detection results)
    const double angle_offset = 2.0 * kPi * static_cast<double>(frame) / kFrames;

    rec.log("detections/lines",
            rerun::archetypes::LineStrips2D(make_star_strips(static_cast<float>(kWidth) * 0.5f,
                                                             static_cast<float>(kHeight) * 0.5f,
                                                             static_cast<float>(kHeight) * 0.4f, angle_offset))
                .with_colors({rerun::Color(255, 100, 50, 200)}));
  }

  rec.log("log",
          rerun::archetypes::TextLog("C++ Rerun demo finished").with_level(rerun::components::TextLogLevel::Info));

  return 0;
}
