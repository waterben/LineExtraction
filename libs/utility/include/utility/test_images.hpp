/// @file test_images.hpp
/// @brief Utility for resolving test image paths across Bazel and CMake builds.
///
/// This header provides a simple way to locate test images in examples and tests.
/// It supports both Bazel builds (via runfiles) and CMake builds (relative paths).
///
/// @code
/// #include <utility/test_images.hpp>
///
/// int main(int argc, char** argv) {
///     // Initialize once at program start
///     lsfm::TestImages::init(argv[0]);
///
///     // Get path to a test image
///     std::string path = lsfm::TestImages::get("windmill.jpg");
///     cv::Mat img = cv::imread(path);
///
///     // For stereo images
///     std::string left = lsfm::TestImages::stereoPair("Adirondack").first;
///     std::string right = lsfm::TestImages::stereoPair("Adirondack").second;
/// }
/// @endcode
#pragma once

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace lsfm {

/// @brief Utility class for resolving test image paths.
///
/// Searches for images in standard locations:
/// - Bazel: via runfiles (@bsds500, //resources/datasets)
/// - CMake: resources/datasets/, ../resources/datasets/, etc.
class TestImages {
 public:
  /// @brief Initialize the test image resolver.
  /// @param argv0 The argv[0] from main() - used for runfiles detection
  /// Call this once at program start before using get().
  static void init(const char* argv0) {
    instance().argv0_ = argv0 ? argv0 : "";
    instance().initialized_ = true;
    instance().detectSearchPaths();
  }

  /// @brief Get the full path to a test image.
  /// @param relative_path The relative path within datasets (e.g., "windmill.jpg", "noise/bike.png")
  /// @return The resolved absolute path, or empty string if not found
  static std::string get(const std::string& relative_path) {
    if (!instance().initialized_) {
      std::cerr << "Warning: TestImages::init() not called, using default paths" << std::endl;
      instance().detectSearchPaths();
      instance().initialized_ = true;
    }
    return instance().resolve(relative_path);
  }

  /// @brief Get path to the windmill test image (convenience).
  static std::string windmill() { return get("windmill.jpg"); }

  /// @brief Get path to an image in the noise dataset.
  /// @param name Image name (e.g., "bike.png", "circle.png")
  static std::string noise(const std::string& name) { return get("noise/" + name); }

  /// @brief Get path to a BSDS500 image.
  /// @param name Image name (e.g., "100007.jpg")
  /// @param split Dataset split: "train", "test", "val", or empty for flat structure
  static std::string bsds500(const std::string& name, const std::string& split = "") {
    if (!split.empty()) {
      return get("BSDS500/" + split + "/" + name);
    }
    return get("BSDS500/" + name);
  }

  /// @brief Get path to a MDB stereo pair (left/right images).
  /// @param scene Scene name (e.g., "Adirondack", "Bicycle1")
  /// @param resolution Resolution: "Q" (quarter), "H" (half), or "F" (full)
  /// @return Pair of paths (left, right) or empty strings if not found
  /// @note MDB dataset must be downloaded manually using setup_mdb_dataset.sh
  static std::pair<std::string, std::string> stereoPair(const std::string& scene, const std::string& resolution = "H") {
    std::string res_dir = "mdb_" + resolution;
    std::string left = get(res_dir + "/" + scene + "/im0.png");
    std::string right = get(res_dir + "/" + scene + "/im1.png");
    return {left, right};
  }

  /// @brief Get the left image of a MDB stereo pair.
  /// @param scene Scene name (e.g., "Adirondack")
  /// @param resolution Resolution: "Q", "H", or "F"
  static std::string stereoLeft(const std::string& scene, const std::string& resolution = "H") {
    return stereoPair(scene, resolution).first;
  }

  /// @brief Get the right image of a MDB stereo pair.
  /// @param scene Scene name (e.g., "Adirondack")
  /// @param resolution Resolution: "Q", "H", or "F"
  static std::string stereoRight(const std::string& scene, const std::string& resolution = "H") {
    return stereoPair(scene, resolution).second;
  }

  /// @brief Check if running under Bazel.
  static bool isBazelRun() {
    // Check for Bazel runfiles environment variables
    return std::getenv("RUNFILES_DIR") != nullptr || std::getenv("RUNFILES_MANIFEST_FILE") != nullptr;
  }

 private:
  TestImages() : argv0_{}, search_paths_{}, initialized_{false} {}
  static TestImages& instance() {
    static TestImages inst;
    return inst;
  }

  void detectSearchPaths() {
    namespace fs = std::filesystem;
    search_paths_.clear();

    if (isBazelRun()) {
      // Bazel: construct runfiles paths
      const char* runfiles_dir = std::getenv("RUNFILES_DIR");
      if (runfiles_dir) {
        // External BSDS500 repository
        search_paths_.push_back(std::string(runfiles_dir) + "/bsds500/BSDS500/data/images");
        // Local resources (windmill.jpg, etc.)
        search_paths_.push_back(std::string(runfiles_dir) + "/line_extraction/resources");
        // Local datasets (noise, MDB, etc.)
        search_paths_.push_back(std::string(runfiles_dir) + "/line_extraction/resources/datasets");
      }
    }

    // CMake / standalone: search relative to working directory and executable
    std::vector<std::string> relative_paths = {
        "resources",       "resources/datasets",       "../resources",       "../resources/datasets",
        "../../resources", "../../resources/datasets", "../../../resources", "../../../resources/datasets",
    };

    for (const auto& rel : relative_paths) {
      if (fs::exists(rel)) {
        search_paths_.push_back(fs::absolute(rel).string());
      }
    }

    // Also try from executable directory if argv0 is available
    if (!argv0_.empty()) {
      fs::path exe_dir = fs::path(argv0_).parent_path();
      if (!exe_dir.empty()) {
        for (const auto& rel : relative_paths) {
          fs::path candidate = exe_dir / rel;
          if (fs::exists(candidate)) {
            search_paths_.push_back(fs::absolute(candidate).string());
          }
        }
      }
    }
  }

  std::string resolve(const std::string& relative_path) const {
    namespace fs = std::filesystem;

    for (const auto& base : search_paths_) {
      fs::path full_path = fs::path(base) / relative_path;
      if (fs::exists(full_path)) {
        return full_path.string();
      }
    }

    // Not found - return the relative path as fallback (may work if cwd is correct)
    std::cerr << "Warning: Test image not found: " << relative_path << std::endl;
    std::cerr << "  Searched in:" << std::endl;
    for (const auto& p : search_paths_) {
      std::cerr << "    - " << p << std::endl;
    }
    return relative_path;
  }

  std::string argv0_;
  std::vector<std::string> search_paths_;
  bool initialized_;
};

}  // namespace lsfm
