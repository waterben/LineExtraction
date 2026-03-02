/// @file image_operator_binding.cpp
/// @brief pybind11 bindings for ImageOperator hierarchy.
///
/// Binds the image preprocessing operators: geometric transforms (rotate,
/// scale, translate, affine, perspective), filtering (blur, Gaussian, median,
/// bilateral, FastNlMeans), noise generation, resize, no-op, and pipeline.
///
/// Templated operators (Rotate, Scale, Translate, Affine, Perspective) are
/// instantiated for float and double.

#include "filter_binding.hpp"
#include <cvnp/cvnp.h>
#include <imgproc/image_operator.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Trampoline for Python subclassing of ImageOperator
// ============================================================================

class PyImageOperator : public ImageOperator {
 public:
  /// Expose the protected base constructor so pybind11 can create instances.
  explicit PyImageOperator(const std::string& name) : ImageOperator(name) {}

  void apply(cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, ImageOperator, apply, img); }
};

// ============================================================================
// ImageOperator base + non-templated operators
// ============================================================================

void bind_image_operators(py::module_& m) {
  // --- ImageOperator base ---
  py::class_<ImageOperator, PyImageOperator, std::shared_ptr<ImageOperator>>(
      m, "ImageOperator",
      "Abstract base for image processing operators.\n\n"
      "Subclass and override apply(img) to create custom operators.\n"
      "Operators can be chained in a PipelineOperator.")
      .def(py::init<const std::string&>(), py::arg("name"), "Construct with operator name.")
      .def("name", &ImageOperator::name, "Get operator name.")
      .def("apply", static_cast<void (ImageOperator::*)(cv::Mat&)>(&ImageOperator::apply), py::arg("img"),
           "Apply operator to image in-place.")
      .def(
          "apply_copy",
          [](ImageOperator& op, const cv::Mat& in) {
            cv::Mat out;
            op.apply(in, out);
            return out;
          },
          py::arg("img"), "Apply operator to a copy of the image, return result.");

  // --- PipelineOperator ---
  py::class_<PipelineOperator, ImageOperator, std::shared_ptr<PipelineOperator>>(
      m, "PipelineOperator",
      "Chain multiple image operators into a pipeline.\n\n"
      "Operators are applied in order of addition.")
      .def(py::init<>(), "Construct an empty pipeline.")
      .def("push", static_cast<void (PipelineOperator::*)(const ImageOperatorPtr&)>(&PipelineOperator::push),
           py::arg("op"), py::keep_alive<1, 2>(), "Add an operator (shared_ptr) to the pipeline.")
      .def("pop", &PipelineOperator::pop, "Remove the last operator.")
      .def("clear", &PipelineOperator::clear, "Clear all operators.")
      .def("apply", static_cast<void (PipelineOperator::*)(cv::Mat&)>(&PipelineOperator::apply), py::arg("img"),
           "Apply all operators in sequence.")
      .def_static("create", &PipelineOperator::create, "Factory: create a pipeline operator.");

  // --- NoOp ---
  py::class_<NoOp, ImageOperator, std::shared_ptr<NoOp>>(m, "NoOp",
                                                         "No-op placeholder operator (passes image through unchanged).")
      .def(py::init<>())
      .def("apply", static_cast<void (NoOp::*)(cv::Mat&)>(&NoOp::apply), py::arg("img"))
      .def_static("create", &NoOp::create);

  // --- ResizeOperator ---
  py::class_<ResizeOperator, ImageOperator, std::shared_ptr<ResizeOperator>>(m, "ResizeOperator",
                                                                             "Resize image to specified dimensions.")
      .def(py::init<int, int, int>(), py::arg("width"), py::arg("height"),
           py::arg("interpolation") = static_cast<int>(cv::INTER_LINEAR), "Construct resize operator.")
      .def("apply", static_cast<void (ResizeOperator::*)(cv::Mat&)>(&ResizeOperator::apply), py::arg("img"))
      .def_static("create", &ResizeOperator::create, py::arg("width"), py::arg("height"),
                  py::arg("interpolation") = static_cast<int>(cv::INTER_LINEAR));

  // --- BlurOperator ---
  py::class_<BlurOperator, ImageOperator, std::shared_ptr<BlurOperator>>(m, "BlurOperator",
                                                                         "Box blur (uniform averaging) filter.")
      .def(py::init<int>(), py::arg("kernel_size"), "Construct with kernel size.")
      .def("apply", static_cast<void (BlurOperator::*)(cv::Mat&)>(&BlurOperator::apply), py::arg("img"))
      .def_static("create", &BlurOperator::create, py::arg("kernel_size"));

  // --- GaussianBlurOperator ---
  py::class_<GaussianBlurOperator, ImageOperator, std::shared_ptr<GaussianBlurOperator>>(
      m, "GaussianBlurOperator", "Gaussian blur filter.\n\nConstruct with sigma or kernel size.")
      .def(py::init([](double sigma, int kw, int kh) {
             return std::make_shared<GaussianBlurOperator>(sigma, cv::Size(kw, kh));
           }),
           py::arg("sigma"), py::arg("ksize_w") = 0, py::arg("ksize_h") = 0,
           "Construct with Gaussian sigma and optional kernel size (w, h).")
      .def(py::init<int>(), py::arg("kernel_size"), "Construct with explicit kernel size (sigma auto-computed).")
      .def("apply", static_cast<void (GaussianBlurOperator::*)(cv::Mat&)>(&GaussianBlurOperator::apply),
           py::arg("img"));

  // --- MedianBlurOperator ---
  py::class_<MedianBlurOperator, ImageOperator, std::shared_ptr<MedianBlurOperator>>(
      m, "MedianBlurOperator", "Median blur for salt-and-pepper noise removal.")
      .def(py::init<int>(), py::arg("ksize") = 3, "Construct with kernel size (must be odd).")
      .def("apply", static_cast<void (MedianBlurOperator::*)(cv::Mat&)>(&MedianBlurOperator::apply), py::arg("img"))
      .def_static("create", &MedianBlurOperator::create, py::arg("ksize") = 3);

  // --- BilateralOperator ---
  py::class_<BilateralOperator, ImageOperator, std::shared_ptr<BilateralOperator>>(
      m, "BilateralOperator",
      "Edge-preserving bilateral filter.\n\n"
      "Smooths while keeping edges sharp using spatial and color sigma.")
      .def(py::init<int, double, double>(), py::arg("d") = 5, py::arg("sigma_color") = 50.0,
           py::arg("sigma_space") = 50.0, "Construct bilateral filter operator.")
      .def("apply", static_cast<void (BilateralOperator::*)(cv::Mat&)>(&BilateralOperator::apply), py::arg("img"))
      .def_static("create", &BilateralOperator::create, py::arg("d") = 5, py::arg("sigma_color") = 50.0,
                  py::arg("sigma_space") = 50.0);

#ifdef HAVE_OPENCV_PHOTO
  // --- FastNlMeansOperator ---
  py::class_<FastNlMeansOperator, ImageOperator, std::shared_ptr<FastNlMeansOperator>>(
      m, "FastNlMeansOperator",
      "Fast Non-Local Means Denoising filter.\n\n"
      "Advanced patch-based denoising preserving texture and detail.\n"
      "Requires OpenCV photo module.")
      .def(py::init<float, int, int>(), py::arg("h") = 3.0f, py::arg("template_window_size") = 7,
           py::arg("search_window_size") = 21, "Construct NL-Means operator.")
      .def("apply", static_cast<void (FastNlMeansOperator::*)(cv::Mat&)>(&FastNlMeansOperator::apply), py::arg("img"))
      .def_static("create", &FastNlMeansOperator::create, py::arg("h") = 3.0f, py::arg("template_window_size") = 7,
                  py::arg("search_window_size") = 21);
#endif

  // --- UniformNoiseOperator ---
  py::class_<UniformNoiseOperator, ImageOperator, std::shared_ptr<UniformNoiseOperator>>(
      m, "UniformNoiseOperator", "Add uniform random noise U[lower, upper] to image.")
      .def(py::init<double, double>(), py::arg("lower"), py::arg("upper"), "Construct with noise range [lower, upper].")
      .def("apply", static_cast<void (UniformNoiseOperator::*)(cv::Mat&)>(&UniformNoiseOperator::apply), py::arg("img"))
      .def_static("create", &UniformNoiseOperator::create, py::arg("lower"), py::arg("upper"));

  // --- GaussianNoiseOperator ---
  py::class_<GaussianNoiseOperator, ImageOperator, std::shared_ptr<GaussianNoiseOperator>>(
      m, "GaussianNoiseOperator", "Add Gaussian noise N(mean, sigma^2) to image.")
      .def(py::init<double, double>(), py::arg("sigma"), py::arg("mean") = 0.0,
           "Construct with sigma and optional mean.")
      .def("apply", static_cast<void (GaussianNoiseOperator::*)(cv::Mat&)>(&GaussianNoiseOperator::apply),
           py::arg("img"))
      .def_static("create", &GaussianNoiseOperator::create, py::arg("sigma"), py::arg("mean") = 0.0);
}

// ============================================================================
// Templated geometric operators (float + double)
// ============================================================================

template <class T>
void bind_geometric_operators(py::module_& m, const std::string& suffix) {
  // --- RotateOperator ---
  py::class_<RotateOperator<T>, ImageOperator, std::shared_ptr<RotateOperator<T>>>(
      m, ("RotateOperator" + suffix).c_str(), ("Rotate image around a pivot point" + suffix + ".").c_str())
      .def(py::init([](T angle, T px, T py_val, int interp) {
             return std::make_shared<RotateOperator<T>>(angle, cv::Point_<T>(px, py_val), interp);
           }),
           py::arg("angle"), py::arg("pivot_x") = static_cast<T>(0), py::arg("pivot_y") = static_cast<T>(0),
           py::arg("interpolation") = static_cast<int>(cv::INTER_LINEAR),
           "Construct rotation (angle in radians, pivot at (x, y)).")
      .def("apply", static_cast<void (RotateOperator<T>::*)(cv::Mat&)>(&RotateOperator<T>::apply), py::arg("img"));

  // --- ScaleOperator ---
  py::class_<ScaleOperator<T>, ImageOperator, std::shared_ptr<ScaleOperator<T>>>(
      m, ("ScaleOperator" + suffix).c_str(), ("Scale image around a pivot point" + suffix + ".").c_str())
      .def(py::init([](T scale, T px, T py_val, int interp) {
             return std::make_shared<ScaleOperator<T>>(scale, cv::Point_<T>(px, py_val), interp);
           }),
           py::arg("scale"), py::arg("pivot_x") = static_cast<T>(0), py::arg("pivot_y") = static_cast<T>(0),
           py::arg("interpolation") = static_cast<int>(cv::INTER_LINEAR),
           "Construct uniform scale around pivot (x, y).")
      .def("apply", static_cast<void (ScaleOperator<T>::*)(cv::Mat&)>(&ScaleOperator<T>::apply), py::arg("img"));

  // --- TranslateOperator ---
  py::class_<TranslateOperator<T>, ImageOperator, std::shared_ptr<TranslateOperator<T>>>(
      m, ("TranslateOperator" + suffix).c_str(), ("Translate (shift) image by offset" + suffix + ".").c_str())
      .def(py::init([](T dx, T dy, int interp) {
             return std::make_shared<TranslateOperator<T>>(cv::Point_<T>(dx, dy), interp);
           }),
           py::arg("dx"), py::arg("dy"), py::arg("interpolation") = static_cast<int>(cv::INTER_LINEAR),
           "Construct translation operator.")
      .def("apply", static_cast<void (TranslateOperator<T>::*)(cv::Mat&)>(&TranslateOperator<T>::apply),
           py::arg("img"));
}

// --- Explicit template instantiations ---
template void bind_geometric_operators<float>(py::module_&, const std::string&);
template void bind_geometric_operators<double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
