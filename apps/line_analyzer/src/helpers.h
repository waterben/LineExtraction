#pragma once

#include "qplot/qcustomplot.h"
#include <imgproc/derivative_gradient.hpp>
#include <lsd/lsd_base.hpp>

#include <memory>
#include <vector>

typedef double float_type;

struct ImageMode {
  ImageMode(const QString& n = QString(), const QCPColorGradient& g = QCPColorGradient::gpGrayscale)
      : name(n), grad(g) {}

  QString name;
  QCPColorGradient grad;
};

typedef std::vector<ImageMode> ImageModes;

struct ImageSource {
  ImageSource(const QString& n = QString(),
              const cv::Mat& d = cv::Mat(),
              const ImageModes& m = ImageModes(),
              const QCPRange& r = QCPRange())
      : name(n), data(d), modes(m), range(r) {}

  QString name;
  cv::Mat data;
  ImageModes modes;
  QCPRange range;
};

typedef std::vector<ImageSource> ImageSources;

typedef std::shared_ptr<lsfm::LsdBase<float_type, cv::Point_>> LsdPtr;
typedef lsfm::LineSegment<float_type, cv::Point_> LineSegment;
typedef std::vector<LineSegment> LineSegmentVector;

constexpr int D_MAG_SQR = 1;
constexpr int D_COLOR_INPUT = 2;

struct Detector {
  Detector(const QString& n = QString(), int f = 0) : flags(f), name(n), lsd() {}
  virtual ~Detector() = default;
  Detector(const Detector&) = default;
  Detector& operator=(const Detector&) = default;

  int flags;
  QString name;
  LsdPtr lsd;

  //! create / reset internal lsd detector
  virtual void create() = 0;

  //! get lsd sources (image data with default modes)
  virtual ImageSources sources(const cv::Mat& src = cv::Mat());


  //! run lsd detect and return detected lines
  inline LineSegmentVector detect(const cv::Mat& src) {
    if (!lsd) return LineSegmentVector();
    lsd->detect(src);
    return lsd->lineSegments();
  }

  inline bool magSqr() const { return flags & D_MAG_SQR; }

  inline bool colorInput() const { return flags & D_COLOR_INPUT; }

  static ImageModes imageModePresets;
};

typedef cv::Ptr<Detector> DetectorPtr;
typedef std::vector<DetectorPtr> DetectorVector;

ImageModes segmentMapModes(const cv::Mat& data, QCPRange& r);
ImageModes& dirMapModes();

template <class LSD>
void gradSources(const LSD* lsd, ImageSources& src) {
  src.push_back(ImageSource("dir", lsd->grad().direction(), dirMapModes(),
                            QCPRange(lsd->grad().directionRange().lower, lsd->grad().directionRange().upper)));
  lsfm::FilterResults res = lsd->grad().results();
  for_each(res.begin(), res.end(), [&](const lsfm::FilterResult& r) {
    if (r.first == "dir" || r.first == "mag" || r.first == "gx" || r.first == "gy") return;
    if (r.first == "phase")
      src.push_back(ImageSource(r.first.c_str(), r.second.data, dirMapModes(),
                                QCPRange(r.second.range.lower, r.second.range.upper)));
    else
      src.push_back(ImageSource(r.first.c_str(), r.second.data, Detector::imageModePresets));
  });
}

template <class LSD>
void gradEdgeSources(const LSD* lsd, ImageSources& src) {
  src.push_back(
      ImageSource("dir", lsd->edgeSource().direction(), dirMapModes(),
                  QCPRange(lsd->edgeSource().directionRange().lower, lsd->edgeSource().directionRange().upper)));
  lsfm::FilterResults res = lsd->edgeSource().results();
  for_each(res.begin(), res.end(), [&](const lsfm::FilterResult& r) {
    if (r.first == "dir" || r.first == "mag" || r.first == "gx" || r.first == "gy") return;
    if (r.first == "phase")
      src.push_back(ImageSource(r.first.c_str(), r.second.data, dirMapModes(),
                                QCPRange(r.second.range.lower, r.second.range.upper)));
    else
      src.push_back(ImageSource(r.first.c_str(), r.second.data, Detector::imageModePresets));
  });
}

template <class LSD>
void segmentSources(const LSD* lsd, ImageSources& src) {
  if (lsd->imageData("segment_map").empty()) {
    lsfm::EdgeSegmentVector segs = lsd->segments();
    lsfm::IndexVector indexes = lsd->indexes();
    cv::Mat m(src[0].data.size(), CV_32S);
    m.setTo(-1);
    cv::RNG& rng = cv::theRNG();

    int id = 0;
    for_each(segs.begin(), segs.end(), [&](const lsfm::EdgeSegment& seg) {
      cv::Vec3b color(static_cast<uchar>(20 + rng.uniform(0, 225)), static_cast<uchar>(20 + rng.uniform(0, 225)),
                      static_cast<uchar>(20 + rng.uniform(0, 225)));
      for (size_t i = seg.begin(); i != seg.end(); ++i) lsfm::set<int>(m, indexes[i], id);
      ++id;
    });
    QCPRange r;
    src.push_back(ImageSource("segment_map", m, segmentMapModes(m, r)));
    src.back().range = r;
  }
}

template <class LSD>
void patternSources(const LSD* lsd, ImageSources& src) {
  if (lsd->imageData("pattern_map").empty()) {
    lsfm::EdgeSegmentVector segs = lsd->patterns();
    lsfm::IndexVector indexes = lsd->indexes();
    cv::Mat m(src[0].data.size(), CV_32S);
    m.setTo(-1);
    cv::RNG& rng = cv::theRNG();

    int id = 0;
    for_each(segs.begin(), segs.end(), [&](const lsfm::EdgeSegment& seg) {
      cv::Vec3b color(static_cast<uchar>(20 + rng.uniform(0, 225)), static_cast<uchar>(20 + rng.uniform(0, 225)),
                      static_cast<uchar>(20 + rng.uniform(0, 225)));
      for (size_t i = seg.begin(); i != seg.end(); ++i) lsfm::set<int>(m, indexes[i], id);
      ++id;
    });
    QCPRange r;
    src.push_back(ImageSource("pattern_map", m, segmentMapModes(m, r)));
    src.back().range = r;
  }
}

//! default lsd Detector object
//! Implement custom Detector to change default behavior
template <class LSD>
struct DetectorT : public Detector {
  DetectorT<LSD>(const QString& n = QString(), int f = 0) : Detector(n, f) {}

  void create() { this->lsd = LsdPtr(new LSD); }
};

template <class LSD>
struct DetectorG : public Detector {
  DetectorG<LSD>(const QString& n = QString(), int f = 0) : Detector(n, f) {}

  void create() { this->lsd = LsdPtr(new LSD); }

  ImageSources sources(const cv::Mat& src = cv::Mat()) {
    ImageSources ret = Detector::sources(src);
    gradSources<LSD>(dynamic_cast<const LSD*>(lsd.get()), ret);
    return ret;
  }
};

template <class LSD>
struct DetectorGS : public Detector {
  DetectorGS<LSD>(const QString& n = QString(), int f = 0) : Detector(n, f) {}

  void create() { this->lsd = LsdPtr(new LSD); }

  ImageSources sources(const cv::Mat& src = cv::Mat()) {
    ImageSources ret = Detector::sources(src);
    const LSD* ptr = dynamic_cast<const LSD*>(lsd.get());
    gradSources<LSD>(ptr, ret);
    segmentSources<LSD>(ptr, ret);
    return ret;
  }
};

template <class LSD>
struct DetectorGSP : public Detector {
  DetectorGSP<LSD>(const QString& n = QString(), int f = 0) : Detector(n, f) {}

  void create() { this->lsd = LsdPtr(new LSD); }

  ImageSources sources(const cv::Mat& src = cv::Mat()) {
    ImageSources ret = Detector::sources(src);
    const LSD* ptr = dynamic_cast<const LSD*>(lsd.get());
    gradSources<LSD>(ptr, ret);
    segmentSources<LSD>(ptr, ret);
    patternSources<LSD>(ptr, ret);
    return ret;
  }
};

template <class LSD>
struct DetectorE : public Detector {
  DetectorE<LSD>(const QString& n = QString(), int f = 0) : Detector(n, f) {}

  void create() { this->lsd = LsdPtr(new LSD); }

  ImageSources sources(const cv::Mat& src = cv::Mat()) {
    ImageSources ret = Detector::sources(src);
    const LSD* ptr = dynamic_cast<const LSD*>(lsd.get());
    gradEdgeSources<LSD>(ptr, ret);
    return ret;
  }
};

template <class LSD>
struct DetectorES : public Detector {
  DetectorES<LSD>(const QString& n = QString(), int f = 0) : Detector(n, f) {}

  void create() { this->lsd = LsdPtr(new LSD); }

  ImageSources sources(const cv::Mat& src = cv::Mat()) {
    ImageSources ret = Detector::sources(src);
    const LSD* ptr = dynamic_cast<const LSD*>(lsd.get());
    gradEdgeSources<LSD>(ptr, ret);
    segmentSources<LSD>(ptr, ret);
    return ret;
  }
};


template <class LSD>
struct DetectorESP : public Detector {
  DetectorESP<LSD>(const QString& n = QString(), int f = 0) : Detector(n, f) {}

  void create() { this->lsd = LsdPtr(new LSD); }

  ImageSources sources(const cv::Mat& src = cv::Mat()) {
    ImageSources ret = Detector::sources(src);
    const LSD* ptr = dynamic_cast<const LSD*>(lsd.get());
    gradEdgeSources<LSD>(ptr, ret);
    segmentSources<LSD>(ptr, ret);
    patternSources<LSD>(ptr, ret);
    return ret;
  }
};

//! no data
template <class LSD>
struct DetectorND : public Detector {
  DetectorND<LSD>(const QString& n = QString()) : Detector(n) {}

  void create() { this->lsd = LsdPtr(new LSD); }

  ImageSources sources(const cv::Mat& src = cv::Mat()) {
    ImageSources ret;
    ret.push_back(ImageSource("gx", cv::Mat(), Detector::imageModePresets));
    ret.push_back(ImageSource("gy", cv::Mat(), Detector::imageModePresets));
    ret.push_back(ImageSource("qmag", cv::Mat(), Detector::imageModePresets));
    ret.push_back(ImageSource("mag", cv::Mat(), Detector::imageModePresets));
    ret.push_back(ImageSource("dir", cv::Mat(), dirMapModes()));

    lsfm::DerivativeGradient<unsigned char, short, int, float_type, lsfm::SobelDerivative, lsfm::QuadraticMagnitude>
        grad;
    grad.process(src, ret[0].data, ret[1].data);
    ret[2].data = grad.magnitude();
    ret[2].data.convertTo(ret[3].data, cv::DataType<float_type>::type);
    cv::sqrt(ret[3].data, ret[3].data);
    ret[4].data = grad.direction();
    ret[4].range = QCPRange(grad.directionRange().lower, grad.directionRange().upper);

    return ret;
  }
};

//! default lsd creator
template <class LSD>
inline DetectorPtr createDetector(const QString& name, int f = 0) {
  return DetectorPtr(new DetectorT<LSD>(name, f));
}

//! gradiend based lsd creator
template <class LSD>
inline DetectorPtr createDetectorG(const QString& name, int f = 0) {
  return DetectorPtr(new DetectorG<LSD>(name, f));
}

//! gradiend based lsd creator
template <class LSD>
inline DetectorPtr createDetectorGS(const QString& name, int f = 0) {
  return DetectorPtr(new DetectorGS<LSD>(name, f));
}

//! gradiend based lsd creator
template <class LSD>
inline DetectorPtr createDetectorGSP(const QString& name, int f = 0) {
  return DetectorPtr(new DetectorGSP<LSD>(name, f));
}

//! edge lsd creator
template <class LSD>
inline DetectorPtr createDetectorE(const QString& name, int f = 0) {
  return DetectorPtr(new DetectorE<LSD>(name, f));
}

//! edge lsd creator
template <class LSD>
inline DetectorPtr createDetectorES(const QString& name, int f = 0) {
  return DetectorPtr(new DetectorES<LSD>(name, f));
}

//! edge pattern lsd creator
template <class LSD>
inline DetectorPtr createDetectorESP(const QString& name, int f = 0) {
  return DetectorPtr(new DetectorESP<LSD>(name, f));
}

//! edge pattern lsd creator
template <class LSD>
inline DetectorPtr createDetectorND(const QString& name) {
  return DetectorPtr(new DetectorND<LSD>(name));
}

//! specialised lsd creator
template <class LSD>
inline DetectorPtr createDetector() {
  return DetectorPtr(new LSD);
}


/*
// cclsd detector wrapper
struct DetectorRHLSD : public Detector {
    DetectorRHLSD() : Detector("RH LSD") {
        grads.push_back(lsfm::SobelGradientd::create());
        grads.push_back(lsfm::ScharrGradientd::create());
        grads.push_back(lsfm::DifferenceGradientd::create());

        fits.push_back(lsfm::RegressionFitLined::create());
        fits.push_back(lsfm::FPCAFitLined::create());
        fits.push_back(lsfm::PCAFitLined::create());
        fits.push_back(lsfm::MEstimatorFitLined::create());
    }

    virtual void create() {
        lsd = lsfm::RandomHoughDetector<double, lsfm::Point>::create();
        setGradient(0);
        setFitLine(0);
    }

    ImageSources sources(const cv::Mat& src = cv::Mat());

    void resetFitLineParams() {
        if (fitIdx == 2) {
            fits[2] = lsfm::MEstimatorFitLined::create();
        }
    };
};
*/
