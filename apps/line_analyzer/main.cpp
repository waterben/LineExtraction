#include "controlwindow.h"
#include "helpers.h"
#include "pofuncplot.h"
#include "precisionoptimizer.h"

#include <QApplication>
// #include "continuityoptimizer.h"
// #include "connectionoptimizer.h"
// #include "lineanalyser2d.h"
#include "profileanalyzer.h"

#define USE_PERIODIC_FFT
#include <edge/edge_drawing.hpp>
#include <edge/edge_simple.hpp>
#include <imgproc/gradient_adapter.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>
#include <lsd/lsd_burns.hpp>
#include <lsd/lsd_cc.hpp>
#include <lsd/lsd_cp.hpp>
#include <lsd/lsd_edlz.hpp>
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_ep.hpp>
#include <lsd/lsd_fbw.hpp>
#include <lsd/lsd_fgioi.hpp>
#include <lsd/lsd_hcv.hpp>
#ifdef _MSC_VER
#  include <lsd/lsd_edta.hpp>
#  pragma comment(lib, "../../../lib/EDLinesLib.lib")  // DetectLinesByED function
#  if (_MSC_VER >= 1900)
#    pragma comment( \
        lib,         \
        "legacy_stdio_definitions.lib")  // add legacy_stdio_definitions.lib for VS2015 because of define changes!!!
#  endif
#endif

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);
  ControlWindow w;

  w.addDetector(createDetectorES<lsfm::LsdEL<float_type, cv::Point_>>("LSD EL", D_MAG_SQR));
  w.addDetector(
      createDetectorES<
          lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                      lsfm::EdgeSourceGRAD<lsfm::DerivativeGradient<uchar, short, int, float_type,
                                                                    lsfm::SobelDerivative, lsfm::QuadraticMagnitude>,
                                           lsfm::NonMaximaSuppression<short, int, float_type>>,
                      lsfm::EsdDrawing<int, lsfm::NonMaximaSuppression<short, int, float_type>::NUM_DIR>>>("LSD ED",
                                                                                                           D_MAG_SQR));
  w.addDetector(
      createDetectorES<
          lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                      lsfm::EdgeSourceGRAD<lsfm::DerivativeGradient<uchar, short, int, float_type,
                                                                    lsfm::SobelDerivative, lsfm::QuadraticMagnitude>,
                                           lsfm::NonMaximaSuppression<short, int, float_type>>,
                      lsfm::EsdSimple<int, lsfm::NonMaximaSuppression<short, int, float_type>::NUM_DIR>>>("LSD ES",
                                                                                                          D_MAG_SQR));
  w.addDetector(createDetectorESP<lsfm::LsdEP<float_type, cv::Point_>>("LSD EP", D_MAG_SQR));
  w.addDetector(createDetectorGS<lsfm::LsdCC<float_type, cv::Point_>>("LSD CC", D_MAG_SQR));
  w.addDetector(createDetectorGSP<lsfm::LsdCP<float_type, cv::Point_>>("LSD CP", D_MAG_SQR));

  w.addDetector(createDetectorE<lsfm::LsdHough<float_type, cv::Point_>>("LSD HOUGH", D_MAG_SQR));
  w.addDetector(createDetectorE<lsfm::LsdHoughP<float_type, cv::Point_>>("LSD HOUGHP", D_MAG_SQR));

  w.addDetector(createDetector<lsfm::LsdBurns<float_type, cv::Point_>>("LSD BURNS", D_MAG_SQR));
  w.addDetector(createDetectorND<lsfm::LsdFGioi<float_type, cv::Point_>>("LSD FGIOI"));
  w.addDetector(createDetector<lsfm::LsdFBW<float_type, cv::Point_>>("LSD FBW", D_MAG_SQR));
  w.addDetector(createDetector<lsfm::LsdEDLZ<float_type, cv::Point_>>("LSD EDLZ"));
#ifdef _MSC_VER
  w.addDetector(createDetectorND<lsfm::LsdEDTA<float_type, cv::Point_>>("LSD EDTA"));
#endif

  w.addDetector(
      createDetectorES<lsfm::LsdEL<
          float_type, cv::Point_, lsfm::Vec2i,
          lsfm::EdgeSourceQUAD<lsfm::QuadratureG2<uchar, float_type>,
                               lsfm::NonMaximaSuppression<float_type, float_type, float_type,
                                                          lsfm::FastNMS4<float_type, float_type, float_type>>>>>(
          "LSD EL QFSt Odd"));
  w.addDetector(createDetectorES<
                lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                            lsfm::EdgeSourceQUAD<lsfm::QuadratureS<uchar, float_type, float_type>,
                                                 lsfm::NonMaximaSuppression<float_type, float_type, float_type>>>>(
      "LSD EL SQ Odd"));
  w.addDetector(createDetectorES<
                lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                            lsfm::EdgeSourceQUAD<lsfm::QuadratureSF<uchar, float_type>,
                                                 lsfm::NonMaximaSuppression<float_type, float_type, float_type>>>>(
      "LSD EL SQF Odd"));
  w.addDetector(createDetectorES<
                lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                            lsfm::EdgeSourceQUAD<lsfm::QuadratureLGF<uchar, float_type>,
                                                 lsfm::NonMaximaSuppression<float_type, float_type, float_type>>>>(
      "LSD EL SQLGF Odd"));

  w.addDetector(
      createDetectorES<lsfm::LsdEL<
          float_type, cv::Point_, lsfm::Vec2i,
          lsfm::EdgeSourceQUADZC<
              lsfm::QuadratureG2<uchar, float_type>,
              lsfm::ZeroCrossing<float_type, float_type, float_type,
                                 lsfm::FastZC<float_type, float_type, float_type, lsfm::NCC_BASIC, lsfm::EZCMap4>>>>>(
          "LSD EL QFSt Even"));
  w.addDetector(
      createDetectorES<lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                                   lsfm::EdgeSourceQUADZC<lsfm::QuadratureS<uchar, float_type, float_type>,
                                                          lsfm::ZeroCrossing<float_type, float_type, float_type>>>>(
          "LSD EL SQ Even"));
  w.addDetector(
      createDetectorES<lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                                   lsfm::EdgeSourceQUADZC<lsfm::QuadratureSF<uchar, float_type>,
                                                          lsfm::ZeroCrossing<float_type, float_type, float_type>>>>(
          "LSD EL SQF Even"));
  w.addDetector(
      createDetectorES<lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                                   lsfm::EdgeSourceQUADZC<lsfm::QuadratureLGF<uchar, float_type>,
                                                          lsfm::ZeroCrossing<float_type, float_type, float_type>>>>(
          "LSD EL SQLGF Even"));

  /*w.addDetector(createDetectorES<lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
  lsfm::EdgeSourceQUAD<lsfm::QuadratureG2<uchar, float_type>, lsfm::NonMaximaSuppression<float_type, float_type,
  float_type, lsfm::FastNMS4<float_type, float_type, float_type>>>>>("LSD EL QFSt"));
  w.addDetector(createDetectorES<lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
  lsfm::EdgeSourceQUAD<lsfm::QuadratureS<uchar, float_type, float_type>,lsfm::NonMaximaSuppression<float_type,
  float_type, float_type>>>>("LSD EL SQ")); w.addDetector(createDetectorES<lsfm::LsdEL<float_type, cv::Point_,
  lsfm::Vec2i, lsfm::EdgeSourceQUAD<lsfm::QuadratureSF<uchar, float_type>, lsfm::NonMaximaSuppression<float_type,
  float_type, float_type>>>>("LSD EL SQF")); w.addDetector(createDetectorES<lsfm::LsdEL<float_type, cv::Point_,
  lsfm::Vec2i, lsfm::EdgeSourceQUAD<lsfm::QuadratureLGF<uchar, float_type>, lsfm::NonMaximaSuppression<float_type,
  float_type, float_type>>>>("LSD EL SQLGF"));*/
  w.addDetector(
      createDetectorES<lsfm::LsdEL<
          float_type, cv::Point_, lsfm::Vec2i,
          lsfm::EdgeSourcePC<lsfm::PCLgf<uchar, float_type>,
                             lsfm::NonMaximaSuppression<float_type, float_type, float_type,
                                                        lsfm::FastNMS4<float_type, float_type, float_type>>>>>(
          "LSD EL PCLG"));
  w.addDetector(
      createDetectorES<lsfm::LsdEL<
          float_type, cv::Point_, lsfm::Vec2i,
          lsfm::EdgeSourcePC<lsfm::PCSqf<uchar, float_type>,
                             lsfm::NonMaximaSuppression<float_type, float_type, float_type,
                                                        lsfm::FastNMS4<float_type, float_type, float_type>>>>>(
          "LSD EL PCSQ"));
  w.addDetector(
      createDetectorES<lsfm::LsdEL<
          float_type, cv::Point_, lsfm::Vec2i,
          lsfm::EdgeSourcePCLZC<
              lsfm::PCLSq<uchar, float_type, float_type>,
              lsfm::ZeroCrossing<float_type, float_type, float_type,
                                 lsfm::FastZC<float_type, float_type, float_type, lsfm::NCC_BASIC, lsfm::EZCMap4>>>>>(
          "LSD EL PCLSq"));
  w.addDetector(
      createDetectorES<lsfm::LsdEL<
          float_type, cv::Point_, lsfm::Vec2i,
          lsfm::EdgeSourcePCLZC<
              lsfm::PCLSqf<uchar, float_type>,
              lsfm::ZeroCrossing<float_type, float_type, float_type,
                                 lsfm::FastZC<float_type, float_type, float_type, lsfm::NCC_BASIC, lsfm::EZCMap4>>>>>(
          "LSD EL PCLSqf"));
  // static_assert (sizeof(float_type) == sizeof(double), "Comment out this and next line!");
  // w.addDetector(createDetectorES<lsfm::LsdEL<double, cv::Point_, lsfm::Vec2i,
  // lsfm::EdgeSourcePC<lsfm::PCMatlab<uchar>, lsfm::NonMaximaSuppression<double, double, double, lsfm::FastNMS4<double,
  // double, double>>>>>("LSD EL PCML"));
  w.addDetector(
      createDetectorES<
          lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                      lsfm::EdgeSourceGRAD<
                          lsfm::SusanGradient<short, int, float_type>,
                          lsfm::NonMaximaSuppression<short, int, float_type, lsfm::FastNMS4<short, int, float_type>>>>>(
          "LSD EL SUSAN"));
  w.addDetector(
      createDetectorES<
          lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                      lsfm::EdgeSourceGRAD<
                          lsfm::RCMGradient<uchar, 1, short, int, float_type>,
                          lsfm::NonMaximaSuppression<short, int, float_type, lsfm::FastNMS4<short, int, float_type>>>>>(
          "LSD EL RMG"));
  w.addDetector(
      createDetectorES<
          lsfm::LsdEL<float_type, cv::Point_, lsfm::Vec2i,
                      lsfm::EdgeSourceGRAD<
                          lsfm::RCMGradient<uchar, 3, short, int, float_type>,
                          lsfm::NonMaximaSuppression<short, int, float_type, lsfm::FastNMS4<short, int, float_type>>>>>(
          "LSD EL RCMG", D_COLOR_INPUT));

  w.addTool<ProfileAnalyzer>();
  w.addTool<PrecisionOptimizer>();
  w.addTool<POFuncPlot>();
  /*

  w.addTool<ContinuityOptimizer>();
  w.addTool<ConnectionOptimizer>();
  w.addTool<LineAnalyser2D>();*/

  w.show();

  return a.exec();
}
