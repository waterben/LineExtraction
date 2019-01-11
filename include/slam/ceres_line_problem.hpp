
#ifndef _CERES_LINE_PROBLEM_
#define _CERES_LINE_PROBLEM_
#ifdef __cplusplus


#include <cmath>
#include <cstdio>
#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "../geometry/line3.hpp"
#include "../geometry/camera.hpp"
#include "../slam/line_jet.hpp"

namespace lsfm {

    double coutJet(double v) {
      return v;
    }
    float coutJet(float v) {
      return v;
    }
    template<class JET>
    double coutJet(const JET& v) {
      return v.a;
    }

    template<class FT, int nCams>
    struct CayleyMultiCamError {

      //! Constructor for variable Pose and Line
      CayleyMultiCamError(const LineSegment<FT> &line, const Camera<FT> &c)
           : observedLine(line), cam(c), fixedPose(Pose<FT>()), fixedCayley(Vec4<FT>()) {}

      //! Constructor for fixed Pose (and changeable Line)
      CayleyMultiCamError(const LineSegment<FT> &line, const Camera<FT> &c, const Pose<FT> &fP)
          : observedLine(line), cam(c), fixedPose(fP), fixedCayley(Vec4<FT>()) {}

      //! Constructor for fixed Line (and changeable Pose)
      CayleyMultiCamError(const LineSegment<FT> &line, const Camera<FT> &c, const lsfm::Vec4<FT> &fC)
          : observedLine(line), cam(c), fixedPose(Pose<FT>()), fixedCayley(fC) {}


      // Nothing fixed
      bool operator()(const FT* const origin,
                      const FT* const rot,
                      const FT* const cayley,
                      FT* residuals) const {

          //const Vec3<FT> &trans = *reinterpret_cast<const Vec3<FT>*>(origin);
          //Matx33<FT> rotM;
          //ceres::AngleAxisToRotationMatrix(rot, ceres::RowMajorAdapter3x3(&rotM[0]));

          Pose<FT> tmpCam(cam);
          tmpCam.concat(Pose<FT> (origin,rot));

          // convert pointer to cayley object and compute projection to get 2d line from cayley
          Line<FT> projectedLine = CameraPluecker<FT>::projectCayleyCof(cam.camMCOF(),tmpCam.origin(),tmpCam.rotM(),*reinterpret_cast<const Vec4<FT>*>(cayley));

          // empty line -> we can not compute the residuals
          if (projectedLine.empty())
              return false;

          observedLine.error(projectedLine,residuals[0],residuals[1]);

          return true;
      }


      template <typename jetT>
      bool operator()(const jetT* const origin,
                      const jetT* const rot,
                      const jetT* const cayley,
                      jetT* residuals) const {

    /*
        const Vec3<jetT> &trans(*reinterpret_cast<const Vec3<jetT>*>(origin));
        Matx33<jetT> rotM, camCOF;
        convert2Jet(cam.camMCOF(),camCOF);
        ceres::AngleAxisToRotationMatrix(rot, RowMajorAdapter3x3(&rotM[0]));
        // convert pointer to cayley object and compute projection to get 2d line from cayley
        Line<jetT> projectedLine = CameraPluecker<jetT>::projectCayleyCof(camCOF,trans,rotM,*reinterpret_cast<const Vec4<jetT>*>(cayley));
    */

        Matx33<jetT> camCOF;
        convert2Jet(cam.camMCOF(),camCOF);
        Pose<jetT> tmpCam(convert2Jet<jetT>(cam.origin()), convert2Jet<jetT>(cam.orientation()));
        tmpCam.concat(Pose<jetT> (origin,rot));

        // convert pointer to cayley object and compute projection to get 2d line from cayley
        Line<jetT> projectedLine = CameraPluecker<jetT>::projectCayleyCof(camCOF,tmpCam.origin(),tmpCam.rotM(),*reinterpret_cast<const Vec4<jetT>*>(cayley));


        // empty line -> we can not compute the residuals
        if (projectedLine.empty())
            return false;

        LineSegment<jetT> obsLine(jetT(observedLine.normalX()),jetT(observedLine.normalY()),
                                  jetT(observedLine.originDist()),jetT(observedLine.start()),
                                  jetT(observedLine.end()));
        obsLine.error(projectedLine,residuals[0],residuals[1]);

        return true;
      }


      // Fixed Pose
      bool operator()(const FT* const cayley,
                      FT* residuals) const {

          //const Vec3<FT> &trans = *reinterpret_cast<const Vec3<FT>*>(origin);
          //Matx33<FT> rotM;
          //ceres::AngleAxisToRotationMatrix(rot, ceres::RowMajorAdapter3x3(&rotM[0]));

          Pose<FT> tmpCam(cam);
          tmpCam.concat(fixedPose);

          // convert pointer to cayley object and compute projection to get 2d line from cayley
          Line<FT> projectedLine = CameraPluecker<FT>::projectCayleyCof(cam.camMCOF(),tmpCam.origin(),tmpCam.rotM(),*reinterpret_cast<const Vec4<FT>*>(cayley));

          // empty line -> we can not compute the residuals
          if (projectedLine.empty())
              return false;

          observedLine.error(projectedLine,residuals[0],residuals[1]);

          return true;
      }

      template <typename jetT>
      bool operator()(const jetT* const cayley,
                      jetT* residuals) const {

    /*
        const Vec3<jetT> &trans(*reinterpret_cast<const Vec3<jetT>*>(origin));
        Matx33<jetT> rotM, camCOF;
        convert2Jet(cam.camMCOF(),camCOF);
        ceres::AngleAxisToRotationMatrix(rot, RowMajorAdapter3x3(&rotM[0]));
        // convert pointer to cayley object and compute projection to get 2d line from cayley
        Line<jetT> projectedLine = CameraPluecker<jetT>::projectCayleyCof(camCOF,trans,rotM,*reinterpret_cast<const Vec4<jetT>*>(cayley));
    */

        Matx33<jetT> camCOF;
        convert2Jet(cam.camMCOF(),camCOF);
        Pose<jetT> tmpCam(convert2Jet<jetT>(cam.origin()), convert2Jet<jetT>(cam.orientation()));
        Pose<jetT> tmpPoseFixed(convert2Jet<jetT>(fixedPose.origin()), convert2Jet<jetT>(fixedPose.orientation()));     // TODO - in constructor auslagern um rechenaufwand zu sparen
        tmpCam.concat(tmpPoseFixed);

        // convert pointer to cayley object and compute projection to get 2d line from cayley
        Line<jetT> projectedLine = CameraPluecker<jetT>::projectCayleyCof(camCOF,tmpCam.origin(),tmpCam.rotM(),*reinterpret_cast<const Vec4<jetT>*>(cayley));

        // empty line -> we can not compute the residuals
        if (projectedLine.empty())
            return false;

        LineSegment<jetT> obsLine(jetT(observedLine.normalX()),jetT(observedLine.normalY()),
                                  jetT(observedLine.originDist()),jetT(observedLine.start()),
                                  jetT(observedLine.end()));
        obsLine.error(projectedLine,residuals[0],residuals[1]);

        return true;
      }

      // Fixed Line
      bool operator()(const FT* const origin,
                      const FT* const rot,
                      FT* residuals) const {

          Pose<FT> tmpCam(cam);
          tmpCam.concat(Pose<FT> (origin,rot));
          // convert pointer to cayley object and compute projection to get 2d line from cayley
          Line<FT> projectedLine = CameraPluecker<FT>::projectCayleyCof(cam.camMCOF(),tmpCam.origin(),tmpCam.rotM(), fixedCayley);

          // empty line -> we can not compute the residuals
          if (projectedLine.empty()){
              return false;
          }

          observedLine.error(projectedLine,residuals[0],residuals[1]);

          return true;
      }


      template <typename jetT>
      bool operator()(const jetT* const origin,
                      const jetT* const rot,
                      jetT* residuals) const {

        Matx33<jetT> camCOF;
        convert2Jet(cam.camMCOF(),camCOF);
        Pose<jetT> tmpCam(convert2Jet<jetT>(cam.origin()), convert2Jet<jetT>(cam.orientation()));
        tmpCam.concat(Pose<jetT> (origin,rot));

        // convert pointer to cayley object and compute projection to get 2d line from cayley
        lsfm::Vec4< jetT > fixedCayleyJetT ( (jetT( fixedCayley[0] )) ,  (jetT( fixedCayley[1] )) ,  (jetT( fixedCayley[2] )) ,  (jetT( fixedCayley[3] )) );
        Line<jetT> projectedLine = CameraPluecker<jetT>::projectCayleyCof(camCOF,tmpCam.origin(),tmpCam.rotM(), fixedCayleyJetT);

        // empty line -> we can not compute the residuals
        if (projectedLine.empty()){
            return false;
        }

        LineSegment<jetT> obsLine(jetT(observedLine.normalX()),jetT(observedLine.normalY()),
                                  jetT(observedLine.originDist()),jetT(observedLine.start()),
                                  jetT(observedLine.end()));
        obsLine.error(projectedLine,residuals[0],residuals[1]);

        return true;
      }



      // Factory to hide the construction of the CostFunction object from
      // the client code.
      static ceres::CostFunction* Create(const LineSegment<FT> &line, const Camera<FT> &c) {
        return (new ceres::AutoDiffCostFunction<CayleyMultiCamError, 2, 3, 3, 4>(
                    new CayleyMultiCamError(line, c)));
      }


      // Factory to hide the construction of the CostFunction object from
      // the client code.
      static ceres::CostFunction* CreateFixedPose(const LineSegment<FT> &line, const Camera<FT> &c, const Pose<FT> &p) {
        return (new ceres::AutoDiffCostFunction<CayleyMultiCamError, 2, 4>(
                    new CayleyMultiCamError(line, c, p)));
      }


      // Factory to hide the construction of the CostFunction object from
      // the client code.
      static ceres::CostFunction* CreateFixedLine(const LineSegment<FT> &line, const Camera<FT> &c, const lsfm::Vec4<FT> &fC) {
        return (new ceres::AutoDiffCostFunction<CayleyMultiCamError, 2, 3, 3>(
                    new CayleyMultiCamError(line, c, fC)));
      }

      const LineSegment<FT> &observedLine;
      CameraPluecker<FT> cam;
      const Pose<FT> &fixedPose;
      const lsfm::Vec4<FT> &fixedCayley;
    };



    template<class FT>
    struct CayleyMultiCamError<FT, 1> {
      CayleyMultiCamError(const LineSegment<FT> &line, const Camera<FT> &c)
          : observedLine(line), cam(c) {}

      bool operator()(const FT* const origin,
                      const FT* const rot,
                      const FT* const cayley,
                      FT* residuals) const {

          const Vec3<FT> &trans = *reinterpret_cast<const Vec3<FT>*>(origin);

          Matx33<FT> rotM;
          ceres::AngleAxisToRotationMatrix(rot, ceres::RowMajorAdapter3x3(&rotM[0]));

          // convert pointer to cayley object and compute projection to get 2d line from cayley
          Line<FT> projectedLine = CameraPluecker<FT>::projectCayleyCof(cam.camMCOF(),trans,rotM,*reinterpret_cast<const Vec4<FT>*>(cayley));

          // empty line -> we can not compute the residuals
          if (projectedLine.empty())
              return false;

          observedLine.error(projectedLine,residuals[0],residuals[1]);

          return true;
      }


      template <typename jetT>
      bool operator()(const jetT* const origin,
                      const jetT* const rot,
                      const jetT* const cayley,
                      jetT* residuals) const {

        const Vec3<jetT> &trans(*reinterpret_cast<const Vec3<jetT>*>(origin));

        Matx33<jetT> rotM, camCOF;
        convert2Jet(cam.camMCOF(),camCOF);
        ceres::AngleAxisToRotationMatrix(rot, RowMajorAdapter3x3(&rotM[0]));

        // convert pointer to cayley object and compute projection to get 2d line from cayley
        Line<jetT> projectedLine = CameraPluecker<jetT>::projectCayleyCof(camCOF,trans,rotM,*reinterpret_cast<const Vec4<jetT>*>(cayley));

        // empty line -> we can not compute the residuals
        if (projectedLine.empty())
            return false;

        LineSegment<jetT> obsLine(jetT(observedLine.normalX()),jetT(observedLine.normalY()),
                                  jetT(observedLine.originDist()),jetT(observedLine.start()),
                                  jetT(observedLine.end()));
        obsLine.error(projectedLine,residuals[0],residuals[1]);

        return true;

      }
      // Factory to hide the construction of the CostFunction object from
      // the client code.
      static ceres::CostFunction* Create(const LineSegment<FT> &line, const Camera<FT> &c) {
        return (new ceres::AutoDiffCostFunction<CayleyMultiCamError, 2, 3, 3, 4>(
                    new CayleyMultiCamError(line, c)));
      }

      const LineSegment<FT> &observedLine;
      Camera<FT> cam;
    };



    template<class FT, int nCams>
    struct PointMultiCamError {
      PointMultiCamError(const lsfm::Vec2<FT> point, const Camera<FT> &c)
          : observedPoint(point), cam(c) {}

      bool operator()(const FT* const origin,
                      const FT* const rot,
                      const FT* const p3d,
                      FT* residuals) const {

          Matx34<FT> projM;
          Matx33<FT> camM = (cam.camM());

          Pose<FT> tmpPose((cam.origin()), (cam.orientation()));
          tmpPose.concat(Pose<FT> (origin,rot));

          projM = Camera<FT>::composeProjectionMatrix(camM,tmpPose.origin(), tmpPose.rotM());

          lsfm::Vec2<FT> projectedPoint;
          if(!Camera<FT>::project(projM, *reinterpret_cast<const Vec3<FT>*>(p3d), projectedPoint))
              return false;

          residuals[0] = detail::abs(observedPoint.x() - projectedPoint.x());
          residuals[1] = detail::abs(observedPoint.y() - projectedPoint.y());

          return true;
      }


      template <typename jetT>
      bool operator()(const jetT* const origin,
                      const jetT* const rot,
                      const jetT* const p3d,
                      jetT* residuals) const {

        Matx34<jetT> projM;
        Matx33<jetT> camM = convert2Jet<jetT>(cam.camM());

        Pose<jetT> tmpPose(convert2Jet<jetT>(cam.origin()), convert2Jet<jetT>(cam.orientation()));
        tmpPose.concat(Pose<jetT> (origin,rot));

        projM = Camera<jetT>::composeProjectionMatrix(camM,tmpPose.origin(), tmpPose.rotM());

        lsfm::Vec2<jetT> projectedPoint;
        if(!Camera<jetT>::project(projM, *reinterpret_cast<const Vec3<jetT>*>(p3d), projectedPoint))
            return false;

        residuals[0] = detail::abs(observedPoint.x() - projectedPoint.x());
        residuals[1] = detail::abs(observedPoint.y() - projectedPoint.y());
/*
        std::cout << "obsX: " << coutJet(observedPoint.x()) << " projX: " << coutJet(projectedPoint.x()) << std:: endl;
        std::cout << "obsY: " << coutJet(observedPoint.y()) << " projY: " << coutJet(projectedPoint.y()) << std:: endl;
*/

        return true;
      }

      // Factory to hide the construction of the CostFunction object from
      // the client code.
      static ceres::CostFunction* Create(const lsfm::Vec2<FT> &point, const Camera<FT> &c) {
        return (new ceres::AutoDiffCostFunction<PointMultiCamError, 2, 3, 3, 3>(
                    new PointMultiCamError(point, c)));
      }

      const lsfm::Vec2<FT> observedPoint;
      Camera<FT> cam;
    };



    template<class FT>
    struct PointMultiCamError<FT, 1> {
      PointMultiCamError(const lsfm::Vec2<FT> point, const Camera<FT> &c)
          : observedPoint(point), cam(c) {}

      bool operator()(const FT* const origin,
                      const FT* const rot,
                      const FT* const p3d,
                      FT* residuals) const {

          Matx34<FT> projM;
          Matx33<FT> camM = (cam.camM());

          //Pose<FT> tmpPose((cam.origin()), (cam.orientation()));
          //tmpPose.concat(Pose<FT> (origin,rot));
          Pose<FT> tmpPose(origin, rot);

          projM = Camera<FT>::composeProjectionMatrix(camM,tmpPose.origin(), tmpPose.rotM());

          lsfm::Vec2<FT> projectedPoint;
          if(!Camera<FT>::project(projM, *reinterpret_cast<const Vec3<FT>*>(p3d), projectedPoint))
              return false;

          residuals[0] = detail::abs(observedPoint.x() - projectedPoint.x());
          residuals[1] = detail::abs(observedPoint.y() - projectedPoint.y());

          return true;
      }


      template <typename jetT>
      bool operator()(const jetT* const origin,
                      const jetT* const rot,
                      const jetT* const p3d,
                      jetT* residuals) const {

          Matx34<jetT> projM;
          Matx33<jetT> camM = convert2Jet<jetT>(cam.camM());

          //Pose<jetT> tmpPose(convert2Jet<jetT>(cam.origin()), convert2Jet<jetT>(cam.orientation()));
          //tmpPose.concat(Pose<jetT> (origin,rot));
          Pose<jetT> tmpPose(origin, rot);

          projM = Camera<jetT>::composeProjectionMatrix(camM,tmpPose.origin(), tmpPose.rotM());

          lsfm::Vec2<jetT> projectedPoint;
          if(!Camera<jetT>::project(projM, *reinterpret_cast<const Vec3<jetT>*>(p3d), projectedPoint))
              return false;

          residuals[0] = detail::abs(observedPoint.x() - projectedPoint.x());
          residuals[1] = detail::abs(observedPoint.y() - projectedPoint.y());

          return true;
      }
      // Factory to hide the construction of the CostFunction object from
      // the client code.
      static ceres::CostFunction* Create(const lsfm::Vec2<FT> &point, const Camera<FT> &c) {
        return (new ceres::AutoDiffCostFunction<PointMultiCamError, 2, 3, 3, 3>(
                    new PointMultiCamError(point, c)));
      }

      const lsfm::Vec2<FT> observedPoint;
      CameraPluecker<FT> cam;
    };


    template<class FT>
    struct ModelLinesToGtTransformationError {
        ModelLinesToGtTransformationError(const lsfm::Line3<FT> &mL, const lsfm::LineSegment3<FT> &gtL)
            : modelLine(mL), gtLine(gtL) {}

        bool operator()(const FT* const origin,
                        const FT* const rot,
                        FT* residuals) const {

            Pose<FT> tmpPose(origin, rot);

            lsfm::Vec4<FT> p1h(modelLine.origin()[0], modelLine.origin()[1], modelLine.origin()[2], 1);
            lsfm::Vec4<FT>p1ht = tmpPose.homM() * p1h;

            lsfm::Vec4<FT> p2h(modelLine.origin()[0] + modelLine.direction()[0], modelLine.origin()[1] + modelLine.direction()[1], modelLine.origin()[2] + modelLine.direction()[2], 1);
            lsfm::Vec4<FT>p2ht = tmpPose.homM() * p2h;

            lsfm::Vec3<FT> p1(p1ht[0] / p1ht[3], p1ht[1] / p1ht[3], p1ht[2] / p1ht[3]);
            lsfm::Vec3<FT> p2(p2ht[0] / p2ht[3], p2ht[1] / p2ht[3], p2ht[2] / p2ht[3]);

            lsfm::Line3<FT> transformedLine(p1, p2 - p1);

            //residuals[0] = transformedLine.distance(gtLine);

            residuals[0] = detail::abs( transformedLine.distance(gtLine.startPoint()) );
            residuals[1] = detail::abs( transformedLine.distance(gtLine.endPoint()) );

            FT r1 = residuals[0];
            FT r2 = residuals[1];
            FT rSum = r1 + r2;

            residuals[0] = FT(0);
            residuals[1] = rSum;

            return true;
        }

        template <typename jetT>
        bool operator()(const jetT* const origin,
                        const jetT* const rot,
                        jetT* residuals) const {

            Pose<jetT> tmpPose(origin, rot);

            lsfm::Vec4<jetT> p1h(jetT(modelLine.origin()[0]), jetT(modelLine.origin()[1]), jetT(modelLine.origin()[2]), jetT(1));
            lsfm::Vec4<jetT> p1ht = tmpPose.homM() * p1h;

            lsfm::Vec4<jetT> p2h(jetT(modelLine.origin()[0] + modelLine.direction()[0]), jetT(modelLine.origin()[1] + modelLine.direction()[1]), jetT(modelLine.origin()[2] + modelLine.direction()[2]), jetT(1));
            lsfm::Vec4<jetT> p2ht = tmpPose.homM() * (p2h);

            lsfm::Vec3<jetT> p1(p1ht[0] / p1ht[3], p1ht[1] / p1ht[3], p1ht[2] / p1ht[3]);
            lsfm::Vec3<jetT> p2(p2ht[0] / p2ht[3], p2ht[1] / p2ht[3], p2ht[2] / p2ht[3]);


            lsfm::Line3<jetT> transformedLine(p1, p2 - p1);

            //lsfm::LineSegment3<jetT> gtLineJet(convert2Jet<jetT>(gtLine.startPoint()), convert2Jet<jetT>(gtLine.endPoint()));

            residuals[0] = detail::abs( transformedLine.distance(convert2Jet<jetT>(gtLine.startPoint())) );
            residuals[1] = detail::abs( transformedLine.distance(convert2Jet<jetT>(gtLine.endPoint())) );

            jetT r1 = residuals[0];
            jetT r2 = residuals[1];
            jetT rSum = r1 + r2;

            residuals[0] = jetT(0);
            residuals[1] = rSum;

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const lsfm::Line3<FT> &mL, const lsfm::LineSegment3<FT> &gtL) {
            return (new ceres::AutoDiffCostFunction<ModelLinesToGtTransformationError, 2, 3, 3>(
                        new ModelLinesToGtTransformationError(mL, gtL)));
        }

        const lsfm::Line3<FT> modelLine;
        const lsfm::LineSegment3<FT> gtLine;

    };

}


#endif
#endif
