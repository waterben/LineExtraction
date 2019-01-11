#ifndef LINEPROCESSING_H
#define LINEPROCESSING_H

#ifdef __cplusplus

#include <iomanip>  // for std::setw
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
//#include <line_tracker/linetracker.hpp>
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_cc.hpp>
#include <geometry/draw.hpp>
#include <slam/slamDataModel.hpp>
#include <slam/pose_estimator.hpp>
#include <opengl_sim/lineMatching.hpp>


namespace lsfm{

    template<class T>
    static void visibilityCheck(const std::vector<lsfm::LineSegment3<FT>> & modelsToCheck, std::vector<lsfm::LineSegment3<FT>> & visibleLineModels3d, const lsfm::Matx44<FT> & homMat,const std::vector<T>& lineModel3dIDs, std::vector<T> & visibleLineModel3dIDs )
    {
        if(modelsToCheck.size() <= 0)
            return;

        lsfm::Vec4<FT> sp, ep, spc, epc;
        for(int i = 0; i < modelsToCheck.size(); ++i){
            sp[0] = modelsToCheck[i].startPoint()[0];
            sp[1] = modelsToCheck[i].startPoint()[1];
            sp[2] = modelsToCheck[i].startPoint()[2];
            sp[3] = 1;
            ep[0] = modelsToCheck[i].endPoint()[0];
            ep[1] = modelsToCheck[i].endPoint()[1];
            ep[2] = modelsToCheck[i].endPoint()[2];
            ep[3] = 1;

            spc = homMat * sp;
            epc = homMat * ep;

            if(spc[2] >= 0 && epc[2] >= 0){
                visibleLineModels3d.push_back(modelsToCheck[i]);
    //            if(lineModel3dIDs.size() > i)
                visibleLineModel3dIDs.push_back(lineModel3dIDs[i]);
            }
        }
        if(visibleLineModels3d.size() != visibleLineModel3dIDs.size()){
            std::cout << "error" << std::endl;
        }
    }

    static void visibilityCheck(const std::vector<lsfm::LineSegment3<FT>> & modelsToCheck, std::vector<lsfm::LineSegment3<FT>> & visibleLineModels3d, const lsfm::Matx44<FT> & homMat){
        std::vector<std::string> lineModel3dIDs, visibleLineModel3dIDs;
        for(int i = 0; i < modelsToCheck.size(); ++i){
            lineModel3dIDs.push_back(std::to_string(i));
        }
        visibilityCheck(modelsToCheck, visibleLineModels3d, homMat, lineModel3dIDs, visibleLineModel3dIDs);
    }


    template<class FT>
    void getIdsFromGT(typename lsfm::LsdBase<FT>::LineSegmentVector lines, cv::Mat gtImg, std::vector<int> & lineIds){

        using PT = typename lsfm::LsdBase<FT>::LineSegment::point_type;
        //const int samples = 19;

        for_each(lines.begin(), lines.end(), [&](typename lsfm::LsdBase<FT>::LineSegment ls){

            FT lineLength = (ls.end() - ls.start());

            typedef std::map<int, int> voteMap;
            voteMap votes;
            FT step = lineLength; //  / samples;

            for(int i = 0; i <=lineLength; ++i){
                PT point = ls.lineDist(ls.start() + i, ls.origin());
                if(point.y() >= gtImg.size().height || point.x() >= gtImg.size().width)
                    continue;
//                cv::circle(gtImg, cv::Point_<FT>(point.x(), point.y()), 1, cv::Scalar(255,255,255),1);
                cv::Point3_<uchar>* p = gtImg.ptr<cv::Point3_<uchar> >(point.y(),point.x());

                int id = static_cast<int>(p->z) + 256 * static_cast<int>(p->y);     //  std::cout << "R: " << (int)p->z << " G: " << (int)p->y << " B: " << (int)p->x << std::endl;
                int b = static_cast<int>(p->x);
                if(id != 65535 && id != 0 && b == 255) // if not black background
                    votes[id] = votes[id] + 1;
            }

            int key = -1, maxVotes = 0;
            for(voteMap::iterator iterator = votes.begin(); iterator != votes.end(); iterator++) {
                if (iterator->second > maxVotes){
                    maxVotes = iterator->second;
                    key = iterator->first;
                }
            }

            if(maxVotes < 0.90 * lineLength)   // sort out if not clearly one certain line
                key = -1;
            lineIds.push_back(key);
        });
    }


    template<class FT, class MySlam, class MapType>
    void lineProcessing(cv::Mat cam0img, cv::Mat cam1img, cv::Mat cam0GTimg, cv::Mat cam1GTimg, MapType & gtToModelId, MySlam * slam, lsfm::Pose<FT> & currentPose, lsfm::Camera<FT> cam0Gt = lsfm::Camera<FT>(), lsfm::Camera<FT> cam1Gt = lsfm::Camera<FT>()){

        //class SPE = PixelEstimator<FT,PT>
        //PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT,FT,cv::Point_, QuadraticEstimate, CubicInterpolator>> spe;
        //PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>> spezc;

        lsfm::LsdEL<FT> lsd0(0.004, 0.03, 60, 4, 30, 10);   // EL_USE_NFA
        lsfm::LsdEL<FT> lsd1(0.004, 0.03, 60, 4, 30, 10);

        cv::Mat cam0grey, cam1grey;
        cvtColor(cam0img, cam0grey, CV_RGB2GRAY);
        cvtColor(cam1img, cam1grey, CV_RGB2GRAY);
        lsd0.detect(cam0grey);
        lsd1.detect(cam1grey);

        // Tracking
        std::vector<int> lineIds0, lineIds1;
        getIdsFromGT<FT>(lsd0.lineSegments(), cam0GTimg, lineIds0);
        getIdsFromGT<FT>(lsd1.lineSegments(), cam1GTimg, lineIds1);

        std::vector<DetectedLine> detectedLines0, detectedLines1;
        detectedLines0.assign(lsd0.lineSegments().size(), DetectedLine());
        detectedLines1.assign(lsd1.lineSegments().size(), DetectedLine());

        cv::Mat line_desc0 = createLBD(cam0grey, lsd0.lineSegments());
        cv::Mat line_desc1 = createLBD(cam1grey, lsd1.lineSegments());

        // matching tracking -----------------------------------------------------------------------------
        // Left -------------------
        std::vector<DMatch> matchesLeft, matchesLeftVV, matchesLeftLRcheck, matchesRight, matchesRightVV, matchesRightLRcheck;
        if(slam->getFrameNum() >= 2){   // Check if 'initialized'

            cv::Mat line_desc0_previous = slam->getLineDescriptorsMat(slam->getFrameNum() - 2);
            matchesLeftLRcheck = lsfm::matchingLRcheck(line_desc0, line_desc0_previous);
            imshow("Detected matches tracking Left LR check", drawKeyLineMatches<FT,DescriptorMatch<FT>>(cam0img, lsd0.lineSegments(), slam->getImageFrame(slam->getFrameNum() - 2), slam->getLineSegments(slam->getFrameNum() - 2), matchesLeftLRcheck));

            std::vector<DetectedLine> previousDetectedLines0 = slam->getDetectedLines(slam->getFrameNum() - 2);
            for(int i = 0; i < matchesLeftLRcheck.size(); ++i){
                if(previousDetectedLines0[matchesLeftLRcheck[i].trainIdx].modelIndex >= 0){
                    detectedLines0[matchesLeftLRcheck[i].queryIdx].modelIndex = previousDetectedLines0[matchesLeftLRcheck[i].trainIdx].modelIndex;
                }
            }


        // Right ------------------

            cv::Mat line_desc1_previous = slam->getLineDescriptorsMat(slam->getFrameNum() - 1);
            matchesRightLRcheck = lsfm::matchingLRcheck(line_desc1, line_desc1_previous);
            imshow("Detected matches tracking Right LR check", drawKeyLineMatches<FT,DescriptorMatch<FT>>(cam1img, lsd1.lineSegments(), slam->getImageFrame(slam->getFrameNum() - 1), slam->getLineSegments(slam->getFrameNum() - 1), matchesRightLRcheck));

            std::vector<DetectedLine> previousDetectedLines1 = slam->getDetectedLines(slam->getFrameNum() - 1);
            for(int i = 0; i < matchesRightLRcheck.size(); ++i){
                if(previousDetectedLines1[matchesRightLRcheck[i].trainIdx].modelIndex >= 0){
                    detectedLines1[matchesRightLRcheck[i].queryIdx].modelIndex = previousDetectedLines1[matchesRightLRcheck[i].trainIdx].modelIndex;
                }
            }

        }

//        std::sort by train index

        for(int i = 0; i < matchesLeft.size(); ++i){
            std::cout << "matchNr: " << i << std::endl <<
                         "dist: " << matchesLeft[i].distance << std::endl <<
                         "  query: " << matchesLeft[i].queryIdx << std::endl <<
                         "  train: " << matchesLeft[i].trainIdx << std::endl << std::endl;
        }


        slam->addData(lsd0.lineSegments(), detectedLines0);
        slam->addData(lsd1.lineSegments(), detectedLines1);
        slam->addData(cam0img);
        slam->addData(cam1img);
        slam->addDataLineDescriptor(line_desc0);
        slam->addDataLineDescriptor(line_desc1);

        // calculate position------------------------------------------------------------------------------

        lsfm::Camera<FT> tmpCamLeft(slam->getCameraConfig(0));
        lsfm::Camera<FT> tmpCamRight(slam->getCameraConfig(1));

        tmpCamLeft.concat(currentPose);
        tmpCamRight.concat(currentPose);

        std::vector<typename MySlam::line_segment_type> reprojSegsL, reprojSegsR;
        std::vector<typename MySlam::line_3d_type> modelLinesCamL, modelLinesCamR;

        for(int i = 0; i < detectedLines0.size(); ++i){
            if(detectedLines0[i].modelIndex >= 0){
                reprojSegsL.push_back(lsd0.lineSegments()[i]);
                modelLinesCamL.push_back(slam->getModeledLines()[detectedLines0[i].modelIndex].line);
            }
        }
        for(int i = 0; i < detectedLines1.size(); ++i){
            if(detectedLines1[i].modelIndex >= 0){
                reprojSegsR.push_back(lsd1.lineSegments()[i]);
                modelLinesCamR.push_back(slam->getModeledLines()[detectedLines1[i].modelIndex].line);
            }
        }               

        lsfm::Pose<FT> newPose1, newPose2, newPose3, newPose4;
        typedef std::pair<FT, lsfm::Pose<FT>> PosePair;
        std::vector<PosePair> possiblePoses;
        lsfm::CameraPluecker<FT> plueckerCamL(tmpCamLeft), plueckerCamR(tmpCamRight);

        double start = double(cv::getTickCount());
        double minReprojELeftAuto = PnLpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, true);
        possiblePoses.push_back(PosePair(minReprojELeftAuto, newPose1));
        std::cout << " e : " << minReprojELeftAuto << std::endl;
        std::cout << "auto time: " << (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() << std::endl;
        FT err = FT(0);
        plueckerCamL.pose(newPose1);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        double minReprojERightAuto = PnLpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, true);
        possiblePoses.push_back(PosePair(minReprojERightAuto, newPose2));
        std::cout << " e : " << minReprojERightAuto << std::endl;
        err = FT(0);
        plueckerCamR.pose(newPose2);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        start = double(cv::getTickCount());
        double minimalReprojectionErrorLeft1 = PnLpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, false);
        possiblePoses.push_back(PosePair(minimalReprojectionErrorLeft1, newPose1));
        err = FT(0);
        plueckerCamL.pose(newPose1);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        std::cout << "time: " << (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() << std::endl;

        double minimalReprojectionErrorRight1 = PnLpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, false);
        newPose2.concatInverse(slam->getCameraConfig(1));  // TODO: Check
        possiblePoses.push_back(PosePair(minimalReprojectionErrorRight1, newPose2));
        err = FT(0);
        plueckerCamR.pose(newPose2);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        R_and_Tpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, currentPose);
        double minReprojErrLeft = reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose3);
        possiblePoses.push_back(PosePair(minReprojErrLeft, newPose3));
        err = FT(0);
        plueckerCamL.pose(newPose3);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        R_and_Tpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, currentPose);
        double minReprojErrRight = reprojectionErrorPnL(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose4);
        newPose4.concatInverse(slam->getCameraConfig(1));  // TODO: Check
        possiblePoses.push_back(PosePair(minReprojErrRight, newPose4));
        err = FT(0);
        plueckerCamR.pose(newPose4);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        double minReprojErrCurrentPose = reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), currentPose);
        possiblePoses.push_back(PosePair(minReprojErrCurrentPose, currentPose));

        std::cout << "previous Pose: " << minReprojErrCurrentPose << std::setw(12) << currentPose.origin().x() << " " << currentPose.origin().y() << " " << currentPose.origin().z() << " " << std::endl;
        plueckerCamL.pose(currentPose);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        auto minPose = std::min_element(possiblePoses.begin(), possiblePoses.end(),[](const PosePair& p1, const PosePair& p2) { return p1.first <= p2.first; });
        if(minPose->first < 0.1){ // max reproj error
            currentPose = minPose->second;
            std::cout << "selected pose E: " << minPose->first << std::endl;
        }
        currentPose = cam0Gt;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin())->first << std::setw(12) << (possiblePoses.begin())->second.origin().x() << std::setw(12) << (possiblePoses.begin())->second.origin().y() << std::setw(12) << (possiblePoses.begin())->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin())->second.orientation().x() << std::setw(12) << (possiblePoses.begin())->second.orientation().y() << std::setw(12) << (possiblePoses.begin())->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+1)->first << std::setw(12) << (possiblePoses.begin()+1)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+1)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+1)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+1)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+1)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+1)->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+2)->first << std::setw(12) << (possiblePoses.begin()+2)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+2)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+2)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+2)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+2)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+2)->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+3)->first << std::setw(12) << (possiblePoses.begin()+3)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+3)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+3)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+3)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+3)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+3)->second.orientation().z() <<std::endl;

        std::cout << "GT:          " << cam0Gt.pose().origin().x() << std::setw(12) << cam0Gt.pose().origin().y() << std::setw(12) << cam0Gt.pose().origin().z() << std::setw(12)
                     << cam0Gt.pose().orientation().x() << std::setw(12) << cam0Gt.pose().orientation().y() << std::setw(12) << cam0Gt.pose().orientation().z() << std::setw(12) << std::endl;

        std::cout <<  reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), cam0Gt.pose()) << std::endl;

        std::cout <<  reprojectionErrorPnL(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), cam1Gt.pose()) << std::endl;


//        std::cout << "new current Pose: " << currentPose.origin() << std::endl;
//        std::cout << currentPose.orientation() << std::endl;


//        std::vector<lsfm::LineSegment3<FT>> test3dSegments;



        // matching Stereo LR -----------------------------------------------------------------------------
        std::vector<DMatch> good_matchesLR = lsfm::lineMatchingLsdAndCV(cam0img, cam1img, lsd0.lineSegments(), lsd1.lineSegments());
        std::vector<DMatch> good_matchesRL = lsfm::lineMatchingLsdAndCV(cam1img, cam0img, lsd1.lineSegments(), lsd0.lineSegments());
        std::vector<DMatch> matchesStereoRightLRcheck = lsfm::leftRightCheck(good_matchesLR, good_matchesRL);

        imshow("Detected stereo matches", drawKeyLineMatches<FT,DescriptorMatch<FT>>(cam0img, lsd0.lineSegments(), cam1img, lsd1.lineSegments(), matchesStereoRightLRcheck));


        std::cout << "triangulating lines" << std::endl;
        // triangulate new models -----------------------------------------------------------------
        for(int i = 0; i < matchesStereoRightLRcheck.size(); ++i){
            const int qIdx = matchesStereoRightLRcheck[i].queryIdx;
            const int tIdx = matchesStereoRightLRcheck[i].trainIdx;
            if(detectedLines0[qIdx].modelIndex >= 0 && detectedLines1[tIdx].modelIndex >= 0){
                // TODO: check if same model
                std::cout << "Line correspondence collision detected!" << std::endl;
                continue;
            }

            if(detectedLines0[qIdx].modelIndex >= 0 && detectedLines1[tIdx].modelIndex < 0){
//                detectedLines1[tIdx].modelIndex = detectedLines0[qIdx].modelIndex;
                // TODO: update detected Lines in slam model
                continue;
            }

            if(detectedLines0[qIdx].modelIndex < 0 && detectedLines1[tIdx].modelIndex >= 0){
//                detectedLines0[qIdx].modelIndex = detectedLines1[tIdx].modelIndex;
                // TODO: update detected Lines in slam model
                continue;
            }

            // continue because it was not possible to determine the corresponding GT line, adding without gt knowledge would lead to not being able to compare to GT
            if(lineIds0[qIdx] == -1 && lineIds1[tIdx] == -1){
                continue;
            }
            if(lineIds0[qIdx] >= 0 && lineIds1[tIdx] >= 0){
                if(lineIds0[qIdx] != lineIds1[tIdx]){
                    std::cout << "wrong match " << qIdx << "   " << tIdx << "  but still continuing to avoid gt influence" << std::endl;
//                    continue;
                }
            }

//            std::vector<int>::iterator it;
//            it = std::find(lineIds1.begin(), lineIds1.end(), lineIds0[i]);
//            if(it != lineIds1.end()){
//                int idx = it - lineIds1.begin();

                if(lsd0.lineSegments()[qIdx].normalY() > 0.985 || lsd0.lineSegments()[qIdx].normalY() < -0.985 ||
                   lsd1.lineSegments()[tIdx].normalY() > 0.985 ||  lsd1.lineSegments()[tIdx].normalY() < -0.985 //||
                   //lineIds0[i] == -1 || lineIds1[idx] == -1
                        )
                    continue;


                lsfm::Camera<FT> triangulationCamLeft(slam->getCameraConfig(0));
                lsfm::Camera<FT> triangulationCamRight(slam->getCameraConfig(1));

                //lsfm::Camera<FT> triangulationCamLeft(cam0Gt), triangulationCamRight(cam1Gt);
                triangulationCamLeft.concat(currentPose);
                triangulationCamRight.concat(currentPose);
                lsfm::StereoPlane<FT> modelStereo(triangulationCamLeft, triangulationCamRight);
                lsfm::LineSegment3<FT> modelLine3 = modelStereo.triangulate(lsd0.lineSegments()[qIdx], lsd1.lineSegments()[tIdx]);

//                test3dSegments.push_back(modelLine3);

                ModelLine<lsfm::Line3<FT>> modeledLine(modelLine3);
                slam->addData( qIdx, slam->getFrameNum()-2, tIdx, slam->getFrameNum()-1, modeledLine, lsfm::OBS_FLAGS_STEREO);

                if(lineIds0[qIdx] == -1){
                    gtToModelId[lineIds1[tIdx]] = slam->getModeledLines().size() - 1;
                } else {
                    gtToModelId[lineIds0[qIdx]] = slam->getModeledLines().size() - 1;
                }

//            }
        }


        slam->addData(currentPose);
        //slam->addData(cam0Gt.pose());
/*
        std::cout << "pre: " << std::endl;
        std::cout << slam->getRobotPoses().back().origin() << std::endl;
        std::cout << slam->getRobotPoses().back().orientation() << std::endl << std::endl;
*/
/*
        // BA
        std::vector<int> optiFrames;
        for(int i = 0; i < slam->getFrameNum(); ++i){
            optiFrames.push_back(i);
        }
        slam->bundleAdjustmentOnFramesFixed(optiFrames, 0);
*/
/*
        std::cout << "post: " << std::endl;
        std::cout << slam->getRobotPoses().back().origin() << std::endl;
        std::cout << slam->getRobotPoses().back().orientation() << std::endl << std::endl;
*/
        std::cout << "FrameNum: " << slam->getFrameNum() << std::endl;


        std::vector<LineSegment<FT>> models3d;
        lsfm::CameraPluecker<FT> camPL(cam0Gt);
        camPL.project(slam->getModelLineSegments(),models3d);

        cv::Mat cam0imageLines, cam1imageLines, modelImage;
        std::vector<std::string> lineModels0, lineModels1;
        std::transform(lineIds0.begin(), lineIds0.end(), std::back_inserter(lineModels0), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });
        std::transform(lineIds1.begin(), lineIds1.end(), std::back_inserter(lineModels1), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });
        cam0imageLines = drawLines<FT>(cam0img, lsd0.lineSegments(), lineIds0);
        cam1imageLines = drawLines<FT>(cam1img, lsd1.lineSegments(), lineIds1);
 //       cam0imageLines = drawLines<FT>(cam0img, lsd0.lineSegments());
 //       cam1imageLines = drawLines<FT>(cam1img, lsd1.lineSegments());
        modelImage = drawLines<FT>(cam0img, models3d);

        cv::imshow("cam0i", cam0imageLines);
      //  cv::imshow("cam1i", cam1imageLines);

//        cv::imshow("cam0m", modelImage);
/*
        std::vector<lsfm::LineSegment<FT>> models2d;
        lsfm::CameraCV<FT>camSegsProj(cam0Gt);
        camSegsProj.project(test3dSegments, models2d);
        cv::Mat lineSegmentsImg = lsfm::drawLines<FT>(cam0img, models2d);
        cv::imshow("triangulatedSegments", lineSegmentsImg);
*/


        cv::waitKey(5);
    }

    //! GT Correspondences
    template<class FT, class MySlam, class MapType>
    void lineProcessingGt(cv::Mat cam0img, cv::Mat cam1img, cv::Mat cam0GTimg, cv::Mat cam1GTimg, MapType & gtToModelId, MySlam * slam, lsfm::Pose<FT> & currentPose, lsfm::Camera<FT> cam0Gt = lsfm::Camera<FT>(), lsfm::Camera<FT> cam1Gt = lsfm::Camera<FT>()){

        lsfm::LsdEL<FT> lsd0(0.004, 0.03, 60, 4, 30, 10);   // EL_USE_NFA
        lsfm::LsdEL<FT> lsd1(0.004, 0.03, 60, 4, 30, 10);

        cv::Mat cam0grey, cam1grey;
        cvtColor(cam0img, cam0grey, CV_RGB2GRAY);
        cvtColor(cam1img, cam1grey, CV_RGB2GRAY);
        lsd0.detect(cam0grey);
        lsd1.detect(cam1grey);

        // Tracking
        std::vector<int> lineIds0, lineIds1;
        getIdsFromGT<FT>(lsd0.lineSegments(), cam0GTimg, lineIds0);
        getIdsFromGT<FT>(lsd1.lineSegments(), cam1GTimg, lineIds1);

        std::vector<DetectedLine> detectedLines0, detectedLines1;
        detectedLines0.assign(lsd0.lineSegments().size(), DetectedLine());
        detectedLines1.assign(lsd1.lineSegments().size(), DetectedLine());


        // add to model if model already exists
        for(int i = 0; i < lineIds0.size(); ++i){
            typename MapType::iterator it = gtToModelId.find(lineIds0[i]);
            if(it != gtToModelId.end()){
                detectedLines0[i].modelIndex = it->second;
            }
        }
        for(int i = 0; i < lineIds1.size(); ++i){
            typename MapType::iterator it = gtToModelId.find(lineIds1[i]);
            if(it != gtToModelId.end()){
                detectedLines1[i].modelIndex = it->second;
            }
        }

        slam->addData(lsd0.lineSegments(), detectedLines0);
        slam->addData(lsd1.lineSegments(), detectedLines1);
        slam->addData(cam0img);
        slam->addData(cam1img);


        // calculate position------------------------------------------------------------------------------

        lsfm::Camera<FT> tmpCamLeft(slam->getCameraConfig(0));
        lsfm::Camera<FT> tmpCamRight(slam->getCameraConfig(1));

        tmpCamLeft.concat(currentPose);
        tmpCamRight.concat(currentPose);

        std::vector<typename MySlam::line_segment_type> reprojSegsL, reprojSegsR;
        std::vector<typename MySlam::line_3d_type> modelLinesCamL, modelLinesCamR;


        for(int i = 0; i < detectedLines0.size(); ++i){
            if(detectedLines0[i].modelIndex >= 0){
                reprojSegsL.push_back(lsd0.lineSegments()[i]);
                modelLinesCamL.push_back(slam->getModeledLines()[detectedLines0[i].modelIndex].line);
            }
        }
        for(int i = 0; i < detectedLines1.size(); ++i){
            if(detectedLines1[i].modelIndex >= 0){
                reprojSegsR.push_back(lsd1.lineSegments()[i]);
                modelLinesCamR.push_back(slam->getModeledLines()[detectedLines1[i].modelIndex].line);
            }
        }


        lsfm::Pose<FT> newPose1, newPose2, newPose3, newPose4;
        typedef std::pair<FT, lsfm::Pose<FT>> PosePair;
        std::vector<PosePair> possiblePoses;
        lsfm::CameraPluecker<FT> plueckerCamL(tmpCamLeft), plueckerCamR(tmpCamRight);

        double start = double(cv::getTickCount());
        double minReprojELeftAuto = PnLpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, true);
        possiblePoses.push_back(PosePair(minReprojELeftAuto, newPose1));
        std::cout << " e : " << minReprojELeftAuto << std::endl;
        std::cout << "auto time: " << (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() << std::endl;
        FT err = FT(0);
        plueckerCamL.pose(newPose1);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        double minReprojERightAuto = PnLpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, true);
        possiblePoses.push_back(PosePair(minReprojERightAuto, newPose2));
        std::cout << " e : " << minReprojERightAuto << std::endl;
        err = FT(0);
        plueckerCamR.pose(newPose2);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        start = double(cv::getTickCount());
        double minimalReprojectionErrorLeft1 = PnLpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, false);
        possiblePoses.push_back(PosePair(minimalReprojectionErrorLeft1, newPose1));
        err = FT(0);
        plueckerCamL.pose(newPose1);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        std::cout << "time: " << (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() << std::endl;

        double minimalReprojectionErrorRight1 = PnLpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, false);
        newPose2.concatInverse(slam->getCameraConfig(1));  // TODO: Check
        possiblePoses.push_back(PosePair(minimalReprojectionErrorRight1, newPose2));
        err = FT(0);
        plueckerCamR.pose(newPose2);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        R_and_Tpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), currentPose, newPose3);
        double minReprojErrLeft = reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose3);
        possiblePoses.push_back(PosePair(minReprojErrLeft, newPose3));
        err = FT(0);
        plueckerCamL.pose(newPose3);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        R_and_Tpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), currentPose, newPose4);
        double minReprojErrRight = reprojectionErrorPnL(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose4);
        newPose4.concatInverse(slam->getCameraConfig(1));  // TODO: Check
        possiblePoses.push_back(PosePair(minReprojErrRight, newPose4));
        err = FT(0);
        plueckerCamR.pose(newPose4);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        double minReprojErrCurrentPose = reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), currentPose);
        possiblePoses.push_back(PosePair(minReprojErrCurrentPose, currentPose));

        std::cout << "previous Pose: " << minReprojErrCurrentPose << std::setw(12) << currentPose.origin().x() << " " << currentPose.origin().y() << " " << currentPose.origin().z() << " " << std::endl;
        plueckerCamL.pose(currentPose);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        auto minPose = std::min_element(possiblePoses.begin(), possiblePoses.end(),[](const PosePair& p1, const PosePair& p2) { return p1.first <= p2.first; });
        if(minPose->first < 0.1){ // max reproj error
            currentPose = minPose->second;
            std::cout << "selected pose E: " << minPose->first << std::endl;
        }
        currentPose = cam0Gt;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin())->first << std::setw(12) << (possiblePoses.begin())->second.origin().x() << std::setw(12) << (possiblePoses.begin())->second.origin().y() << std::setw(12) << (possiblePoses.begin())->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin())->second.orientation().x() << std::setw(12) << (possiblePoses.begin())->second.orientation().y() << std::setw(12) << (possiblePoses.begin())->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+1)->first << std::setw(12) << (possiblePoses.begin()+1)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+1)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+1)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+1)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+1)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+1)->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+2)->first << std::setw(12) << (possiblePoses.begin()+2)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+2)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+2)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+2)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+2)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+2)->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+3)->first << std::setw(12) << (possiblePoses.begin()+3)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+3)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+3)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+3)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+3)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+3)->second.orientation().z() <<std::endl;

        std::cout << "GT:          " << cam0Gt.pose().origin().x() << std::setw(12) << cam0Gt.pose().origin().y() << std::setw(12) << cam0Gt.pose().origin().z() << std::setw(12)
                     << cam0Gt.pose().orientation().x() << std::setw(12) << cam0Gt.pose().orientation().y() << std::setw(12) << cam0Gt.pose().orientation().z() << std::setw(12) << std::endl;

        std::cout <<  reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), cam0Gt.pose()) << std::endl;

        std::cout <<  reprojectionErrorPnL(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), cam1Gt.pose()) << std::endl;


//        std::cout << "new current Pose: " << currentPose.origin() << std::endl;
//        std::cout << currentPose.orientation() << std::endl;


        std::vector<lsfm::LineSegment3<FT>> test3dSegments;

        // triangulate new models -----------------------------------------------------------------
        for(int i = 0; i < lineIds0.size(); ++i){
            if(detectedLines0[i].modelIndex >= 0)
                continue;

            std::vector<int>::iterator it;
            it = std::find(lineIds1.begin(), lineIds1.end(), lineIds0[i]);
            if(it != lineIds1.end()){
                int idx = it - lineIds1.begin();

                if(lsd0.lineSegments()[i].normalY() > 0.985 || lsd0.lineSegments()[i].normalY() < -0.985 ||
                   lsd1.lineSegments()[idx].normalY() > 0.985 ||  lsd1.lineSegments()[idx].normalY() < -0.985 ||
                   lineIds0[i] == -1 || lineIds1[idx] == -1)
                    continue;


                lsfm::Camera<FT> triangulationCamLeft(slam->getCameraConfig(0));
                lsfm::Camera<FT> triangulationCamRight(slam->getCameraConfig(1));

                triangulationCamLeft.concat(currentPose);
                triangulationCamRight.concat(currentPose);
                lsfm::StereoPlane<FT> modelStereo(triangulationCamLeft, triangulationCamRight);
                lsfm::LineSegment3<FT> modelLine3 = modelStereo.triangulate(lsd0.lineSegments()[i], lsd1.lineSegments()[idx]);

                test3dSegments.push_back(modelLine3);

                ModelLine<lsfm::Line3<FT>> modeledLine(modelLine3);
                slam->addData( i, slam->getFrameNum()-2, idx, slam->getFrameNum()-1, modeledLine, lsfm::OBS_FLAGS_STEREO);

                gtToModelId[lineIds0[i]] = slam->getModeledLines().size() - 1;

            }
        }


        slam->addData(currentPose);
        //slam->addData(cam0Gt.pose());
/*
        std::cout << "pre: " << std::endl;
        std::cout << slam->getRobotPoses().back().origin() << std::endl;
        std::cout << slam->getRobotPoses().back().orientation() << std::endl << std::endl;
*/
/*
        // BA
        std::vector<int> optiFrames;
        for(int i = 0; i < slam->getFrameNum(); ++i){
            optiFrames.push_back(i);
        }
        slam->bundleAdjustmentOnFramesFixed(optiFrames, 0);
*/
/*
        std::cout << "post: " << std::endl;
        std::cout << slam->getRobotPoses().back().origin() << std::endl;
        std::cout << slam->getRobotPoses().back().orientation() << std::endl << std::endl;
*/
        std::cout << "FrameNum: " << slam->getFrameNum() << std::endl;


        std::vector<LineSegment<FT>> models3d;
        lsfm::CameraPluecker<FT> camPL(cam0Gt);
        camPL.project(slam->getModelLineSegments(),models3d);

        cv::Mat cam0imageLines, cam1imageLines, modelImage;
        std::vector<std::string> lineModels0, lineModels1;
        std::transform(lineIds0.begin(), lineIds0.end(), std::back_inserter(lineModels0), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });
        std::transform(lineIds1.begin(), lineIds1.end(), std::back_inserter(lineModels1), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });
        cam0imageLines = drawLines<FT>(cam0img, lsd0.lineSegments(), lineIds0);
        cam1imageLines = drawLines<FT>(cam1img, lsd1.lineSegments(), lineIds1);
 //       cam0imageLines = drawLines<FT>(cam0img, lsd0.lineSegments());
 //       cam1imageLines = drawLines<FT>(cam1img, lsd1.lineSegments());
        modelImage = drawLines<FT>(cam0img, models3d);

        cv::imshow("cam0i", cam0imageLines);
      //  cv::imshow("cam1i", cam1imageLines);

//        cv::imshow("cam0m", modelImage);
/*
        std::vector<lsfm::LineSegment<FT>> models2d;
        lsfm::CameraCV<FT>camSegsProj(cam0Gt);
        camSegsProj.project(test3dSegments, models2d);
        cv::Mat lineSegmentsImg = lsfm::drawLines<FT>(cam0img, models2d);
        cv::imshow("triangulatedSegments", lineSegmentsImg);
*/


        cv::waitKey(5);
    }

    template<class FT, class MySlam, class MapType>
    void lineProcessingAllGt(const std::vector<lsfm::LineSegment2<FT>> & ls0, const std::vector<lsfm::LineSegment2<FT>> & ls1, std::vector<int> lineIds0, std::vector<int> lineIds1, cv::Mat cam0img, cv::Mat cam1img, cv::Mat cam0GTimg, cv::Mat cam1GTimg, MapType & gtToModelId, MySlam * slam, lsfm::Pose<FT> & currentPose, lsfm::Camera<FT> cam0Gt = lsfm::Camera<FT>(), lsfm::Camera<FT> cam1Gt = lsfm::Camera<FT>()){

//        lsfm::LsdEL<FT> lsd0(0.004, 0.03, 60, 4, 30, 10);   // EL_USE_NFA
//        lsfm::LsdEL<FT> lsd1(0.004, 0.03, 60, 4, 30, 10);

        cv::Mat cam0grey, cam1grey;
        cvtColor(cam0img, cam0grey, CV_RGB2GRAY);
        cvtColor(cam1img, cam1grey, CV_RGB2GRAY);
//        lsd0.detect(cam0grey);
//        lsd1.detect(cam1grey);

        // Tracking
        //std::vector<int> lineIds0, lineIds1;
        //getIdsFromGT<FT>(ls0, cam0GTimg, lineIds0);
        //getIdsFromGT<FT>(ls1, cam1GTimg, lineIds1);

        std::vector<DetectedLine> detectedLines0, detectedLines1;
        detectedLines0.assign(ls0.size(), DetectedLine());
        detectedLines1.assign(ls1.size(), DetectedLine());


        // add to model if model already exists
        for(int i = 0; i < lineIds0.size(); ++i){
            typename MapType::iterator it = gtToModelId.find(lineIds0[i]);
            if(it != gtToModelId.end()){
                detectedLines0[i].modelIndex = it->second;
            }
        }
        for(int i = 0; i < lineIds1.size(); ++i){
            typename MapType::iterator it = gtToModelId.find(lineIds1[i]);
            if(it != gtToModelId.end()){
                detectedLines1[i].modelIndex = it->second;
            }
        }

        slam->addData(ls0, detectedLines0);
        slam->addData(ls1, detectedLines1);
        slam->addData(cam0img);
        slam->addData(cam1img);


        // calculate position------------------------------------------------------------------------------

        lsfm::Camera<FT> tmpCamLeft(slam->getCameraConfig(0));
        lsfm::Camera<FT> tmpCamRight(slam->getCameraConfig(1));

        tmpCamLeft.concat(currentPose);
        tmpCamRight.concat(currentPose);

        std::vector<typename MySlam::line_segment_type> reprojSegsL, reprojSegsR;
        std::vector<typename MySlam::line_3d_type> modelLinesCamL, modelLinesCamR;


        for(int i = 0; i < detectedLines0.size(); ++i){
            if(detectedLines0[i].modelIndex >= 0){
                reprojSegsL.push_back(ls0[i]);
                modelLinesCamL.push_back(slam->getModeledLines()[detectedLines0[i].modelIndex].line);
            }
        }
        for(int i = 0; i < detectedLines1.size(); ++i){
            if(detectedLines1[i].modelIndex >= 0){
                reprojSegsR.push_back(ls1[i]);
                modelLinesCamR.push_back(slam->getModeledLines()[detectedLines1[i].modelIndex].line);
            }
        }


        lsfm::Pose<FT> newPose1, newPose2, newPose3, newPose4;
        typedef std::pair<FT, lsfm::Pose<FT>> PosePair;
        std::vector<PosePair> possiblePoses;
        lsfm::CameraPluecker<FT> plueckerCamL(tmpCamLeft), plueckerCamR(tmpCamRight);

        double start = double(cv::getTickCount());
        double minReprojELeftAuto = PnLpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, true);
        possiblePoses.push_back(PosePair(minReprojELeftAuto, newPose1));
        std::cout << " e : " << minReprojELeftAuto << std::endl;
        std::cout << "auto time: " << (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() << std::endl;
        FT err = FT(0);
        plueckerCamL.pose(newPose1);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        double minReprojERightAuto = PnLpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, true);
        possiblePoses.push_back(PosePair(minReprojERightAuto, newPose2));
        std::cout << " e : " << minReprojERightAuto << std::endl;
        err = FT(0);
        plueckerCamR.pose(newPose2);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        start = double(cv::getTickCount());
        double minimalReprojectionErrorLeft1 = PnLpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, false);
        possiblePoses.push_back(PosePair(minimalReprojectionErrorLeft1, newPose1));
        err = FT(0);
        plueckerCamL.pose(newPose1);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        std::cout << "time: " << (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() << std::endl;

        double minimalReprojectionErrorRight1 = PnLpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, false);
        newPose2.concatInverse(slam->getCameraConfig(1));  // TODO: Check
        possiblePoses.push_back(PosePair(minimalReprojectionErrorRight1, newPose2));
        err = FT(0);
        plueckerCamR.pose(newPose2);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        R_and_Tpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), currentPose, newPose3);
        double minReprojErrLeft = reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose3);
        possiblePoses.push_back(PosePair(minReprojErrLeft, newPose3));
        err = FT(0);
        plueckerCamL.pose(newPose3);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        R_and_Tpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), currentPose, newPose4);
        double minReprojErrRight = reprojectionErrorPnL(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose4);
        newPose4.concatInverse(slam->getCameraConfig(1));  // TODO: Check
        possiblePoses.push_back(PosePair(minReprojErrRight, newPose4));
        err = FT(0);
        plueckerCamR.pose(newPose4);
        MySlam::reprojectionError(plueckerCamR, modelLinesCamR, reprojSegsR, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;


        double minReprojErrCurrentPose = reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), currentPose);
        possiblePoses.push_back(PosePair(minReprojErrCurrentPose, currentPose));

        std::cout << "previous Pose: " << minReprojErrCurrentPose << std::setw(12) << currentPose.origin().x() << " " << currentPose.origin().y() << " " << currentPose.origin().z() << " " << std::endl;
        plueckerCamL.pose(currentPose);
        MySlam::reprojectionError(plueckerCamL, modelLinesCamL, reprojSegsL, err);
        std::cout << "ecalc prev Pose: " << err << std::endl;

        auto minPose = std::min_element(possiblePoses.begin(), possiblePoses.end(),[](const PosePair& p1, const PosePair& p2) { return p1.first <= p2.first; });
        if(minPose->first < 0.1){ // max reproj error
            currentPose = minPose->second;
            std::cout << "selected pose E: " << minPose->first << std::endl;
        }
        currentPose = cam0Gt;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin())->first << std::setw(12) << (possiblePoses.begin())->second.origin().x() << std::setw(12) << (possiblePoses.begin())->second.origin().y() << std::setw(12) << (possiblePoses.begin())->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin())->second.orientation().x() << std::setw(12) << (possiblePoses.begin())->second.orientation().y() << std::setw(12) << (possiblePoses.begin())->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+1)->first << std::setw(12) << (possiblePoses.begin()+1)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+1)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+1)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+1)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+1)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+1)->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+2)->first << std::setw(12) << (possiblePoses.begin()+2)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+2)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+2)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+2)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+2)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+2)->second.orientation().z() <<std::endl;

        std::cout << " e : " << std::setw(5) << (possiblePoses.begin()+3)->first << std::setw(12) << (possiblePoses.begin()+3)->second.origin().x() << std::setw(12) << (possiblePoses.begin()+3)->second.origin().y() << std::setw(12) << (possiblePoses.begin()+3)->second.origin().z() <<
                     std::setw(12) << (possiblePoses.begin()+3)->second.orientation().x() << std::setw(12) << (possiblePoses.begin()+3)->second.orientation().y() << std::setw(12) << (possiblePoses.begin()+3)->second.orientation().z() <<std::endl;

        std::cout << "GT:          " << cam0Gt.pose().origin().x() << std::setw(12) << cam0Gt.pose().origin().y() << std::setw(12) << cam0Gt.pose().origin().z() << std::setw(12)
                     << cam0Gt.pose().orientation().x() << std::setw(12) << cam0Gt.pose().orientation().y() << std::setw(12) << cam0Gt.pose().orientation().z() << std::setw(12) << std::endl;

        std::cout <<  reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), cam0Gt.pose()) << std::endl;

        std::cout <<  reprojectionErrorPnL(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), cam1Gt.pose()) << std::endl;


//        std::cout << "new current Pose: " << currentPose.origin() << std::endl;
//        std::cout << currentPose.orientation() << std::endl;


        //std::vector<lsfm::LineSegment3<FT>> test3dSegments;

        // triangulate new models -----------------------------------------------------------------
        for(int i = 0; i < lineIds0.size(); ++i){
            if(detectedLines0[i].modelIndex >= 0)
                continue;

            std::vector<int>::iterator it;
            it = std::find(lineIds1.begin(), lineIds1.end(), lineIds0[i]);
            if(it != lineIds1.end()){
                int idx = it - lineIds1.begin();

                if(ls0.at(i).normalY() > 0.985 || ls0[i].normalY() < -0.985 ||
                   ls1[idx].normalY() > 0.985 ||  ls1[idx].normalY() < -0.985 ||
                   lineIds0[i] == -1 || lineIds1[idx] == -1)
                    continue;


                lsfm::Camera<FT> triangulationCamLeft(slam->getCameraConfig(0));
                lsfm::Camera<FT> triangulationCamRight(slam->getCameraConfig(1));

                //lsfm::Camera<FT> triangulationCamLeft(cam0Gt), triangulationCamRight(cam1Gt);
                triangulationCamLeft.concat(currentPose);
                triangulationCamRight.concat(currentPose);
                lsfm::StereoPlane<FT> modelStereo(triangulationCamLeft, triangulationCamRight);
                lsfm::LineSegment3<FT> modelLine3 = modelStereo.triangulate(ls0[i], ls1[idx]);

                // backproject and check error - if it is high -> use other projection method and check error -> still high? -> reject
                FT err = FT(0);
                slam->reprojectionError(lsfm::CameraPluecker<FT>(triangulationCamLeft), modelLine3, ls0[i], err);
                slam->reprojectionError(lsfm::CameraPluecker<FT>(triangulationCamRight), modelLine3, ls1[idx], err);

                if(err > 0.001){

                    //std::cout << "first backprojectionError: --------------------------------------------------- " << std::endl << err << "  id left:  " << i << std::endl;
                    // failed - try similar triangulation, only cutting the planes the other way around -> sometimes helps the cayley backprojection not to produce an error
                    lsfm::StereoPlane<FT> modelStereo2(triangulationCamRight, triangulationCamLeft);
                    modelLine3 = modelStereo2.triangulate(ls1[idx], ls0[i]);

                    // backproject and check error - if it is high -> cut the edges the other way around and check error -> still high? -> reject
                    FT err = FT(0);
                    slam->reprojectionError(lsfm::CameraPluecker<FT>(triangulationCamLeft), modelLine3, ls0[i], err);
                    slam->reprojectionError(lsfm::CameraPluecker<FT>(triangulationCamRight), modelLine3, ls1[idx], err);

                    // std::cout << "second backprojectionError: --------------------------------------------------- " << std::endl << err << "  id:  " << slam->getModeledLines().size() << std::endl;
                    if(err > 0.001){
                        //std::cout << "rejecting" << std::endl;
                        continue;
                    }
                }

                // test3dSegments.push_back(modelLine3);

                ModelLine<lsfm::Line3<FT>> modeledLine(modelLine3);
                slam->addData( i, slam->getFrameNum()-2, idx, slam->getFrameNum()-1, modeledLine, lsfm::OBS_FLAGS_STEREO);

                gtToModelId[lineIds0[i]] = slam->getModeledLines().size() - 1;

/*
                //planeFromLine(camL_.focal(),camL_.offset(),rotL_,camL_.origin(),lineL)
                Camera<FT> camL_(triangulationCamLeft),  camR_(triangulationCamRight);
                Matx33<FT> rotL_ = triangulationCamLeft.rotM(), rotR_ = triangulationCamRight.rotM();

                LineSegment<FT> lineL = ls0[i];
                LineSegment<FT> lineR = ls1[idx];

                Plane<FT> p1 = planeFromLine(camL_.focal(),camL_.offset(),rotL_,camL_.origin(), Line<FT>(lineL));
                Plane<FT> p2 = planeFromLine(camR_.focal(),camR_.offset(),rotR_,camR_.origin(), Line<FT>(lineR));


                Line3<FT> lTest = modelStereo.triangulate(static_cast<const Line<FT>>(lineL),static_cast<const Line<FT>>(lineR));

                lsfm::Vec3<FT> p1n_ = p1.normal(), p2n_ = p2.normal();
                FT p1d_ = p1.dist2origin(), p2d_ = p2.dist2origin();

                // inline bool intersection(const Plane<FT>& p, Line3<FT> &intl) const
                lsfm::Vec3<FT> u = p2n_.cross(p1n_);
                if (u.dot(u) <= LIMITS<FT>::tau())
                    std::cout << "problem!" << std::endl;

                // get point on line and both planes
                FT a = p2n_.dot(p1n_);
                FT n = 1 - a*a;
                lsfm::Vec3<FT> q = p2n_ * ((p2d_ - p1d_*a) / n) + p1n_ * ((p1d_ - p2d_*a)/n);
                Line3<FT> intl = Line3<FT>(q,u);

                FT err2 = 0;
                slam->reprojectionError(lsfm::CameraPluecker<FT>(triangulationCamLeft), intl, ls0[i], err2);
                slam->reprojectionError(lsfm::CameraPluecker<FT>(triangulationCamRight), intl, ls1[idx], err2);
                std::cout << "e21: " << err2 << std::endl;


                if(err > 0.01 ){ //|| err2 > 0.01
                    std::cout << "backprojectionError: --------------------------------------------------- " << std::endl << err;
                    std::cout << "  id:  " << slam->getModeledLines().size() - 1 << std::endl;
                }
                */

            }
        }


        slam->addData(currentPose);
        //slam->addData(cam0Gt.pose());
/*
        std::cout << "pre: " << std::endl;
        std::cout << slam->getRobotPoses().back().origin() << std::endl;
        std::cout << slam->getRobotPoses().back().orientation() << std::endl << std::endl;
*/
/*
        // BA
        std::vector<int> optiFrames;
        for(int i = 0; i < slam->getFrameNum(); ++i){
            optiFrames.push_back(i);
        }
        slam->bundleAdjustmentOnFramesFixed(optiFrames, 0);
*/
/*
        std::cout << "post: " << std::endl;
        std::cout << slam->getRobotPoses().back().origin() << std::endl;
        std::cout << slam->getRobotPoses().back().orientation() << std::endl << std::endl;
*/
        std::cout << "FrameNum: " << slam->getFrameNum() << std::endl;


        std::vector<LineSegment<FT>> models3d;
        lsfm::CameraPluecker<FT> camPL(cam0Gt);
        camPL.project(slam->getModelLineSegments(),models3d);

        cv::Mat cam0imageLines, cam1imageLines, modelImage;
        std::vector<std::string> lineModels0, lineModels1;
        std::transform(lineIds0.begin(), lineIds0.end(), std::back_inserter(lineModels0), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });
        std::transform(lineIds1.begin(), lineIds1.end(), std::back_inserter(lineModels1), [](const DetectedLine & dl) { return std::to_string(dl.modelIndex); });
        cam0imageLines = drawLines<FT>(cam0img, ls0, lineIds0);
        cam1imageLines = drawLines<FT>(cam1img, ls1, lineIds1);
 //       cam0imageLines = drawLines<FT>(cam0img, lsd0.lineSegments());
 //       cam1imageLines = drawLines<FT>(cam1img, lsd1.lineSegments());
        modelImage = drawLines<FT>(cam0img, models3d);

        cv::imshow("cam0i", cam0imageLines);
      //  cv::imshow("cam1i", cam1imageLines);

//        cv::imshow("cam0m", modelImage);
/*
        std::vector<lsfm::LineSegment<FT>> models2d;
        lsfm::CameraCV<FT>camSegsProj(cam0Gt);
        camSegsProj.project(test3dSegments, models2d);
        cv::Mat lineSegmentsImg = lsfm::drawLines<FT>(cam0img, models2d);
        cv::imshow("triangulatedSegments", lineSegmentsImg);
*/


        cv::waitKey(5);
    }

}

#endif
#endif
