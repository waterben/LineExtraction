#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define USE_CERES_JET
#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <slam/line_jet.hpp>
#include <line_tracker/stereo_line_analyzer.hpp>
#include <lfd/PairwiseLineMatcher.hpp>
#include <slam/pose_estimator.hpp>
#include <slam/slamDataModel.hpp>
#include <utility/camera_utilities.hpp>

#include <lfd/StereoLineFilter.hpp>
#include <lfd/StereoLineMatcher.hpp>
#include <lfd/MotionDescriptor.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <lfd/MotionLineMatcher.hpp>
#include <line_tracker/tracking_utilities.hpp>


#define NUM_CAMS 2
using namespace std;
using namespace lsfm;

typedef double FT;
typedef std::array<FT, 4> cayleyLine;
typedef lsfm::Line3<FT> MyLine3D;
typedef lsfm::LineSegment<FT> MyLine2;
//typedef LsdCC<FT>::LineSegment MyDetectorLine;
typedef std::vector<MyLine2> MyLineVec2;



int main(int argc, char** argv)
{
    std::string filename1 = "../../Datasets/euroc/t1_3_stereo.avi";
    std::string filename2 = "../../Datasets/euroc/intrinsics.yml";
    std::string filename3 = "../../Datasets/euroc/extrinsics.yml";

    if (argc > 2) {
        filename1 = argv[1];
        filename2 = argv[2];
        filename3 = argv[3];
    }

    cv::Mat frames1, im_grayLeft, im_grayRight;
    VideoCapture capture;
    capture = VideoCapture(filename1);
//    for(int i = 0; i < 450; ++i){     // fast forward
        capture.grab();
        capture.retrieve(frames1, IMREAD_GRAYSCALE);
//    }
    cvtColor(frames1,frames1,CV_RGB2GRAY);

    double scaleFactor = 1.0;
    Size img_size;
    img_size.width = (frames1.size().width / 2) * scaleFactor;
    img_size.height = frames1.size().height * scaleFactor;

    cv::Mat P1, P2, map11, map12, map21, map22;

    if (!lsfm::stereoRectification( filename2, filename3, img_size, P1, P2, map11, map12, map21, map22, scaleFactor)){
        cout << "Can not open files " << endl;
        return -1;
    }

    // find black borders
    cv::Mat borderDetect = cv::Mat::ones(frames1.rows, frames1.cols / 2, frames1.type());
    borderDetect *= 255;
    cv::Mat bordersRecti;
    remap(borderDetect, bordersRecti, map11, map12, INTER_LINEAR);      // Undistort
    std::vector<int> border;
    findBlackCroppingArea<uchar>(bordersRecti, border);

    borderDetect = cv::Mat::ones(frames1.rows, frames1.cols / 2, frames1.type());
    borderDetect *= 255;
    remap(borderDetect, bordersRecti, map21, map22, INTER_LINEAR);      // Undistort
    findBlackCroppingArea<uchar>(bordersRecti, border);

    int croppingHeight = std::max(border[0], std::max(border[1], std::max(border[4], border[5])));
    int croppingWidth = std::max(border[2], std::max(border[3], std::max(border[6], border[7])));
    std::cout << "croppingHeight: " << croppingHeight << std::endl;
    std::cout << "croppingWidth: " << croppingWidth << std::endl;

    //GaussianBlur(src1, src1, Size(3, 3), 0.6);

    // initialize Line Detector -------------------------------------------------------------------------

    LsdCC<FT> lsd1(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.012, 0.008, 30, 0, 2);
    LsdCC<FT> lsd2(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.012, 0.008, 30, 0, 2);

    im_grayLeft  = frames1(cv::Range(0, frames1.size().height), cv::Range(0, frames1.size().width/2)); //no data copying here - Range excludes the last value of width, thus correct.
    im_grayRight = frames1(cv::Range(0, frames1.size().height), cv::Range(frames1.size().width/2, frames1.size().width)); //no data copying here

//    im_grayLeft  = imread("/media/lange/Data/EurocBags/mav0/cam0/data/1403638127295097088.png", IMREAD_GRAYSCALE);
//    im_grayRight = imread("/media/lange/Data/EurocBags/mav0/cam1/data/1403638127295097088.png", IMREAD_GRAYSCALE);


    cv::Mat img1r, img2r;
    remap(im_grayLeft, img1r, map11, map12, INTER_LINEAR);      // Undistort
    remap(im_grayRight, img2r, map21, map22, INTER_LINEAR);

    im_grayLeft = img1r(Rect(croppingWidth,croppingHeight,img1r.size().width-croppingWidth*2,img1r.size().height-croppingHeight*2)).clone();
    im_grayRight = img2r(Rect(croppingWidth,croppingHeight,img2r.size().width-croppingWidth*2,img2r.size().height-croppingHeight*2)).clone();

    lsfm::Camera<FT> cameras[2];

    cameras[0] = lsfm::Camera<FT> (lsfm::Matx34<FT>(P1.ptr<double>()));
    cameras[1] = lsfm::Camera<FT> (lsfm::Matx34<FT>(P2.ptr<double>()));

    std::cout << cameras[1].pose().origin()[0] << "  " <<  cameras[1].pose().origin()[1] << "  " <<  cameras[1].pose().origin()[2] << "  " << std::endl;

    typedef lsfm::SlamModel<FT, NUM_CAMS, MyLine2, MyLine3D> MySlam;
    MySlam slam(cameras);    // change lineDescriptor if needed

    typedef std::vector<DescriptorMatch<FT>> MatchType;
    MatchType motionMatchesLeft, motionMatchesRight;
    MyLineVec2 previousLeftLines, previousRightLines;

    cv::Mat previousImGrayLeft = im_grayLeft;
    cv::Mat previousImGrayRight = im_grayRight;

    lsfm::Pose<FT> currentPose;
    int last_key_press = 0;
    int cycleNr = 0;
    while (last_key_press != 'q')
    {

        // stereo undistort and rectify  ---------------------------------------
        im_grayLeft  = frames1(cv::Range(0, frames1.size().height), cv::Range(0, frames1.size().width/2)); //no data copying here - Range excludes the last value of width, so correct.
        im_grayRight = frames1(cv::Range(0, frames1.size().height), cv::Range(frames1.size().width/2, frames1.size().width)); //no data copying here

//        im_grayLeft  = imread("/media/lange/Data/EurocBags/mav0/cam0/data/1403638127295097088.png", IMREAD_GRAYSCALE);
//        im_grayRight = imread("/media/lange/Data/EurocBags/mav0/cam1/data/1403638127295097088.png", IMREAD_GRAYSCALE);

/*
        resize(im_grayLeft, im_grayLeft, Size(im_grayLeft.size().width * scaleFactor, im_grayLeft.size().height * scaleFactor));
        resize(im_grayRight, im_grayRight, Size(im_grayRight.size().width * scaleFactor, im_grayRight.size().height * scaleFactor));
*/
        cv::Mat img1r, img2r;
        remap(im_grayLeft, img1r, map11, map12, INTER_LINEAR);      // Undistort
        remap(im_grayRight, img2r, map21, map22, INTER_LINEAR);

        im_grayLeft = img1r(Rect(croppingWidth,croppingHeight,img1r.size().width-croppingWidth*2,img1r.size().height-croppingHeight*2)).clone();
        im_grayRight = img2r(Rect(croppingWidth,croppingHeight,img2r.size().width-croppingWidth*2,img2r.size().height-croppingHeight*2)).clone();


        // line detection ---------------------------------------
        lsd1.detect(im_grayLeft);
        lsd2.detect(im_grayRight);

        std::vector<MyLine2> currentLeftLines, currentRightLines;
        for(int i = 0; i < lsd1.lineSegments().size(); ++i){
            currentLeftLines.push_back(lsd1.lineSegments()[i]);
        }
        for(int i = 0; i < lsd2.lineSegments().size(); ++i){
            currentRightLines.push_back(lsd2.lineSegments()[i]);
        }


        std::cout << "lines left: " << lsd1.lineSegments().size() << ", lines right: " << lsd2.lineSegments().size() << std::endl;
        std::vector<DetectedLine> currentDetectedLeftLines, currentDetectedRightLines;
        currentDetectedLeftLines.assign(lsd1.lineSegments().size(), DetectedLine());
        currentDetectedRightLines.assign(lsd2.lineSegments().size(), DetectedLine());

        MatMap dataL, dataR;
        dataL["gx"] = lsd1.imageData()[0];
        dataL["gy"] = lsd1.imageData()[1];
        dataL["img"] = im_grayLeft;

        dataR["gx"] = lsd2.imageData()[0];
        dataR["gy"] = lsd2.imageData()[1];
        dataR["img"] = im_grayRight;

        // track lines (filter candidates, then match the remaining candidates) ---------------------------------------

        std::vector<DetectedLine> previouslyDetectedLinesLeft, previouslyDetectedLinesRight;
        int modeledLinesCount = slam.getModeledLines().size();

        if(cycleNr > 0){
            previousLeftLines = slam.getLineSegments(slam.getFrameNr(cycleNr - 1, 0));
            previousRightLines = slam.getLineSegments(slam.getFrameNr(cycleNr - 1, 1));

            previouslyDetectedLinesLeft = slam.getDetectedLines(slam.getFrameNr(cycleNr - 1, 0));
            previouslyDetectedLinesRight = slam.getDetectedLines(slam.getFrameNr(cycleNr - 1, 1));
        }

        typedef FdcMotion<FT, MyLine2> MyMotionFdc;
        MyMotionFdc::FdcPtr fdcM1 = MyMotionFdc::createFdc();
        MyMotionFdc::FdcPtr fdcM2 = MyMotionFdc::createFdc();

        std::vector<int> leftModelIDs, rightModelIDs;

        double start = double(getTickCount());
        MotionLineMatcher<FT, MyLineVec2, MyMotionFdc> mlm(fdcM1, fdcM2, im_grayLeft.cols,  im_grayLeft.rows);
        if(cycleNr - 1 >= 0){
            lsfm::matchToLineIDs<FT>(motionMatchesLeft, currentLeftLines, previousLeftLines, mlm);
            lsfm::matchToLineIDs<FT>(motionMatchesRight, currentRightLines, previousRightLines, mlm);


            leftModelIDs = matchToModelIDs<MatchType>(currentLeftLines, slam, mlm, cycleNr, 0, currentPose, 10);
            rightModelIDs = matchToModelIDs<MatchType>(currentRightLines, slam, mlm, cycleNr, 1, currentPose, 10);
        }

/*
        PairwiseLineMatcher<FT, typename LbdFdc::descriptor_type> pmatcher();
        if(motionMatches.size() > 1)
            pmatcher.match1D(lsd1.lineSegments(), lsd2.lineSegments(), mlm.getDscNew(), mlm.getDscPrev(), motionMatches, motionMatchesRes);
*/

        double end = double(getTickCount());
        std::cout <<  "mlm matches: " << motionMatchesLeft.size() << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;
//        std::cout <<  "mlmRight matches: " << motionMatchesRight.size() << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;

        std::vector<int> outputLeftMotionModelIds, outputRightMotionModelIds;
        outputLeftMotionModelIds.assign(currentLeftLines.size(), -1);
        outputRightMotionModelIds.assign(currentRightLines.size(), -1);

        for(int i = 0; i < leftModelIDs.size(); ++i){
            currentDetectedLeftLines.at(i).modelIndex = leftModelIDs[i];
            outputLeftMotionModelIds.at(i) = leftModelIDs[i];
        }
        for(int i = 0; i < rightModelIDs.size(); ++i){
            currentDetectedRightLines.at(i).modelIndex = rightModelIDs[i];
            outputRightMotionModelIds.at(i) = rightModelIDs[i];
        }

/*
        for_each(motionMatchesLeft.begin(), motionMatchesLeft.end(), [&](const DescriptorMatch<FT>& v){
                currentDetectedLeftLines.at(v.queryIdx).modelIndex = previouslyDetectedLinesLeft.at(v.matchIdx).modelIndex;
                outputLeftMotionModelIds.at(v.queryIdx) = (previouslyDetectedLinesLeft.at(v.matchIdx).modelIndex);
        });

        for_each(motionMatchesRight.begin(), motionMatchesRight.end(), [&](const DescriptorMatch<FT>& v){
                currentDetectedRightLines.at(v.queryIdx).modelIndex = previouslyDetectedLinesRight.at(v.matchIdx).modelIndex;
                outputRightMotionModelIds.at(v.queryIdx) = (previouslyDetectedLinesRight.at(v.matchIdx).modelIndex);
        });
*/

        slam.addData(currentLeftLines, currentDetectedLeftLines);
        slam.addData(currentRightLines, currentDetectedRightLines);

        // estimate position ------------------------------------------------------------------------------------------------
        // left Side ---

        std::vector<MyLine2> reprojSegsL, reprojSegsR;
        std::vector<MyLine3D> modelLinesCamL, modelLinesCamR;
/*
        // fill models
        for(int i = 0; i < currentDetectedLeftLines.size(); ++i){
            if(currentDetectedLeftLines.at(i).modelIndex>0){
                modelLinesCamL.push_back(slam.getModeledLines()[currentDetectedLeftLines.at(i).modelIndex].line);
                reprojSegsL.push_back(currentLeftLines.at(i));
            }
        }

        for(int i = 0; i < currentDetectedRightLines.size(); ++i){
            if(currentDetectedRightLines.at(i).modelIndex>0){
                modelLinesCamR.push_back(slam.getModeledLines()[currentDetectedRightLines.at(i).modelIndex].line);
                reprojSegsR.push_back(currentRightLines.at(i));
            }
        }
*/

        std::vector<int> goodLinesL = slam.goodLinesToTrack(25, slam.getFrameNr(cycleNr, 0));
        for_each(goodLinesL.begin(), goodLinesL.end(), [&](int lNum){
//            goodModelsL.push_back(currentDetectedLeftLines[lNum].modelIndex);

            modelLinesCamL.push_back(slam.getModeledLines()[currentDetectedLeftLines.at(lNum).modelIndex].line);
            reprojSegsL.push_back(currentLeftLines.at(lNum));
        });

        std::vector<int> goodLinesR = slam.goodLinesToTrack(25, slam.getFrameNr(cycleNr, 1));
        for_each(goodLinesR.begin(), goodLinesR.end(), [&](int lNum){
//            goodModelsL.push_back(currentDetectedLeftLines[lNum].modelIndex);

            modelLinesCamR.push_back(slam.getModeledLines()[currentDetectedRightLines.at(lNum).modelIndex].line);
            reprojSegsR.push_back(currentRightLines.at(lNum));
        });


        lsfm::Camera<FT> tmpCamLeft(cameras[0]);
        lsfm::Camera<FT> tmpCamRight(cameras[1]);

        tmpCamLeft.concat(currentPose);
        tmpCamRight.concat(currentPose);

        lsfm::Pose<double> newPose1, newPose2, newPose3, newPose4;
        typedef std::pair<double, lsfm::Pose<double>> PosePair;
        std::vector<PosePair> possiblePoses;

        double minimalReprojectionErrorLeft1 = PnLpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose1, false);
        possiblePoses.push_back(PosePair(minimalReprojectionErrorLeft1, newPose1));

        double minimalReprojectionErrorRight1 = PnLpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose2, false);
        newPose2.concatInverse(cameras[1]);
        possiblePoses.push_back(PosePair(minimalReprojectionErrorRight1, newPose2));

        R_and_Tpose(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), tmpCamLeft, newPose3);
        double minReprojErrLeft = reprojectionErrorPnL(reprojSegsL, modelLinesCamL, tmpCamLeft.camM(), newPose3);
        possiblePoses.push_back(PosePair(minReprojErrLeft, newPose3));

        R_and_Tpose(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), tmpCamRight, newPose4);
        double minReprojErrRight = reprojectionErrorPnL(reprojSegsR, modelLinesCamR, tmpCamRight.camM(), newPose4);
        newPose4.concatInverse(cameras[1]);
        possiblePoses.push_back(PosePair(minReprojErrRight, newPose4));

        auto minPose = std::min_element(possiblePoses.begin(), possiblePoses.end(),[](const PosePair& p1, const PosePair& p2) { return p1.first <= p2.first; });
        if(minPose->first < 10) // max reproj error
            currentPose = minPose->second;
/*
        std::cout << " e : " << (possiblePoses.begin())->first << std::endl << "pose : " << (possiblePoses.begin())->second.origin() << std::endl;
        std::cout << " e : " << (possiblePoses.begin()+1)->first << std::endl << "pose : " << (possiblePoses.begin()+1)->second.origin()<< std::endl;
        std::cout << " e : " << (possiblePoses.begin()+2)->first << std::endl << "pose : " << (possiblePoses.begin()+2)->second.origin() << std::endl;
        std::cout << " e : " << (possiblePoses.begin()+3)->first  << std::endl << "pose : " << (possiblePoses.begin()+3)->second.origin() << std::endl;
*/
        std::cout << " e : " << (possiblePoses.begin())->first << std::endl;
        std::cout << " e : " << (possiblePoses.begin()+1)->first << std::endl;
        std::cout << " e : " << (possiblePoses.begin()+2)->first << std::endl;
        std::cout << " e : " << (possiblePoses.begin()+3)->first << std::endl;

        std::cout << "new current Pose: " << currentPose.origin() << std::endl;

        // Stereo matching ---------------------------------------------------------
        typedef GchGradImgInterpolate<FT> MyGchHelper;
        typedef FdcGenericLR<FT,LsdCC<FT>::LineSegment,MyGchHelper> MyFdc;


        typename lsfm::LsdCC<FT>::LineSegmentVector currentLinesLsdL = lsd1.lineSegments();
        typename lsfm::LsdCC<FT>::LineSegmentVector currentLinesLsdR = lsd2.lineSegments();

        MyFdc::FdcPtr fdcL = MyFdc::createFdc(dataL);
        MyFdc::FdcPtr fdcR = MyFdc::createFdc(dataR);

        imwrite( "/home/lange/tmp/lsMx.bmp", dataR["gx"] );
        imwrite( "/home/lange/tmp/lsMy.bmp", dataR["gy"] );
        imwrite( "/home/lange/tmp/lsMi.bmp", dataR["img"] );

        StereoLineMatcher<FT, LsdCC<FT>::LineSegmentVector,MyGchHelper> slm(fdcL, fdcR, im_grayLeft.rows, im_grayLeft.cols / 2,5,0.6);

        std::vector<DescriptorMatch<FT>> bfmatches3;
        slm.match(lsd1.lineSegments(),lsd2.lineSegments(),bfmatches3);


        // triangulate new stereo lines ---------------------------------------

        std::vector<ModelLine<MyLine3D>> lineModels;

        std::vector<int> outputLeftStereoModelIds;    // modeledLineIdsLeft
        outputLeftStereoModelIds.assign(currentLeftLines.size(), -1);
//        modeledLineIdsLeft.assign(lsd1.lineSegments().size(), -1);

        for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<FT>& v){
//            triangulateLine(&projLeft, &projRight, lsd1.lineSegments().at(v.queryIdx), lsd2.lineSegments().at(v.matchIdx), startPoint3d, endPoint3d);


            // /////////////////////////////////////////////////////////
            if(currentDetectedLeftLines.at(v.queryIdx).modelIndex > 0 ||  currentDetectedRightLines.at(v.matchIdx).modelIndex > 0){
                if((currentDetectedLeftLines.at(v.queryIdx).modelIndex != currentDetectedRightLines.at(v.matchIdx).modelIndex)
                   && currentDetectedLeftLines.at(v.queryIdx).modelIndex > 0 &&  currentDetectedRightLines.at(v.matchIdx).modelIndex > 0){

                    // TODO: Merging from Stereo Correspondence

                    std::cout << "Doppelzuordnung, id: " << currentDetectedLeftLines.at(v.queryIdx).modelIndex << "  und: " << currentDetectedRightLines.at(v.matchIdx).modelIndex << std::endl;
                    outputLeftStereoModelIds.at(v.queryIdx)=(currentDetectedRightLines.at(v.matchIdx).modelIndex);

                } else if(currentDetectedLeftLines.at(v.queryIdx).modelIndex > 0){
                    currentDetectedRightLines.at(v.matchIdx).modelIndex = currentDetectedLeftLines.at(v.queryIdx).modelIndex;
                    slam.getDetectedLine(slam.getFrameNum() - 1, v.matchIdx).modelIndex = currentDetectedLeftLines.at(v.queryIdx).modelIndex;

                    outputLeftStereoModelIds.at(v.queryIdx)=(currentDetectedLeftLines.at(v.queryIdx).modelIndex);

                } else if(currentDetectedRightLines.at(v.matchIdx).modelIndex > 0){
                    currentDetectedLeftLines.at(v.queryIdx).modelIndex = currentDetectedRightLines.at(v.matchIdx).modelIndex;
                    slam.getDetectedLine(slam.getFrameNum() - 2, v.queryIdx).modelIndex = currentDetectedRightLines.at(v.matchIdx).modelIndex;

                    outputLeftStereoModelIds.at(v.queryIdx)=(currentDetectedRightLines.at(v.matchIdx).modelIndex);
                }
            } else {
                // Triangulate new Model ---

                lsfm::Camera<FT> tmpCamLeft(cameras[0]), tmpCamRight(cameras[1]);
                tmpCamLeft.concat(currentPose);
                tmpCamRight.concat(currentPose);
                lsfm::StereoPlane<FT> testStereo(tmpCamLeft, tmpCamRight);
                lsfm::LineSegment3<FT> testLine3 = testStereo.triangulate(lsd1.lineSegments().at(v.queryIdx), lsd2.lineSegments().at(v.matchIdx));

                if(!testLine3.empty()){
                    currentDetectedLeftLines.at(v.queryIdx).modelIndex = modeledLinesCount;
                    currentDetectedRightLines.at(v.matchIdx).modelIndex = modeledLinesCount;

//                    modeledLineIdsLeft.at(v.queryIdx) = modeledLinesCount;

                    outputLeftStereoModelIds.at(v.queryIdx)=(modeledLinesCount);

                    std::cout << "model: " << modeledLinesCount << " leftNr: " << v.queryIdx << "  rightNr: " << v.matchIdx << std::endl;

                    ++modeledLinesCount;

                    ModelLine<MyLine3D> modeledLine(testLine3);
                    lineModels.push_back(modeledLine);

                    slam.addData( v.queryIdx, slam.getFrameNum()-2, v.matchIdx, slam.getFrameNum()-1, modeledLine, lsfm::OBS_FLAGS_STEREO);
                } else {
                    outputLeftStereoModelIds.at(v.queryIdx)=(-1);
                }
            }
        });


        // triangulate lines from tracking ------------------------------------------------------------------
/*        if(cycleNr > 0){
            for_each(motionMatchesLeft.begin(), motionMatchesLeft.end(), [&](const DescriptorMatch<FT>& v){
                if(previouslyDetectedLinesLeft.at(v.matchIdx).modelIndex < 1){
                    if(currentDetectedLeftLines.at(v.queryIdx).modelIndex > 0){
                        // if it was just now stereo-triangulated
                        previouslyDetectedLinesLeft.at(v.matchIdx).modelIndex = currentDetectedLeftLines.at(v.queryIdx).modelIndex;
                    } else {
                        // triangulate new model from tracking only
                        CameraPluecker<FT> newCam(cameras[0]), previousCam(cameras[0]);
                        newCam.concat(currentPose);
                        previousCam.concat(slam.getRobotPoses().at(cycleNr - 1));
                        lsfm::StereoPlane<FT> triangulateTracking(newCam, previousCam);

                        lsfm::LineSegment3<FT> testLine3 = triangulateTracking.triangulate(currentLeftLines.at(v.queryIdx), previousLeftLines.at(v.matchIdx));

                        currentDetectedLeftLines.at(v.queryIdx) = modeledLinesCount;
                        //previouslyDetectedLinesLeft.at(v.matchIdx) = modeledLinesCount;   // TODO add into slam model

                        modeledLineIdsLeft.at(v.queryIdx) = modeledLinesCount;  // TODO: überflüssig?

                        ++modeledLinesCount;

                        std::vector<Observation> obsList;
            //            ModelLine<MyLine3D> modeledLine(LineSegment3<FT> (lsfm::Vec3<FT>(startPoint3d.x, startPoint3d.y, startPoint3d.z), lsfm::Vec3<FT>(endPoint3d.x, endPoint3d.y, endPoint3d.z)), obsList);
                        ModelLine<MyLine3D> modeledLine(testLine3, obsList);
                        lineModels.push_back(modeledLine);

                    }
                }
            });

            for_each(motionMatchesRight.begin(), motionMatchesRight.end(), [&](const DescriptorMatch<FT>& v){
                if(previouslyDetectedLinesRight.at(v.matchIdx).modelIndex < 1){
                    if(currentDetectedRightLines.at(v.queryIdx).modelIndex > 0){
                        // if it was just now stereo-triangulated
                        previouslyDetectedLinesRight.at(v.matchIdx).modelIndex = currentDetectedRightLines.at(v.queryIdx).modelIndex;
                    } else {
                        // triangulate new model from tracking only
                        CameraPluecker<FT> newCam(cameras[1]), previousCam(cameras[1]);
                        newCam.concat(currentPose);
                        previousCam.concat(slam.getRobotPoses().at(cycleNr - 1));
                        lsfm::StereoPlane<FT> triangulateTracking(newCam, previousCam);

                        lsfm::LineSegment3<FT> testLine3 = triangulateTracking.triangulate(currentRightLines.at(v.queryIdx), previousRightLines.at(v.matchIdx));

                        currentDetectedRightLines.at(v.queryIdx) = modeledLinesCount;
                        //previouslyDetectedLinesRight.at(v.matchIdx) = modeledLinesCount;   // TODO add into slam model

//                        modeledLineIdsRight.at(v.queryIdx) = modeledLinesCount;  // TODO: überflüssig?

                        ++modeledLinesCount;

                        std::vector<Observation> obsList;
            //            ModelLine<MyLine3D> modeledLine(LineSegment3<FT> (lsfm::Vec3<FT>(startPoint3d.x, startPoint3d.y, startPoint3d.z), lsfm::Vec3<FT>(endPoint3d.x, endPoint3d.y, endPoint3d.z)), obsList);
                        ModelLine<MyLine3D> modeledLine(testLine3, obsList);
                        lineModels.push_back(modeledLine);

                    }
                }
            });
        }
*/
        // add to slam data model ---------------------------------------

        slam.addData(currentPose);

        cv::Mat linesPrevLeft = previousImGrayLeft;
        cv::Mat linesLeft = im_grayLeft.clone();
        cv::Mat linesLeft2 = im_grayLeft.clone();
        cv::Mat linesRight = im_grayRight.clone();
        cv::Mat linesRight2 = im_grayRight.clone();
        cv::Mat lineModelSegmentsLeft = im_grayLeft.clone();

        std::vector<std::string> leftLineModels;
        for(int i = 0; i < currentDetectedLeftLines.size(); ++i){
            std::string s = "";
            if(currentDetectedLeftLines[i].modelIndex > -1)
                s = std::to_string(currentDetectedLeftLines[i].modelIndex);
            leftLineModels.push_back(s);
        }

        drawLines<FT>(linesLeft, currentLeftLines, leftLineModels);
        drawLines<FT>(linesRight, lsd2.lineSegments());
        drawLines<FT>(linesPrevLeft, previousLeftLines);
        imshow("leftLines", linesLeft);

        std::vector<string> leftLineIds;
        for(int i = 0; i < currentLeftLines.size(); i++)
            leftLineIds.push_back(std::to_string(i));
        drawLines<FT>(linesLeft2, currentLeftLines, leftLineIds);
        imshow("leftLines2", linesLeft2);

        std::vector<string> rightLineIds;
        for(int i = 0; i < currentRightLines.size(); i++)
            rightLineIds.push_back(std::to_string(i));
        drawLines<FT>(linesRight2, currentRightLines, rightLineIds);
        imshow("linesRight2", linesRight2);
//        imshow("rightLines", linesRight);
//        imshow("prevLeftLines", linesPrevLeft);

        std::cout <<  "final matches: " << bfmatches3.size() << std::endl;
        imshow("Detected Stereo matches", drawMatches<FT,DescriptorMatch<FT>>(im_grayLeft, lsd1.lineSegments(), im_grayRight, lsd2.lineSegments(), bfmatches3, outputLeftStereoModelIds));


        imshow("Detected tracking matches Left", drawMatches<FT,DescriptorMatch<FT>, MyLineVec2>(im_grayLeft, currentLeftLines, previousImGrayLeft, previousLeftLines, motionMatchesLeft, outputLeftMotionModelIds));
        imshow("Detected tracking matches Right", drawMatches<FT,DescriptorMatch<FT>, MyLineVec2>(im_grayRight, currentRightLines, previousImGrayRight, previousRightLines, motionMatchesRight, outputRightMotionModelIds));

        previousImGrayLeft = im_grayLeft;
        previousImGrayRight = im_grayRight;
        previousLeftLines = currentLeftLines;
        previousRightLines = currentRightLines;

        // run bundle adjustment --------------------------------

        if(cycleNr > 0){ // (!(cycleNr%5)) &&
/*
            std::vector<int> optiModels, optiFrames;

            std::vector<int> goodLinesL = slam.goodLinesToTrack(25, slam.getFrameNr(cycleNr, 0));
            for_each(goodLinesL.begin(), goodLinesL.end(), [&](int lNum){
                if(currentDetectedLeftLines[lNum].modelIndex >= 0)
                    optiModels.push_back(currentDetectedLeftLines[lNum].modelIndex);
            });

            std::vector<int> goodLinesR = slam.goodLinesToTrack(25, slam.getFrameNr(cycleNr, 1));
            for_each(goodLinesR.begin(), goodLinesR.end(), [&](int lNum){
                if(std::find(optiModels.begin(), optiModels.end(), currentDetectedRightLines[lNum].modelIndex) == optiModels.end()
                        && currentDetectedRightLines[lNum].modelIndex >= 0)
                    optiModels.push_back(currentDetectedRightLines[lNum].modelIndex);
            });

            for(int i = 1; i < 31 && (slam.getFrameNum() - i >= 0); ++i)
                optiFrames.push_back(slam.getFrameNum() - i);


//            slam.bundleAdjustmentOnFrames(optiFrames, optiModels);

            currentPose = slam.getRobotPoses().back();

            for(auto opModel : optiModels)
                slam.outlierRemoveOnModel(opModel);
*/
        }


        std::cout << "Num Frames: " << slam.getFrameNum() << std::endl;
/*
        std::vector<int> fms;
        for(int i = 0; i < slam.getFrameNum(); ++i)
            fms.push_back(i);
        slam.bundleAdjustmentOnFrames(fms);
*/
        // find and print good models to track
        std::vector<int> goodModelsLeft, goodLinesLeft = slam.goodLinesToTrack(25, slam.getFrameNr(cycleNr, 0));
        for_each(goodLinesLeft.begin(), goodLinesLeft.end(), [&](int lNum){
            goodModelsLeft.push_back(currentDetectedLeftLines[lNum].modelIndex);
        });
        std::stringstream result;
        std::copy(goodModelsLeft.begin(), goodModelsLeft.end(), std::ostream_iterator<int>(result, " "));
        std::cout << "good lines left: " << result.str().c_str() << std::endl;

        result.str("");
        std::vector<int> goodModelsRight, goodLinesRight = slam.goodLinesToTrack(25, slam.getFrameNr(cycleNr, 1));
        for_each(goodLinesRight.begin(), goodLinesRight.end(), [&](int lNum){
            goodModelsRight.push_back(currentDetectedRightLines[lNum].modelIndex);
        });
        std::copy(goodModelsRight.begin(), goodModelsRight.end(), std::ostream_iterator<int>(result, " "));
        std::cout << "good lines right: " << result.str().c_str() << std::endl;


        std::cout << "Errors: " << std::endl;
        for_each(goodModelsLeft.begin(), goodModelsLeft.end(), [&](int mNum){
            FT var = 0, maxE = 0, e = slam.reprojectionErrorModel(mNum, var, maxE);
            std::cout << "  ID: "  << mNum << " meanE: " << e << " var: " << var  << " std: " << std::sqrt(var) << " maxE: " << maxE
                      << " numObs: " << slam.getModeledLines()[mNum].observations.size()
                      << " numStereo: " << slam.getModeledLines()[mNum].stereoObs().size() << std::endl;
        });
        std::cout << std::endl;


        std::vector<LineSegment3<FT>> goodSegments;
        std::vector<String> goodSegmentIds;
        for(int i = 0; i < goodModelsLeft.size(); ++i){
            goodSegments.push_back(slam.getModelLineSegment(goodModelsLeft[i]));
            goodSegmentIds.push_back(std::to_string(goodModelsLeft[i]));
        }
        std::vector<LineSegment<FT>> goodSegments2d;
        lsfm::CameraPluecker<FT> camPL(tmpCamLeft);
        camPL.project(goodSegments,goodSegments2d);
        drawLinesT<FT>(lineModelSegmentsLeft, goodSegments2d, goodSegmentIds);
        imshow("lineModelSegmentsLeft", lineModelSegmentsLeft);

        //cam.project(data,res);


        // get new frame ---------------------------------------
        if(!capture.grab())
            break;
        capture.retrieve(frames1, IMREAD_GRAYSCALE);
        cvtColor(frames1,frames1,CV_RGB2GRAY);

        std::cout << "cycleNr: " << cycleNr << std::endl;
        cv::waitKey(10000000);
        last_key_press = cv::waitKey(1)%256;
        ++cycleNr;
    }



    return 0;
}
