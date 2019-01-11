#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define USE_CERES_JET

#include <lsd/lsd_cc.hpp>
#include <geometry/draw.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <slam/line_jet.hpp>
#include <utility/camera_utilities.hpp>
#include <line_tracker/stereo_line_analyzer.hpp>
#include <slam/pose_estimator.hpp>
#include <slam/slamDataModel.hpp>

#include <lfd/StereoLineFilter.hpp>
#include <lfd/StereoLineMatcher.hpp>
#include <lfd/MotionDescriptor.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <lfd/MotionLineMatcher.hpp>


using namespace std;
using namespace lsfm;

#define NUM_CAMS 2


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

    cv::Mat frameLR, imLeft, imRight;
    VideoCapture capture;
    capture = VideoCapture(filename1);
    capture.grab();
    capture.retrieve(frameLR, IMREAD_GRAYSCALE);
    cvtColor(frameLR,frameLR, CV_RGB2GRAY);

    double scaleFactor = 1.0;
    Size img_size;
    img_size.width = (frameLR.size().width / 2) * scaleFactor;
    img_size.height = frameLR.size().height * scaleFactor;

    cv::Mat P1, P2, map11, map12, map21, map22;

    if (!stereoRectification( filename2, filename3, img_size, P1, P2, map11, map12, map21, map22, scaleFactor))
    {
        cout << "Can not open files " << endl;
        return -1;
    }

    // find black borders
    cv::Mat borderDetect = cv::Mat::ones(frameLR.rows, frameLR.cols / 2, frameLR.type());
    borderDetect *= 255;
    cv::Mat bordersRecti;
    remap(borderDetect, bordersRecti, map11, map12, INTER_LINEAR);      // Undistort
    std::vector<int> border;
    findBlackCroppingArea<uchar>(bordersRecti, border);

    borderDetect = cv::Mat::ones(frameLR.rows, frameLR.cols / 2, frameLR.type());
    borderDetect *= 255;
    remap(borderDetect, bordersRecti, map21, map22, INTER_LINEAR);      // Undistort
    findBlackCroppingArea<uchar>(bordersRecti, border);

    int croppingHeight = std::max(border[0], std::max(border[1], std::max(border[4], border[5])));
    int croppingWidth = std::max(border[2], std::max(border[3], std::max(border[6], border[7])));


    //GaussianBlur(src1, src1, Size(3, 3), 0.6);

    // initialize Line Detector -------------------------------------------------------------------------
    typedef double FT;

    typedef LsdCC<FT>::LineSegment MyDetectorLine;
    typedef lsfm::Line3d MyLine3D;

    LsdCC<FT> lsd1(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.012, 0.008, 30, 0, 2);
    LsdCC<FT> lsd2(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.012, 0.008, 30, 0, 2);

    imLeft  = frameLR(cv::Range(0, frameLR.size().height), cv::Range(0, frameLR.size().width/2)); //no data copying here - Range excludes the last value of width, thus correct.
    imRight = frameLR(cv::Range(0, frameLR.size().height), cv::Range(frameLR.size().width/2, frameLR.size().width)); //no data copying here

    cv::Mat img1r, img2r;
    remap(imLeft, img1r, map11, map12, INTER_LINEAR);      // Undistort
    remap(imRight, img2r, map21, map22, INTER_LINEAR);

    imLeft = img1r(Rect(croppingWidth,croppingHeight,img1r.size().width-croppingWidth*2,img1r.size().height-croppingHeight*2)).clone();
    imRight = img2r(Rect(croppingWidth,croppingHeight,img2r.size().width-croppingWidth*2,img2r.size().height-croppingHeight*2)).clone();

    std::vector<FeatureMatch<FT>> matches;
    std::vector<std::vector<FeatureMatch<FT>>> matches2D;
//    MotionLineFilter<FT, LsdCC<FT>::LineSegmentVector> mlf(100);

    // prepare camera matrices
    cv::Mat_<double> current_Rotation(3,3);// = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    current_Rotation = cv::Mat::eye(3,3,CV_64F);
    cv::Mat_<double> current_Position(3,1);// = (cv::Mat_<double>(3,1) << 0, 0, 0);
    current_Position = cv::Mat::zeros(3,1,CV_64F);
    double currentReprojectionError = std::numeric_limits<double>::min();

    cv::Mat_<double> cameraMatrixLeft, cameraMatrixRight;
    {
        cv::Mat_<double> rotMatL, transVectL, rotMatR, transVectR;
        cv::decomposeProjectionMatrix(P1, cameraMatrixLeft, rotMatL, transVectL);
        cv::decomposeProjectionMatrix(P2, cameraMatrixRight, rotMatR, transVectR);
    }

    cv::Mat_<double> posLeft = cv::Mat::zeros(1,3,cameraMatrixLeft.type());// = current_Position - (R * current_Rotation * -T);  // * R
    cv::Mat_<double> posRight = cv::Mat::zeros(1,3,cameraMatrixLeft.type());// = current_Position - (R * current_Rotation * -T);  // * R
    cv::Mat projLeft = cv::Mat::zeros(3,4,cameraMatrixLeft.type()); //= getProjectionMatrix(cameraMatrixLeft, current_Rotation, current_Position);
    cv::Mat projRight = cv::Mat::zeros(3,4,cameraMatrixLeft.type());// = getProjectionMatrix(cameraMatrixRight, current_Rotation, posRight);

    cv::Mat camMat, stereoR;
    cv::Vec<double, 4> tVec;
    cv::decomposeProjectionMatrix(P2, camMat, stereoR, tVec);
    cv::Mat_<double> stereoT(3,1);
    stereoT(0) = -tVec(0) / tVec(3);
    stereoT(1) = -tVec(1) / tVec(3);
    stereoT(2) = -tVec(2) / tVec(3);

    Camera<FT> camOffset[2];
    Eigen::Matrix<double, 3, 3> cofK[2], tmpMat1, tmpMat2;

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            tmpMat1(i,j) = cameraMatrixLeft(i,j);
            tmpMat2(i,j) = cameraMatrixRight(i,j);
        }
    }
    lsfm::cofKcalculation<double, Eigen::Matrix<double, 3, 3>>(tmpMat1, cofK[0]);
    lsfm::cofKcalculation<double, Eigen::Matrix<double, 3, 3>>(tmpMat2, cofK[1]);
/*
    camOffset[1].translation[0] = stereoT(0);
    camOffset[1].translation[1] = stereoT(1);
    camOffset[1].translation[2] = stereoT(2);
    // no rotation here, as its stereo rectified

    camOffset[0].focal.x = cameraMatrixLeft(0,0);
    camOffset[0].focal.y = cameraMatrixLeft(1,1);
    camOffset[0].offset.x = cameraMatrixLeft(0,2);
    camOffset[0].offset.y = cameraMatrixLeft(1,2);
    camOffset[0].height = imLeft.rows;
    camOffset[0].width  = imLeft.cols;

    camOffset[1].focal.x = cameraMatrixRight(0,0);
    camOffset[1].focal.y = cameraMatrixRight(1,1);
    camOffset[1].offset.x = cameraMatrixRight(0,2);
    camOffset[1].offset.y = cameraMatrixRight(1,2);
    camOffset[1].height = imRight.rows;
    camOffset[1].width  = imRight.cols;
    */

    camOffset[0] = Camera<double>(cameraMatrixLeft(0,0), cameraMatrixLeft(1,1), cameraMatrixLeft(0,2), cameraMatrixLeft(1,2), imLeft.size().width, imLeft.size().height, 0, 0, 0, 0, 0, 0);
    camOffset[1] = Camera<double>(cameraMatrixRight(0,0), cameraMatrixRight(1,1), cameraMatrixRight(0,2), cameraMatrixRight(1,2), imRight.size().width, imRight.size().height, stereoT(0), stereoT(1), stereoT(2), 0, 0, 0);

    std::vector<lsfm::Camera<FT>> tmpCams;
    tmpCams.push_back(Camera<double>(cameraMatrixLeft(0,0), cameraMatrixLeft(1,1), cameraMatrixLeft(0,2), cameraMatrixLeft(1,2), imLeft.size().width, imLeft.size().height, 0, 0, 0, 0, 0, 0));
    tmpCams.push_back(Camera<double>(cameraMatrixRight(0,0), cameraMatrixRight(1,1), cameraMatrixRight(0,2), cameraMatrixRight(1,2), imRight.size().width, imRight.size().height, stereoT(0), stereoT(1), stereoT(2), 0, 0, 0));


    std::cout << stereoT(0) << "  " <<  stereoT(1) << "  " <<  stereoT(2) << "  " << std::endl;
    std::cout << camOffset[1].origin().x() << "  " <<  camOffset[1].origin().y() << "  " <<  camOffset[1].origin().z() << "  " << std::endl;

//    SlamModel<FT, NUM_CAMS, MyDetectorLine, MyLine3D> slam(RobotCameras<FT, NUM_CAMS>() , cofK);    // change lineDescriptor if needed
    //SlamModel<FT, NUM_CAMS, MyDetectorLine, MyLine3D> slam(camOffset, cofK);    // change lineDescriptor if needed
    SlamModel<FT, NUM_CAMS, MyDetectorLine, MyLine3D> slam(camOffset);    // change lineDescriptor if needed

    std::vector<DescriptorMatch<FT>> motionMatchesLeft, motionMatchesRight, motionMatchesRes;
    std::vector<size_t> maskN, maskP;

    LsdCC<FT>::LineSegmentVector previousLeftLines, previousRightLines;
    cv::Mat previousImGrayLeft = imLeft;

    int last_key_press = 0;
    int cycleNr = 0;
    while (last_key_press != 'q')
    {

        // stereo undistort and rectify  ---------------------------------------
        imLeft  = frameLR(cv::Range(0, frameLR.rows), cv::Range(0, frameLR.cols/2)); //no data copying here - Range excludes the last value of width, so correct.
        imRight = frameLR(cv::Range(0, frameLR.rows), cv::Range(frameLR.cols/2, frameLR.cols)); //no data copying here
/*
        resize(im_grayLeft, im_grayLeft, Size(im_grayLeft.size().width * scaleFactor, im_grayLeft.size().height * scaleFactor));
        resize(im_grayRight, im_grayRight, Size(im_grayRight.size().width * scaleFactor, im_grayRight.size().height * scaleFactor));
*/
        cv::Mat img1r, img2r;
        remap(imLeft, img1r, map11, map12, INTER_LINEAR);      // Undistort
        remap(imRight, img2r, map21, map22, INTER_LINEAR);

        imLeft = img1r(Rect(croppingWidth,croppingHeight,img1r.size().width-croppingWidth*2,img1r.size().height-croppingHeight*2)).clone();
        imRight = img2r(Rect(croppingWidth,croppingHeight,img2r.size().width-croppingWidth*2,img2r.size().height-croppingHeight*2)).clone();


        // line detection ---------------------------------------
        lsd1.detect(imLeft);
        lsd2.detect(imRight);
        std::cout << "lines left: " << lsd1.lines().size() << ", lines right: " << lsd2.lines().size() << std::endl;
        std::vector<DetectedLine> lsd1Models, lsd2Models;
        lsd1Models.assign(lsd1.lines().size(), DetectedLine());
        lsd2Models.assign(lsd2.lines().size(), DetectedLine());


        MatMap dataL, dataR;
        dataL["gx"] = lsd1.imageData()[0];
        dataL["gy"] = lsd1.imageData()[1];
        dataL["img"] = imLeft;

        dataR["gx"] = lsd2.imageData()[0];
        dataR["gy"] = lsd2.imageData()[1];
        dataR["img"] = imRight;

        // track lines (filter candidates, then match the remaining candidates) ---------------------------------------

        std::vector<DetectedLine> detectedLinesLeft, detectedLinesRight;
        int modeledLinesCount = slam.getModeledLines().size();

        if(cycleNr > 0){
            previousLeftLines = slam.getLineSegments(slam.getFrameNr(cycleNr - 1, 0));
            previousRightLines = slam.getLineSegments(slam.getFrameNr(cycleNr - 1, 1));

            detectedLinesLeft = slam.getDetectedLines(slam.getFrameNr(cycleNr - 1, 0));
            detectedLinesRight = slam.getDetectedLines(slam.getFrameNr(cycleNr - 1, 1));
        }


        typedef FdcMotion<FT,LsdCC<FT>::LineSegment> MyMotionFdc;
        MyMotionFdc::FdcPtr fdcM1 = MyMotionFdc::createFdc();
        MyMotionFdc::FdcPtr fdcM2 = MyMotionFdc::createFdc();

        double start = double(getTickCount());
        MotionLineMatcher<FT, LsdCC<FT>::LineSegmentVector,MyMotionFdc> mlm(fdcM1, fdcM2, imLeft.cols,  imLeft.rows);
        std::pair<FT, FT> movement(0,0);

        size_t num = matches.size();
        std::cout << "reduced candidates from " << lsd1.lines().size() * lsd2.lines().size() << " to " << num << std::endl;

        start = double(getTickCount());
        motionMatchesLeft.clear();
        motionMatchesRight.clear();
        motionMatchesRes.clear();
        mlm.match(lsd1.lines(), previousLeftLines, movement, motionMatchesLeft);

        mlm.match(lsd2.lines(), previousRightLines, movement, motionMatchesRight);

        //mlm.match(previousLeftLines, lsd1.lines(), movement, motionMatches);
/*
        PairwiseLineMatcher<FT, typename LbdFdc::descriptor_type> pmatcher;
        if(motionMatches.size() > 1)
            pmatcher.match1D(lsd1.lines(), lsd2.lines(), mlm.getDscNew(), mlm.getDscPrev(), motionMatches, motionMatchesRes);
*/

        double end = double(getTickCount());
        std::cout <<  "mlmLeft matches: " << motionMatchesLeft.size() << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;
        std::cout <<  "mlmRight matches: " << motionMatchesRight.size() << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;

        imshow("Detected tracking matches", drawMatches<FT,DescriptorMatch<FT>>(imLeft, lsd1.lines(), previousImGrayLeft, previousLeftLines, motionMatchesLeft));

        for_each(motionMatchesLeft.begin(), motionMatchesLeft.end(), [&](const DescriptorMatch<FT>& v){
            if(detectedLinesLeft.at(v.matchIdx).modelIndex > 0){
                lsd1Models.at(v.queryIdx) = detectedLinesLeft.at(v.matchIdx).modelIndex;
            }else{
                // triangulate new model from tracking only
            }
        });

        for_each(motionMatchesRight.begin(), motionMatchesRight.end(), [&](const DescriptorMatch<FT>& v){
            if(detectedLinesRight.at(v.matchIdx).modelIndex > 0){
                lsd2Models.at(v.queryIdx) = detectedLinesRight.at(v.matchIdx).modelIndex;
            }else{
                // triangulate new model from tracking only
            }
        });


        // estimate position ---------------------------------------
        // left Side ---
        double rot_cw_data[9], pos_cw_data[3];
        cv::Mat_<double> rot_cw_data_left(3,3), rot_cw_data_right(3,3), pos_cw_data_left(3,1), pos_cw_data_right(3,1);
        cv::Mat_<double> rightPos2Left(3,1), leftPos2Right(3,1);

        std::vector<LineSegment<double>> linesCam1, linesCam2;    // TODO
        std::vector<Line3<double>> modelLinesCam1, modelLinesCam2;   // TODO

        double minimalReprojectionErrorLeft = PnLpose(linesCam1, modelLinesCam1, lsfm::Matx33<double>(cameraMatrixLeft.ptr<double>()), rot_cw_data, pos_cw_data, false);
        for(int i = 0; i < 9; i++)
            rot_cw_data_left.at<double>(i) = rot_cw_data[i];
        pos_cw_data_left.at<double>(0) = pos_cw_data[0]; pos_cw_data_left.at<double>(1) = pos_cw_data[1]; pos_cw_data_left.at<double>(2) = pos_cw_data[2];


        // right Side ---
        double minimalReprojectionErrorRight = PnLpose(linesCam2, modelLinesCam2, lsfm::Matx33<double>(cameraMatrixRight.ptr<double>()), rot_cw_data, pos_cw_data, false);
        for(int i = 0; i < 9; i++)
            rot_cw_data_right.at<double>(i) = rot_cw_data[i];
        pos_cw_data_right.at<double>(0) = pos_cw_data[0]; pos_cw_data_right.at<double>(1) = pos_cw_data[1]; pos_cw_data_right.at<double>(2) = pos_cw_data[2];

        // R or inverse of R ? ... R close to identity currently
        rightPos2Left = pos_cw_data_right + (stereoR * rot_cw_data_right * stereoT); //+ (R * rot_cw_data_right) * T;
        leftPos2Right = pos_cw_data_left - (stereoR * rot_cw_data_left * stereoT); //- (R * rot_cw_data_left * T);


        // choose new Pose, from left and right PnL estimations - take best

        if(minimalReprojectionErrorLeft < minimalReprojectionErrorRight){
            if(minimalReprojectionErrorLeft < 1)
            {
                pos_cw_data_left.copyTo(current_Position);
                rot_cw_data_left.copyTo(current_Rotation);
                currentReprojectionError = minimalReprojectionErrorLeft;
            }
        } else {
            if(minimalReprojectionErrorLeft < 1)
            {
                rightPos2Left.copyTo(current_Position);
                rot_cw_data_right.copyTo(current_Rotation);     // multiply with R if R not identity
                currentReprojectionError = minimalReprojectionErrorRight;
            }
        }



        // Stereo matching ---------------------------------------------------------
        typedef GchGradImgInterpolate<FT> MyGchHelper;
        typedef FdcGenericLR<FT,LsdCC<FT>::LineSegment,MyGchHelper> MyFdc;

        MyFdc::FdcPtr fdcL = MyFdc::createFdc(dataL);
        MyFdc::FdcPtr fdcR = MyFdc::createFdc(dataR);
        StereoLineMatcher<FT, LsdCC<FT>::LineSegmentVector,MyGchHelper> slm(fdcL, fdcR, imLeft.rows, imLeft.cols / 2,5,0.6);

        std::vector<DescriptorMatch<FT>> bfmatches3;
        slm.match(lsd1.lines(),lsd2.lines(),bfmatches3);

        std::cout <<  "final matches: " << bfmatches3.size() << std::endl;
        imshow("Detected Stereo matches", drawMatches<FT,DescriptorMatch<FT>>(imLeft, lsd1.lines(), imRight, lsd2.lines(), bfmatches3));



        // triangulate new stereo lines ---------------------------------------

        posRight = current_Position.clone(); // * R
        posRight = posRight + (stereoR * current_Rotation * stereoT);
        projRight = (getProjectionMatrix(cameraMatrixRight, current_Rotation, posRight)).clone();
        posLeft = posRight - (stereoR * current_Rotation * stereoT);
        projLeft = (getProjectionMatrix(cameraMatrixLeft, current_Rotation, posLeft)).clone();

        //lsfm::LineSegmentDetector<double>::Line leftLine, rightLine; // TODO

        cv::Point3d startPoint3d, endPoint3d;
        //triangulateLine(&projLeft, &projRight, leftLine, rightLine, startPoint3d, endPoint3d);
       std::vector<ModelLine<MyLine3D>> lineModels;

        for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<FT>& v){
            triangulateLine(&projLeft, &projRight, lsd1.lines().at(v.queryIdx), lsd2.lines().at(v.matchIdx), startPoint3d, endPoint3d);
            Observation ObsL(cycleNr * NUM_CAMS + 0, lsfm::OBS_FLAGS_STEREO, slam.getModeledLines().size() + v.queryIdx);
            Observation ObsR(cycleNr * NUM_CAMS + 1, lsfm::OBS_FLAGS_STEREO, slam.getModeledLines().size() + lsd1.lines().size() + v.matchIdx);
            std::vector<Observation> obsList;
            obsList.push_back(ObsL);
            obsList.push_back(ObsR);

            //ModelLine<MyLine3D> modeledLine(LineSegment3<FT> (startPoint3d, endPoint3d), obsList);
            ModelLine<MyLine3D> modeledLine(LineSegment3<FT> (lsfm::Vec3<FT>(startPoint3d.x, startPoint3d.y, startPoint3d.z), lsfm::Vec3<FT>(endPoint3d.x, endPoint3d.y, endPoint3d.z)), MODEL_FLAGS_NONE, obsList);
            lineModels.push_back(modeledLine);
            lsd1Models.at(v.queryIdx) = modeledLinesCount;
            lsd2Models.at(v.matchIdx) = modeledLinesCount;
            ++modeledLinesCount;
        });



        // add to slam data model ---------------------------------------
        cv::Mat_<FT> rodriguesRot(3,1);
        cv::Rodrigues(current_Rotation, rodriguesRot);
        Pose<FT> currentPose;
        /*
        currentPose.translation[0] = current_Position(0,0);
        currentPose.translation[1] = current_Position(1,0);
        currentPose.translation[2] = current_Position(2,0);
        currentPose.rotation[0] = rodriguesRot(0,0);
        currentPose.rotation[1] = rodriguesRot(1,0);
        currentPose.rotation[5] = rodriguesRot(2,0);
        */

        currentPose.origin(lsfm::Vec3<FT>(current_Position(0,0), current_Position(1,0), current_Position(2,0)));
        currentPose.orientation(lsfm::Vec3<FT>(rodriguesRot(0,0),rodriguesRot(1,0),rodriguesRot(2,0)));

        std::cout << "rodriguez: " << rodrigues(Vec3<double>(0.5, 1.3, 2.6)) << std::endl;

        double testR[3] = {0.5, 1.3, 2.6}, tmpMat[9];
        ceres::AngleAxisToRotationMatrix(&testR[0], tmpMat);
        std::cout << "AngleAxis: " << std::endl << tmpMat[0] << " " << tmpMat[3] << " " << tmpMat[6] << " " << std::endl;
        std::cout << tmpMat[1] << " " << tmpMat[4] << " " << tmpMat[7] << " " << std::endl;
        std::cout << tmpMat[2] << " " << tmpMat[5] << " " << tmpMat[8] << " " << std::endl;



        // add leftImage & Pose
        Pose<FT> rp(currentPose);
        slam.addData(lsd1.lines(), lsd1Models, lineModels, rp);
        for(int i = 0; i < 6; i++){
            std::cout << "robot pose: " << rp.origin() << std::endl;
        }

        //ModelLine
        std::vector<ModelLine<MyLine3D>> emptyModels;
        // add rightImage
        slam.addData(lsd2.lines(), lsd2Models, emptyModels);


        cv::Mat linesPrevLeft = previousImGrayLeft;
        cv::Mat linesLeft = imLeft.clone();
        cv::Mat linesRight = imRight.clone();
        drawLines<FT>(linesLeft, lsd1.lines());
        drawLines<FT>(linesRight, lsd2.lines());
        drawLines<FT>(linesPrevLeft, previousLeftLines);
        imshow("leftLines", linesLeft);
        imshow("rightLines", linesRight);
        imshow("prevLeftLines", linesPrevLeft);

        previousImGrayLeft = imLeft;
        previousLeftLines = lsd1.lines();
        previousRightLines = lsd2.lines();


        // run bundle adjustment --------------------------------
        if(!(cycleNr % 3) && cycleNr > 2){
            //std::vector<int> fms={0,1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
            std::vector<int> fms={1};
            slam.bundleAdjustmentOnFrames(fms);

            slam.getModelLineSegments();
        }


        // get new frame ---------------------------------------
        if(!capture.grab())
            break;
        capture.retrieve(frameLR, IMREAD_GRAYSCALE);
        cvtColor(frameLR,frameLR,CV_RGB2GRAY);

        cv::waitKey(1000000);
        last_key_press = cvWaitKey(1)%256;
        ++cycleNr;
    }



    return 0;
}
