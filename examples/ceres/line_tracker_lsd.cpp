#include <iostream>
#include <fstream>
#include <string>

#include <lsd/lsd_cc.hpp>
#include <opencv2/core/core.hpp>
#include <line_tracker/line_analyzer.hpp>
#include <line_tracker/stereo_line_analyzer.hpp>
#include <slam/pose_estimator.hpp>
#include <geometry/line3.hpp>
#include <geometry/camera.hpp>
#include <geometry/cameracv.hpp>
#include <numeric>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv2/calib3d/calib3d.hpp"

#include <calc_PnL/PnL.h>
#include <calc_PnL/calc_PnL.h>
#include <calc_PnL/calc_PnL_types.h>
#include <calc_PnL/calc_PnL_emxAPI.h>

//#include <line_tracker/ceres_line_problem.hpp>

//#include <slam/slamDataModel.hpp>

#define BINSIZE 12
//#define MEASURE_TIME

#define CROP_WIDTH 34
#define CROP_HEIGHT 57

// Global for easier dlib usage
//StereoLineAnalyzer* sla;

using namespace std;
using namespace lsfm;
// using namespace lsdwrap;

typedef double FT;

int main(int argc, char** argv)
{

    cv::Mat frame, im_grayLeft, im_grayRight,im_gray;
//    cv::Mat image = imread(in, IMREAD_GRAYSCALE);
    //imshow("Input image", image);

    VideoCapture capture;
    const char* intrinsic_filename = 0;
    const char* extrinsic_filename = 0;

    if (argc == 2){
//            std::cout << "visual_test [in]" << std::endl
//                << "\tin - input image" << std::endl;
//            return false;
        std::string in = argv[1];
        capture = VideoCapture(in);
    } else if(argc > 2){
        std::string in = argv[1];
        capture = VideoCapture(in);
        for( int i = 2; i < argc; i++ )
        {
            if( strcmp(argv[i], "-i" ) == 0 )
                intrinsic_filename = argv[++i];
            else if( strcmp(argv[i], "-e" ) == 0 )
                extrinsic_filename = argv[++i];

            else
            {
                printf("Command-line parameter error: unknown option %s\n", argv[i]);
                return -1;
            }
        }

    } else {
       capture = VideoCapture(0);
//       capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
//       capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    }

    capture.grab();
    capture.retrieve(frame, IMREAD_GRAYSCALE);

    // Undistort
    std::cout << "Original width: " << frame.size().width / 2 << " height: " << frame.size().height << std::endl;
    double scaleFactor = 1.0;
    Size img_size;
    img_size.width = (frame.size().width / 2) * scaleFactor;
    img_size.height = frame.size().height * scaleFactor;
    std::cout << "Scaled width: " << img_size.width << " height: " << img_size.height << std::endl;
    Rect roi1, roi2;
    cv::Mat Q;
    cv::Mat map11, map12, map21, map22;
    cv::Mat R, T, R1, P1, R2, P2;
    if( intrinsic_filename )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename);
            return -1;
        }

        cv::Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scaleFactor;
        M2 *= scaleFactor;

        fs.open(extrinsic_filename, READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename);
            return -1;
        }

        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
//        std::cout << "P1 " << P1 << std::endl << "  P2  " << P2 << std::endl;

        // cv::Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

//        cv::Mat img1r, img2r; remap(img1, img1r, map11, map12, INTER_LINEAR); remap(img2, img2r, map21, map22, INTER_LINEAR); img1 = img1r; img2 = img2r;
    }
    std::cout << "imgX " << img_size.width << " imgY " << img_size.height << std::endl;

    Ptr<LsdBase<double>> lsd_cvLeft = new LsdCC<double>(0, 0.012, 0.004, 20, 0, 1.5);
    Ptr<LsdBase<double>> lsd_cvRight = new LsdCC<double>(0, 0.012, 0.004, 20, 0, 1.5);

    std::vector<lsfm::Vec4i> detectedLinesLeft, detectedLinesRight;
    std::vector<lsfm::Vec4d> detectedLinesLeftd, detectedLinesRightd;
    LineAnalyzer laLeft(BINSIZE);
    LineAnalyzer laRight(BINSIZE);
    StereoLineAnalyzer sla(laLeft, laRight, BINSIZE * 2, &P1, &P2);

    double start, duration_ms;

    // Adapt Projection Matrix to Cropping
//    std::cout << "P1: " << P1 << std::endl << "P2 " << P2 << std::endl;
    P1.at<double>(0,2) = P1.at<double>(0,2) - (double) CROP_WIDTH;
    P1.at<double>(1,2) = P1.at<double>(1,2) - (double) CROP_HEIGHT;
    P2.at<double>(0,2) = P2.at<double>(0,2) - (double) CROP_WIDTH;
    P2.at<double>(1,2) = P2.at<double>(1,2) - (double) CROP_HEIGHT;
    std::cout << "New offset: P1: " << P1 << std::endl << "New offset: P2 " << P2 << std::endl;

    int last_key_press = 0;
    int cycleNum = 1;
    double totalDuration = 0;
    start = double(getTickCount());
    double overallAverageStableCorrespondences = 0.0;
    bool drawLines = true;

    std::vector<LineSegment3<double>> lines3;
    lines3.push_back(LineSegment3<double> ());  // add "empty" line, to avoid any line3d having id 0, as 0 means

    // Extract camera matrices
    cv::Mat_<double> cameraMatrixLeft, cameraMatrixRight;
    {
        cv::Mat_<double> rotMatL, transVectL, rotMatR, transVectR;
        cv::decomposeProjectionMatrix(P1, cameraMatrixLeft, rotMatL, transVectL);
        cv::decomposeProjectionMatrix(P2, cameraMatrixRight, rotMatR, transVectR);


        // homogenize
//        transVectR = homogenizeVector(transVectR);
//        std::cout << "transVectR " << transVectR  << std::endl << std::endl;
//        transVectR = -transVectR;
//        std::cout << "transVectR " << transVectR - T  << std::endl << std::endl;
//        std::cout << "T " << T  << std::endl << std::endl;

//        std::cout << "transVectR " << transVectR - T  << std::endl << std::endl;
//        std::cout << "T " << T  << std::endl << std::endl;
//        std::cout << "getProjectionMatrix " << std::endl << getProjectionMatrix(cameraMatrixRight, rotMatR, transVectR) << std::endl << std::endl;
//        std::cout << "getProjectionMatrix " << std::endl << getProjectionMatrix(cameraMatrixLeft, rotMatL, transVectL) << std::endl << std::endl;

    }

    lsfm::Matx33<double> current_Rotation;// = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    current_Rotation.setIdentity();
    lsfm::Vec3<double> current_Position;// = (cv::Mat_<double>(3,1) << 0, 0, 0);
    current_Position.setZero();
    double currentReprojectionError = std::numeric_limits<double>::min();

    //cv::Mat_<double> posLeft = cv::Mat::zeros(1,3,cameraMatrixLeft.type());// = current_Position - (R * current_Rotation * -T);  // * R
    lsfm::Vec3<FT> posLeft;
    posLeft.setZero();
//    cv::Mat_<double> posRight = cv::Mat::zeros(1,3,cameraMatrixLeft.type());// = current_Position - (R * current_Rotation * -T);  // * R
//    cv::Mat projLeft = cv::Mat::zeros(3,4,cameraMatrixLeft.type()); //= getProjectionMatrix(cameraMatrixLeft, current_Rotation, current_Position);
//    cv::Mat projRight = cv::Mat::zeros(3,4,cameraMatrixLeft.type());// = getProjectionMatrix(cameraMatrixRight, current_Rotation, posRight);
    lsfm::Matx34<double> projRight, projLeft;
    projRight.setZero();
    projLeft.setZero();

    lsfm::Matx33<double> camMat, stereoR;
    lsfm::Vec3<double> stereoT;
    //cv::decomposeProjectionMatrix(P2, camMat, stereoR, tVec);
    lsfm::Matx34<double>P2eigen(P2.ptr<double>());
    lsfm::Camera<double>::decomposeProjectionMatrix(P2eigen, camMat, stereoT, stereoR);
    stereoT = -stereoT;

//    projLeft = (getProjectionMatrix(cameraMatrixLeft, current_Rotation, current_Position)).clone();
    lsfm::Vec3<double> posRight(current_Position); // * R
    posRight = posRight + (stereoR * current_Rotation * stereoT);
    projRight = (lsfm::Camera<FT>::composeProjectionMatrix(lsfm::Matx34<FT>(cameraMatrixRight.ptr<double>()), current_Rotation, posRight));
    posLeft = posRight - (stereoR * current_Rotation * stereoT);
//    projLeft = (getProjectionMatrix(cameraMatrixLeft, current_Rotation, posLeft)).clone();
    projLeft = (lsfm::Camera<FT>::composeProjectionMatrix(lsfm::Matx34<FT>(cameraMatrixLeft.ptr<double>()), current_Rotation, posLeft));
    std::cout << "projLeft: " << projLeft << std::endl << " P1 " << P1 << std::endl << " projRight " << projRight << std::endl << " P2 " << P2 << std::endl;
    std::cout << "cameraMatrixLeft: " << cameraMatrixLeft << std::endl << " current_Rotation " << current_Rotation << std::endl << " posLeft " << posLeft << std::endl;


    while (last_key_press != 'q')
    {
        start = double(getTickCount());

        laLeft.setCurrentFrameNr(cycleNum);
        laRight.setCurrentFrameNr(cycleNum);

        double captureTime1 = double(getTickCount());

        if(!capture.grab())
            break;
        capture.retrieve(frame, IMREAD_GRAYSCALE);

        //#ifdef MEASURE_TIME
        double captureTime2 = (double(getTickCount()) - captureTime1) * 1000 / getTickFrequency();
        //#endif

        double convertTime1 = double(getTickCount());
        cvtColor(frame,im_gray,CV_RGB2GRAY);
        //#ifdef MEASURE_TIME
        double convertTime2 = (double(getTickCount()) - convertTime1) * 1000 / getTickFrequency();
        //#endif

        im_grayLeft  = im_gray(cv::Range(0, im_gray.size().height), cv::Range(0, im_gray.size().width/2)); //no data copying here - Range excludes the last value of width, so correct.
        im_grayRight = im_gray(cv::Range(0, im_gray.size().height), cv::Range(im_gray.size().width/2, im_gray.size().width)); //no data copying here

        double resizeTime1 = double(getTickCount());
        resize(im_grayLeft, im_grayLeft, Size(im_grayLeft.size().width * scaleFactor, im_grayLeft.size().height * scaleFactor));
        resize(im_grayRight, im_grayRight, Size(im_grayRight.size().width * scaleFactor, im_grayRight.size().height * scaleFactor));
        //#ifdef MEASURE_TIME
        double resizeTime2 = (double(getTickCount()) - resizeTime1) * 1000 / getTickFrequency();
        //#endif

        double undistortTime1 = double(getTickCount());
        if(intrinsic_filename){
            cv::Mat img1r, img2r;
            remap(im_grayLeft, img1r, map11, map12, INTER_LINEAR);      // Undistort
            remap(im_grayRight, img2r, map21, map22, INTER_LINEAR);

            im_grayLeft = img1r(Rect(CROP_WIDTH,CROP_HEIGHT,img1r.size().width-CROP_WIDTH*2,img1r.size().height-CROP_HEIGHT*2)).clone();
            im_grayRight = img2r(Rect(CROP_WIDTH,CROP_HEIGHT,img2r.size().width-CROP_WIDTH*2,img2r.size().height-CROP_HEIGHT*2)).clone();
            // Adjust Projection Matrix:

        }
        //#ifdef MEASURE_TIME
        double undistortTime2 = (double(getTickCount()) - undistortTime1) * 1000 / getTickFrequency();
        //#endif

        // Detect Lines: Extract Lines and keep modgrad image --------------------------------------------------------------------------------------
        // 0: gx, 1: gy, 2: modgrad, 3: angles
        vector<cv::Mat> imageDataLeft, imageDataRight;
        double detectTime1 = double(getTickCount());
        lsd_cvLeft->detect(im_grayLeft, detectedLinesLeftd, imageDataLeft);
        LineSegment2Vector<double> leftLinesVec = lsd_cvLeft->lines();
        lsd_cvRight->detect(im_grayRight, detectedLinesRightd, imageDataRight);
        LineSegment2Vector<double> rightLinesVec = lsd_cvRight->lines();

        imageDataLeft[2].convertTo(imageDataLeft[2], CV_32FC1);
        imageDataRight[2].convertTo(imageDataRight[2], CV_32FC1);

        // wurzel aus gradienten bild (bei CC_LDE_REGRESSION), könnte noch wegoptimiert werden
        sqrt(imageDataLeft[2]/4, imageDataLeft[2]);
        sqrt(imageDataRight[2]/4, imageDataRight[2]);

        //convert
        detectedLinesLeft.clear();
        for(int i = 0; i < detectedLinesLeftd.size(); i++){
            lsfm::Vec4i vec;
            vec[0] = (int)detectedLinesLeftd[i][0];
            vec[1] = (int)detectedLinesLeftd[i][1];
            vec[2] = (int)detectedLinesLeftd[i][2];
            vec[3] = (int)detectedLinesLeftd[i][3];
            detectedLinesLeft.push_back(vec);
        }
        detectedLinesRight.clear();
        for(int i = 0; i < detectedLinesRightd.size(); i++){
            lsfm::Vec4i vec;
            vec[0] = (int)detectedLinesRightd[i][0];
            vec[1] = (int)detectedLinesRightd[i][1];
            vec[2] = (int)detectedLinesRightd[i][2];
            vec[3] = (int)detectedLinesRightd[i][3];
            detectedLinesRight.push_back(vec);
        }

        //#ifdef MEASURE_TIME
        double detectTime2 = (double(getTickCount()) - detectTime1) * 1000 / getTickFrequency();
        //#endif

        cv::Mat lineImage(cv::Mat::zeros(im_grayLeft.size(), CV_8UC3 ));
        cv::Mat lineImageMixedColors(cv::Mat::zeros(im_grayLeft.size(), CV_8UC3 ));
        cv::Mat lineImageCorrespondences(cv::Mat::zeros(im_grayLeft.size(), CV_8UC3 ));

        laLeft.drawSegmentsMixedColors(lineImage, detectedLinesLeft);

        double analyzeTime1 = double(getTickCount());

        // analyze... get more data out of the lines
        laLeft.analyzeLines(leftLinesVec, im_grayLeft, imageDataLeft[2], (int)sqrt(im_gray.size().height * im_gray.size().height + im_gray.size().width * im_gray.size().width));
        laRight.analyzeLines(rightLinesVec, im_grayRight, imageDataRight[2], (int)sqrt(im_gray.size().height * im_gray.size().height + im_gray.size().width * im_gray.size().width));

        //#ifdef MEASURE_TIME
        double analyzeTime2 = (double(getTickCount()) - analyzeTime1) * 1000 / getTickFrequency();
        //#endif

        double drawingTime1 = double(getTickCount());

        cv::Mat im_grayLeftPainted, im_grayRightPainted;
        im_grayLeft.copyTo(im_grayLeftPainted);
        im_grayRight.copyTo(im_grayRightPainted);

        laLeft.drawAnalyzedLinesMixedColors(lineImageMixedColors, true);
        if(drawLines){
            laLeft.drawAnalyzedLinesMixedColors(im_grayLeftPainted, true);
            laRight.drawAnalyzedLinesMixedColors(im_grayRightPainted, true);
        }
        //#ifdef MEASURE_TIME
        double drawingTime2 = (double(getTickCount()) - drawingTime1) * 1000 / getTickFrequency();
        //#endif

        // Tracking -----------------------------------------------------------------------------------------------------------------------------------

        double trackingTime1 = double(getTickCount());
        laLeft.findCorrespondingLines();
        laRight.findCorrespondingLines();

        //#ifdef MEASURE_TIME
        double trackingTime2 = (double(getTickCount()) - trackingTime1) * 1000 / getTickFrequency();
        //#endif

        //std::cout << "corr Anzahl: " << la->getNextCorrespondenceNr() << std::endl;
        drawingTime1 = double(getTickCount());
        cv::Mat unDistImGray(cv::Mat::zeros(Size(im_grayLeftPainted.size().width*2,im_grayLeftPainted.size().height), CV_8UC3 ));

        //#ifdef MEASURE_TIME
        drawingTime2 += (double(getTickCount()) - drawingTime1) * 1000 / getTickFrequency();
        //#endif

        //  pose estimation from calculated 2d 3d correspondences --------------------------------------------------------------------------------------------------
        std::vector<AnalyzedLine*> *leftAL, *rightAL;
        //std::vector<AnalyzedLine*> *leftAL;
        leftAL = &laLeft.getAnalyzedLinesRef();
        //std::vector<AnalyzedLine*> *rightAL;
        rightAL = &laRight.getAnalyzedLinesRef();

        double rot_cw_data[9], pos_cw_data[3], minimalReprojectionErrorLeft, minimalReprojectionErrorRight;
        cv::Mat_<double> rot_cw_data_left(3,3), rot_cw_data_right(3,3), pos_cw_data_left(3,1), pos_cw_data_right(3,1);
//        cv::Mat_<double> rightPos2Left(3,1), leftPos2Right(3,1);
        lsfm::Vec3<FT> rightPos2Left, leftPos2Right;

        // Fill 2d-3d Correspondences for pose estimation
        std::vector<Line3<double>> currentLines3d;
        std::vector<LineSegment<double>> currentLines2d;

        // get left lines and corresponding 3d Lines
        for(int i = 0; i < leftAL->size(); i++){
            AnalyzedLine * currentLine = leftAL->at(i);
            if(currentLine->line3Nr){
                currentLines3d.push_back(lines3.at(currentLine->line3Nr));
                currentLines2d.push_back(currentLine->line);
            }
        }
/*
        auto maxLine = std::max_element( currentLines2d.begin(), currentLines2d.end(), []( const LineSegment<double> &a, const LineSegment<double> &b )
                                     { return a.length() < b.length(); } );
        int indexPosition = ( std::distance( currentLines2d.begin(), maxLine ) );
        std::cout << "idxPos: " << indexPosition << std::endl;
*/

        // calculate Pose of left Camera
        double PnLposeTime1l = double(getTickCount());
        minimalReprojectionErrorLeft = PnLpose(currentLines2d, currentLines3d, P1, rot_cw_data, pos_cw_data, false);
        double PnLposeTime2l = (double(getTickCount()) - PnLposeTime1l) * 1000 / getTickFrequency();
        for(int i = 0; i < 9; i++)
            rot_cw_data_left.at<double>(i) = rot_cw_data[i];
        pos_cw_data_left.at<double>(0) = pos_cw_data[0]; pos_cw_data_left.at<double>(1) = pos_cw_data[1]; pos_cw_data_left.at<double>(2) = pos_cw_data[2];


        // get right lines and corresponding 3d Lines
        currentLines3d.clear();
        currentLines2d.clear();
        for(int i = 0; i < rightAL->size(); i++){
            AnalyzedLine * currentLine = rightAL->at(i);
            if(currentLine->line3Nr){
                currentLines3d.push_back(lines3.at(currentLine->line3Nr));
                currentLines2d.push_back(currentLine->line);
            }
        }

        // calculate Pose of right Camera
        double PnLposeTime1r = double(getTickCount());
        minimalReprojectionErrorRight = PnLpose(currentLines2d, currentLines3d, P2, rot_cw_data, pos_cw_data, false);
        double PnLposeTime2r = (double(getTickCount()) - PnLposeTime1r) * 1000 / getTickFrequency();
        for(int i = 0; i < 9; i++)
            rot_cw_data_right.at<double>(i) = rot_cw_data[i];
        pos_cw_data_right.at<double>(0) = pos_cw_data[0]; pos_cw_data_right.at<double>(1) = pos_cw_data[1]; pos_cw_data_right.at<double>(2) = pos_cw_data[2];


        // R or inverse of R ? ... R close to identity currently
        rightPos2Left = pos_cw_data_right + (stereoR * rot_cw_data_right * stereoT); //+ (R * rot_cw_data_right) * T;
        leftPos2Right = pos_cw_data_left - (stereoR * rot_cw_data_left * stereoT); //- (R * rot_cw_data_left * T);

//        std::cout << "PnLposeTime1: " << PnLposeTime2l << std::endl;
        std::cout << "posL " << std::endl << pos_cw_data_left << std::endl;
//        std::cout << "rotDataL: " << rot_cw_data_left << std::endl;
        std::cout << "reprojErrorLeft " << minimalReprojectionErrorLeft << std::endl;


//        std::cout << "PnLposeTime2: " << PnLposeTime2r << std::endl;
        std::cout << "posR " << std::endl << pos_cw_data_right << std::endl;
//        std::cout << "rotDataR: " << rot_cw_data_right << std::endl;
        std::cout << "reprojErrorRight " << minimalReprojectionErrorRight << std::endl;

        double reprojE = reprojectionErrorPnL(currentLines2d, currentLines3d, P1, rot_cw_data, pos_cw_data);
//        std::cout << "recalcTestErrorRight " << reprojE << std::endl;


// choose new Pose, from left and right PnL estimations - merge later to one big view?

        if(minimalReprojectionErrorLeft < minimalReprojectionErrorRight){
            if(minimalReprojectionErrorLeft < 1)
            {
    //            current_Position = pos_cw_data_left.clone();
    //            current_Rotation = rot_cw_data_left.clone();
                pos_cw_data_left.copyTo(current_Position);
                rot_cw_data_left.copyTo(current_Rotation);
                currentReprojectionError = minimalReprojectionErrorLeft;
            }
        } else {
            if(minimalReprojectionErrorLeft < 1)
            {
    //            current_Position = rightPos2Left.clone();
    //            current_Rotation = rot_cw_data_right.clone(); // multiply with R if R not identity
                rightPos2Left.copyTo(current_Position);
                rot_cw_data_right.copyTo(current_Rotation);
                currentReprojectionError = minimalReprojectionErrorRight;

            }
        }

        // calculate error from 3d projection to recent 2d perception of left lines
/*        for(int i = 0; i < leftAL->size(); i++){
            AnalyzedLine * currentLine = leftAL->at(i);
            if(currentLine->line3Nr){
                Line3Segment<double> * line3 = &lines3.at(currentLine->line3Nr);
                line3->project2Line(
            }
        }
*/

        std::cout << "current_Position: " << current_Position << std::endl ;
        std::cout << "current_Rotation: " << current_Rotation << std::endl ;
        posRight = current_Position.clone(); // * R
        posRight = posRight + (stereoR * current_Rotation * stereoT);
        projRight = (getProjectionMatrix(cameraMatrixRight, current_Rotation, posRight)).clone();
        posLeft = posRight - (stereoR * current_Rotation * stereoT);
        projLeft = (getProjectionMatrix(cameraMatrixLeft, current_Rotation, posLeft)).clone();
        std::cout << "projLeft: " << projLeft << std::endl << " P1 " << P1 << std::endl << " projRight " << projRight << std::endl << " P2 " << P2 << std::endl;

/*
        std::cout << "new_Position " << current_Position << std::endl;
        std::cout << "new_Rotation " << current_Rotation << std::endl;
*/
        // alternative incremental position optimization method
        R_and_Tpose(currentLines2d, currentLines3d, P1, current_Rotation, current_Position, rot_cw_data, pos_cw_data);
//        std::cout << "rot_cw_data " << std::endl << rot_cw_data << std::endl;
//        std::cout << "pos_cw_data " << "  " << pos_cw_data[0] << "  " << pos_cw_data[1] << "  " << pos_cw_data[2] << std::endl;


        // triangulate -------------------------------------------------------------------------------------------------------------------
        double stereoTime1 = double(getTickCount());

        sla.matchLines(im_grayLeft.size(), *leftAL, *rightAL, false);
//        sla.correctMatchedLines(imageDataLeft[2], imageDataRight[2], unDistImGray, im_grayLeft, im_grayRight);
        //#ifdef MEASURE_TIME
        double stereoTime2 = (double(getTickCount()) - stereoTime1) * 1000 / getTickFrequency();
        //#endif

        std::vector<StereoCorrelation> currentCorrelations;
        currentCorrelations = sla.getCurrentStereoCorrelations();

        // calculate new 3d-lines
        if(currentReprojectionError < 0.016 || true){

            for(int i = 0; i < currentCorrelations.size(); i++){
                StereoCorrelation * currentCorrelation = &currentCorrelations.at(i);

                // Check if Line is Horizontal
                if((currentCorrelation->leftLine->line.angle() > 3.0 || currentCorrelation->rightLine->line.angle() > 3.0 || currentCorrelation->leftLine->line.angle() < -3.0 || currentCorrelation->rightLine->line.angle() < -3.0 ||
                        (currentCorrelation->leftLine->line.angle() < 0.2 && currentCorrelation->leftLine->line.angle() > -0.2) || (currentCorrelation->rightLine->line.angle() < 0.2 && currentCorrelation->rightLine->line.angle() > -0.2)) ){

                    currentCorrelation->horizontal = true;
                    continue;
                }

                // Triangulation
                cv::Point3d startPoint3d, endPoint3d;
                double commonStartY, commonEndY;

                currentCorrelation->horizontal = false;
                triangulateLine(&projLeft, &projRight, currentCorrelation->leftLine->line, currentCorrelation->rightLine->line, startPoint3d, endPoint3d, commonStartY, commonEndY);


                //using planes
               // triangulateLine(projLeft, projRight, currentCorrelation->leftLine->line, currentCorrelation->rightLine->line);

                currentCorrelation->line3d = LineSegment3<double> (lsfm::Vec3d(startPoint3d.x, startPoint3d.y, startPoint3d.z), lsfm::Vec3d(endPoint3d.x, endPoint3d.y, endPoint3d.z) );
                lines3.push_back(currentCorrelation->line3d);
                currentCorrelation->leftLine->line3Nr = lines3.size() - 1;
                currentCorrelation->rightLine->line3Nr = lines3.size() - 1;

                LineSegment2d lTest = CameraPlueckerd::projectLineSegment(lsfm::Matx34<double>(projLeft.ptr<double>()),currentCorrelation->line3d); //TODO: check Type
                LineSegment2d rTest = CameraPlueckerd::projectLineSegment(lsfm::Matx34<double>(projRight.ptr<double>()),currentCorrelation->line3d);

                //Point3_<double> m2, l2;
               // Point3_<double> m(currentCorrelation->line3d.momentum()[0], currentCorrelation->line3d.momentum()[1], currentCorrelation->line3d.momentum()[2]);
                //Point3_<double> l(currentCorrelation->line3d.direction()[0], currentCorrelation->line3d.direction()[1], currentCorrelation->line3d.direction()[2]);
                double w;

                lsfm::Vec3d m2, l2, s, m = (currentCorrelation->line3d.momentum()), l = (currentCorrelation->line3d.direction());
                currentCorrelation->line3d.cayleyRepresentationFromPluecker(m, l, w, s);
                currentCorrelation->line3d.plueckerCoordinatesFromCayley(w, s, m2, l2);

                //cv::Mat K = projLeft(lsfm::Rect(0,0,3,3)).clone();
                cv::Point3d testT(current_Position);
                std::cout << "-----------------------------------------------------------------------------------------------------------"  << std::endl;
                std::cout.precision(500);
                std::cout << " cameraMatrixLeft " << cameraMatrixLeft(0,0) << std::endl;
                std::cout << " cameraMatrixLeft " << cameraMatrixLeft(0,2) << std::endl;
                std::cout << " cameraMatrixLeft " << cameraMatrixLeft(1,1) << std::endl;
                std::cout << " cameraMatrixLeft " << cameraMatrixLeft(1,2) << std::endl;
                std::cout << " cameraMatrixLeft " << cameraMatrixLeft(2,2) << std::endl;
                std::cout << " cameraMatrixLeft " << cameraMatrixLeft << std::endl;
                std::cout << " m " << m << std::endl;
                std::cout << " l " << l << std::endl;
                //static inline Line<FT> projectPlueckerM(const Matx33<FT> &cam, const Vec3<FT> &trans, const Matx33<FT> &rot, const Vec3<FT> &m, const Vec3<FT> &l) {
                Line<double> testLine = CameraPlueckerd::projectPlueckerM(cameraMatrixLeft, testT, current_Rotation, m2, l2);

                std::cout << "current_Rotation: " << current_Rotation << " testT: " << testT << std::endl;
                std::cout << "line3d: " << currentCorrelation->line3d.startPoint() << " endpoint: " << currentCorrelation->line3d.endPoint() << std::endl;
                std::cout << "direction: " << currentCorrelation->line3d.direction() << " origin: " << currentCorrelation->line3d.origin() << std::endl;

                std::cout << "line2d: " << currentCorrelation->leftLine->line.startPoint() << " endpoint: " << currentCorrelation->leftLine->line.endPoint() << std::endl;

                /*
                std::cout << "testLine: " << lTest.normal() << " dist: " << lTest.originDist() << std::endl;
                std::cout << "testLineP: " << testLine.normal() << " dist: " << testLine.originDist() << std::endl;
                currentCorrelation->line3d.projectPluecker();
                */

//                std::cout << "error: " << currentCorrelation->leftLine->line.error(lTest) << std::endl;
//                std::cout << "errorP: " << currentCorrelation->leftLine->line.error(testLine) << std::endl;
                std::cout << "errorSED: " << currentCorrelation->leftLine->line.error(lTest) << std::endl;
                std::cout << "errorSEDp: " << currentCorrelation->leftLine->line.error(testLine) << std::endl;


              //  std::cout << "lDiffS: " << lTest.startPoint().x - currentCorrelation->leftLine->line.x(lTest.startPoint().y) << std::endl;
              //  std::cout << "lDiffE: " << lTest.endPoint().x - currentCorrelation->leftLine->line.x(lTest.endPoint().y) << std::endl;
                //std::cout << "lDiffS: " << rTest.startPoint().x - currentCorrelation->rightLine->line.x(rTest.startPoint().y) << std::endl;
                //  std::cout << "rDiffE: " << rTest.endPoint().x - currentCorrelation->rightLine->line.x(rTest.endPoint().y) << std::endl << std::endl;

            }
        }
        std::cout << "projLeft: " << projLeft << std::endl;

        cv::Mat cameraView(cv::Mat::zeros(Size(im_grayLeftPainted.size().width,im_grayLeftPainted.size().height), CV_8UC3 ));
        cv::Mat cameraView2(cv::Mat::zeros(Size(im_grayLeftPainted.size().width,im_grayLeftPainted.size().height), CV_8UC3 ));

        std::vector<int> line2errorNr;
        std::vector<double> errorVal;

        // get left lines and corresponding 3d Lines
        for(int i = 0; i < leftAL->size(); i++){
            AnalyzedLine * currentLine = leftAL->at(i);
            if(currentLine->line3Nr){
                LineSegment3<double> line3 = lines3.at(currentLine->line3Nr);
                LineSegment<double> line2 = CameraPlueckerd::projectLineSegment(projLeft,line3);
                line2.draw(cameraView2, cv::Scalar(255, 255, 255));

                LineSegment<double> line22 = currentLine->line;
                line22.draw(cameraView2, cv::Scalar(255, 0, 0));
                auto error = currentLine->line.error(line2);

            //    std::cout << " error " << error << std::endl;

                errorVal.push_back(error);
                line2errorNr.push_back(i);
            }
        }
        for(int i = 0; i < rightAL->size(); i++){
            AnalyzedLine * currentLine = rightAL->at(i);
            if(currentLine->line3Nr){
                LineSegment3<double> line3 = lines3.at(currentLine->line3Nr);
                LineSegment<double> line2 = CameraPlueckerd::projectLineSegment(projRight,line3);
                line2.draw(cameraView2, cv::Scalar(255, 255, 255));

                LineSegment<double> line22 = currentLine->line;
                line22.draw(cameraView2, cv::Scalar(255, 0, 255));
                auto error = currentLine->line.error(line2);

            //    std::cout << " error " << error << std::endl;

                errorVal.push_back(error);
                line2errorNr.push_back(i);
            }
        }


        drawingTime1 = double(getTickCount());
        drawStereoLines(unDistImGray, false, 10, currentCorrelations);

        laLeft.drawCorrespondingLines(lineImageCorrespondences);


        // plot 3D lines which are currently in view
        for(int i = 0; i < currentLines3d.size(); i++){
            LineSegment3<double> line3 = currentLines3d.at(i);
            LineSegment<double> line2 = CameraPlueckerd::projectLineSegment(P1,line3);
            line2.draw(cameraView, cv::Scalar(255, 255, 255));
        }
/*
        projLeft = (getProjectionMatrix(cameraMatrixLeft, current_Rotation, current_Position)).clone();
        posRight = current_Position.clone(); // * R
        posRight = posRight - (R * current_Rotation * -T);
        projRight = (getProjectionMatrix(cameraMatrixRight, current_Rotation, posRight)).clone();
*/

        double errorSum = std::accumulate(errorVal.begin(), errorVal.end(), 0.0);
        double errorMean = errorSum / errorVal.size();
        double sq_sum = std::inner_product(errorVal.begin(), errorVal.end(), errorVal.begin(), 0.0);
        double variance = sq_sum / errorVal.size() - errorMean * errorMean;
        double stdev = std::sqrt(variance);

        std::cout << " errorSum " << errorSum << std::endl;
        std::cout << " errorMean " << errorMean << std::endl;
        std::cout << " sq_sum " << sq_sum << std::endl;
        std::cout << " variance " << variance << std::endl;
        std::cout << " stdev " << stdev << std::endl;
/*
        for(int i = 0; i < errorVal.size(); ++i) {
            if(errorVal.at(i) > (errorMean + stdev)){

                AnalyzedLine * currentLine = leftAL->at(line2errorNr.at(i));
                Line3Segment<double> line3 = lines3.erase(currentLine->line3Nr);


            }
            std::cout << " it " << *it << std::endl;
        }
*/
/*
        double scale = 0.75;
        resize(unDistImGray, unDistImGray, Size(unDistImGray.size().width * scale, unDistImGray.size().height * scale));
        resize(lineImageCorrespondences, lineImageCorrespondences, Size(lineImageCorrespondences.size().width * scale, lineImageCorrespondences.size().height * scale));
        resize(im_grayLeftPainted, im_grayLeftPainted, Size(im_grayLeftPainted.size().width * scale, im_grayLeftPainted.size().height * scale));
        resize(im_grayRightPainted, im_grayRightPainted, Size(im_grayRightPainted.size().width * scale, im_grayRightPainted.size().height * scale));
*/
        imshow("GrayImage", unDistImGray);
//        imshow("MixedColors", lineImageMixedColors);
        imshow("Tracking left", lineImageCorrespondences);
        imshow("left", im_grayLeftPainted);
        imshow("right", im_grayRightPainted);
        imshow("cameraView", cameraView);
        imshow("cameraView2", cameraView2);

//        imshow("modgrad", imageDataLeft[3]!= -1024.0f);

        if(cycleNum == 1){
            moveWindow("GrayImage", 50, 70);
            //moveWindow("GreyImage", im_gray.size().width + 150, im_gray.size().height + 80);
            moveWindow("left", 100, im_gray.size().height + 80);
            moveWindow("right", im_grayRight.size().width + 150, im_gray.size().height + 80);
            moveWindow("Correspondences", 100, 35);
            moveWindow("Tracking left", 100, 55);
            moveWindow("cameraView", 100, 15);
            moveWindow("cameraView2", 120, 25);
        }

        cv::waitKey(0);

        last_key_press = cvWaitKey(1)%256;  // wegen bug %256..
        if (last_key_press == 'p'){         // pause function
            last_key_press = 0;
            while (last_key_press != 'p')
                last_key_press = cvWaitKey(50)%256; // wegen bug %256..
        } else if(last_key_press == 'l'){
            drawLines = !drawLines;
        }
        //#ifdef MEASURE_TIME
        drawingTime2 += (double(getTickCount()) - drawingTime1) * 1000 / getTickFrequency();
        //#endif


        double percentageStableCorrespondences = (((double)laLeft.getStableCorrespondencesNum()) / ((double)detectedLinesLeft.size())) * 100.0;
        std::cout << "StableCorrespondences: " << laLeft.getStableCorrespondencesNum() << ", percentage of detected lines: " << percentageStableCorrespondences << "%" << std::endl;
        overallAverageStableCorrespondences = (cycleNum * overallAverageStableCorrespondences + percentageStableCorrespondences) / (cycleNum + 1);
        //std::cout << "overallAverageStableCorrespondences: " << overallAverageStableCorrespondences << "%" << std::endl;

        //#ifdef MEASURE_TIME
        std::cout << "captureTime: " << captureTime2 << std::endl;
        std::cout << "convertTime: " << convertTime2 << std::endl;
        std::cout << "resizeTime: " << resizeTime2 << std::endl;
        std::cout << "undistortTime: " << undistortTime2 << std::endl;
        std::cout << "lsd detectTime: " << detectTime2 << std::endl;
        std::cout << "analyzeTime: " << analyzeTime2 << std::endl;
        std::cout << "trackingTime: " << trackingTime2 << std::endl;
        std::cout << "stereoTime: " << stereoTime2 << std::endl;
        std::cout << "drawingTime: " << drawingTime2 << std::endl;

        std::cout << "sum: " << captureTime2 + convertTime2 + resizeTime2 + undistortTime2 + detectTime2 + analyzeTime2 + trackingTime2 + stereoTime2 + drawingTime2 << std::endl;

        duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
        totalDuration += duration_ms;
        std::cout << "OpenCV lsd - red\n\t" << detectedLinesLeft.size() <<" line segments found. For " << duration_ms << " ms, avg per frame: " << totalDuration/(double)cycleNum << " ms." << std::endl;

        //#endif

        cycleNum++;
    }

    capture.release();
    std::cout << "End." << std::endl;

    return 0;
}
