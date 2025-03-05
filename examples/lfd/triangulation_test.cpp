#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include <lfd/MotionDescriptor.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <lfd/MotionLineMatcher.hpp>
#include <list>
#include <slam/pose_estimator.hpp>
#include <geometry/stereocv.hpp>
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/PairwiseLineMatcher.hpp>
#include <lfd/StereoLineMatcher.hpp>
#include <iomanip>


using namespace std;
using namespace lsfm;
using namespace cv;


//! Calculates the width of the black border on each side of the image, order: top, bottom, left, right
//! Function has to be changed if any of the Borders reach more than one quater of the width/height into the image
template<class MTYPE>
void findBlackCroppingArea(const cv::Mat img, std::vector<int>& border){

    cv::Mat upper  = img(Rect(img.size().width / 4, 0,img.size().width / 2,img.size().height / 2));
    cv::Mat lower  = img(Rect(img.size().width / 4, img.size().height / 2, img.size().width / 2, img.size().height / 2));
    cv::Mat left  = img(Rect(0, img.size().height / 4, img.size().width / 2,img.size().height / 2));
    cv::Mat right  = img(Rect(img.size().width / 2, img.size().height / 4, img.size().width / 2, img.size().height / 2));

    bool done = false;
    for(int i = upper.rows - 1; i >= 0  && !done; i--){
        for(int j = 0; j < upper.cols && !done; j++){
            if(255 != upper.at<MTYPE>(i, j)){
                border.push_back(i);
                done = true;
            }
        }
    }
    done = false;
    for(int i = 0; i < lower.rows  && !done; i++){
        for(int j = 0; j < lower.cols && !done; j++){
            if(255 != lower.at<MTYPE>(i, j)){
                border.push_back(lower.rows - i);
                done = true;
            }
        }
    }
    done = false;
    for(int i = left.cols - 1; i >= 0  && !done; i--){
        for(int j = 0; j < left.rows && !done; j++){
            if(255 != left.at<MTYPE>(j, i)){
                border.push_back(i);
                done = true;
            }
        }
    }
    done = false;
    for(int i = 0; i < right.cols && !done; i++){
        for(int j = 0; j < right.rows && !done; j++){
            if(255 != right.at<MTYPE>(j, i)){
                border.push_back(right.cols - i);
                done = true;
            }
        }
    }
}

//! Calculates Undistort-Rectify Map
int stereoRectification(const std::string intrinsic_filename, const std::string extrinsic_filename, const Size img_size, const double scaleFactor, cv::Mat& P1, cv::Mat& P2, cv::Mat& map11, cv::Mat& map12, cv::Mat& map21, cv::Mat& map22){

    cv::Mat Q;
    cv::Mat R, T, R1, R2;
    Rect roi1, roi2;

    // reading intrinsic parameters
    FileStorage fs(intrinsic_filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file intrinsic_filename\n");
        return -1;
    }

    cv::Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scaleFactor;
    M2 *= scaleFactor;

    fs.open(extrinsic_filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file extrinsic_filename\n");
        return -1;
    }

    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    // cv::Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
    return 1;
}

void coutPairVector(std::vector<std::pair<int, int>> pixel){
    for(int i = 0; i < pixel.size(); ++i){
        std::cout << pixel.at(i).first << " & " << pixel.at(i).second << ", ";
    }
    std::cout << std::endl;
}


int main(int argc, char** argv)
{
    std::string filename1 = "../../Videos/euroc/t1_3_stereo.avi";
    std::string filename2 = "../../Videos/euroc/intrinsics.yml";
    std::string filename3 = "../../Videos/euroc/extrinsics.yml";

    if (argc > 2) {
        filename1 = argv[1];
        filename2 = argv[2];
        filename3 = argv[3];
    }

    cv::Mat frames, imL, imR;
    VideoCapture capture;
    capture = VideoCapture(filename1);
    capture.grab();
    capture.retrieve(frames, IMREAD_GRAYSCALE);
    cvtColor(frames, frames, cv::COLOR_RGB2GRAY);
    std:: cout << "type: " << frames.type() << std::endl;

    double scaleFactor = 1.0;
    Size img_size;
    img_size.width = (frames.size().width / 2) * scaleFactor;
    img_size.height = frames.size().height * scaleFactor;

    cv::Mat P1, P2, map11, map12, map21, map22;

    if (!stereoRectification( filename2, filename3, img_size, scaleFactor, P1, P2, map11, map12, map21, map22))
    {
        cout << "Can not open files " << endl;
        return -1;
    }

    lsfm::Matx34<double> P1m(P1.ptr<double>()), P2m(P2.ptr<double>());

    // find black borders
    cv::Mat borderDetect = cv::Mat::ones(frames.rows, frames.cols / 2, frames.type());
    borderDetect *= 255;
    cv::Mat bordersRecti;
    remap(borderDetect, bordersRecti, map11, map12, INTER_LINEAR);      // Undistort
    std::vector<int> border;
    findBlackCroppingArea<uchar>(bordersRecti, border);

    borderDetect = cv::Mat::ones(frames.rows, frames.cols / 2, frames.type());
    borderDetect *= 255;
    remap(borderDetect, bordersRecti, map21, map22, INTER_LINEAR);      // Undistort
    findBlackCroppingArea<uchar>(bordersRecti, border);

    int croppingHeight = std::max(border[0], std::max(border[1], std::max(border[4], border[5])));
    int croppingWidth = std::max(border[2], std::max(border[3], std::max(border[6], border[7])));


    //GaussianBlur(src1, src1, Size(3, 3), 0.6);

    typedef double MyType;

    LsdCC<MyType> lsdL(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.008, 0.004, 30, 0, 2);
    LsdCC<MyType> lsdR(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.008, 0.004, 30, 0, 2);

    imL  = frames(cv::Range(0, frames.size().height), cv::Range(0, frames.size().width/2)); //no data copying here - Range excludes the last value of width, thus correct.
    imR = frames(cv::Range(0, frames.size().height), cv::Range(frames.size().width/2, frames.size().width)); //no data copying here

    cv::Mat img1r, img2r;
    remap(imL, img1r, map11, map12, INTER_LINEAR);      // Undistort
    remap(imR, img2r, map21, map22, INTER_LINEAR);

    imL = img1r(Rect(croppingWidth,croppingHeight,img1r.size().width-croppingWidth*2,img1r.size().height-croppingHeight*2)).clone();
    imR = img2r(Rect(croppingWidth,croppingHeight,img2r.size().width-croppingWidth*2,img2r.size().height-croppingHeight*2)).clone();

    std::vector<FeatureMatch<MyType>> matches;
    std::vector<DescriptorMatch<MyType>> motionMatches, motionMatchesRes;
    std::cout << "width: " << imL.size().width << " height: " << imL.size().height << std::endl;
    MotionLineFilter<MyType, LsdCC<MyType>::LineSegmentVector> mlf(200, 20, imL.size().width, imL.size().height);
    std::vector<size_t> maskN, maskP;

    LsdCC<MyType>::LineSegmentVector previousLeftLines, previousRightLines;
    cv::Mat previousImGrayLeft = imL;

    //std::cout << (8%8) << "  " << (9%8) << "  " << (7%8) << "  " << (-2%8) << "  " << (-0%8) << std::endl;


    int last_key_press = 0;
    while (last_key_press != 'q')
    {

        imL  = frames(cv::Range(0, frames.size().height), cv::Range(0, frames.size().width/2)); //no data copying here - Range excludes the last value of width, so correct.
        imR = frames(cv::Range(0, frames.size().height), cv::Range(frames.size().width/2, frames.size().width)); //no data copying here
/*
        resize(im_grayLeft, im_grayLeft, Size(im_grayLeft.size().width * scaleFactor, im_grayLeft.size().height * scaleFactor));
        resize(im_grayRight, im_grayRight, Size(im_grayRight.size().width * scaleFactor, im_grayRight.size().height * scaleFactor));
*/
        cv::Mat img1r, img2r;
        remap(imL, img1r, map11, map12, INTER_LINEAR);      // Undistort
        remap(imR, img2r, map21, map22, INTER_LINEAR);

        imL = img1r(Rect(croppingWidth,croppingHeight,img1r.size().width-croppingWidth*2,img1r.size().height-croppingHeight*2)).clone();
        imR = img2r(Rect(croppingWidth,croppingHeight,img2r.size().width-croppingWidth*2,img2r.size().height-croppingHeight*2)).clone();

        lsdL.detect(imL);
        lsdR.detect(imR);

        MatMap dataL, dataR;
        dataL["gx"] = lsdL.imageData()[0];
        dataL["gy"] = lsdL.imageData()[1];
        dataL["img"] = imL;

        dataR["gx"] = lsdR.imageData()[0];
        dataR["gy"] = lsdR.imageData()[1];
        dataR["img"] = imR;

        std::cout << "p2: " << P2 << std::endl;
        cv::Mat_<MyType> cam, rot;
        cv::Mat_<MyType> trans;
        decomposeProjectionMatrix(P2,cam,rot,trans);
        cv::Mat_<MyType> p2new = getProjectionMatrix(cam,rot,trans);
        std::cout << "p2new: " << p2new << std::endl;
        std::cout << "trans: " << trans << std::endl;

        typedef GchGradImgInterpolate<MyType> MyGchHelper;
        typedef FdcGenericLR<MyType,LsdCC<MyType>::LineSegment,MyGchHelper> MyFdc;

        /*typedef FdcLBD<MyType, LsdCC<MyType>::LineSegment, short, FastRoundNearestInterpolator<MyType, short>> MyFdc;
        StereoLineFilter<MyType, LsdCC<MyType>::LineSegmentVector> slf(imL.rows, imL.cols / 2);
        PairwiseLineMatcher<MyType, typename MyFdc::descriptor_type> slm;
        std::vector<typename MyFdc::descriptor_type> dscL, dscR;
        std::vector<size_t> maskL, maskR;*/

        // Stereo matching ---------------------------------------------------------
        MyFdc::FdcPtr fdcL = MyFdc::createFdc(dataL);
        MyFdc::FdcPtr fdcR = MyFdc::createFdc(dataR);
        StereoLineMatcher<MyType, LsdCC<MyType>::LineSegmentVector,MyGchHelper> slm(fdcL, fdcR, imL.rows, imL.cols / 2,5,0.6);

        std::vector<DescriptorMatch<MyType>> bfmatches2, bfmatches3;

        /*slf.create(lsdL.lineSegments(), lsdR.lineSegments(), bfmatches3, maskL, maskR);
        fdcL->createList(lsdL.lineSegments(), maskL, dscL);
        fdcR->createList(lsdR.lineSegments(), maskR, dscR);
        slm.match1D(lsdL.lineSegments(),lsdR.lineSegments(),dscL, dscR, bfmatches3,bfmatches2);*/


        slm.match(lsdL.lineSegments(),lsdR.lineSegments(),bfmatches3);
        MyType mean = 0;
        for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<MyType>& data){
            mean += data.distance;
        });

        mean /= bfmatches3.size();
        std::cout << "mean distance: " << mean << std::endl;
        for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<MyType>& data){
            //mean += data.distance;
            //if (data.distance < mean)
            //if (data.distance < 500)
                bfmatches2.push_back(data);
        });

        std::cout <<  "final matches: " << bfmatches2.size() << std::endl;
        imshow("Detected Stereo matches", drawMatches<MyType,DescriptorMatch<MyType>>(imL, lsdL.lineSegments(), imR, lsdR.lineSegments(), bfmatches2));

        // Triangulation ----------------------------------------------------



        StereoCV<MyType> stereoCV(P1m,P2m);
        StereoPlane<MyType> stereoPlane(P1m,P2m);
        CameraCV<MyType> camCVL(P1m);
        CameraCV<MyType> camCVR(P2m);
        CameraPluecker<MyType> camPlL(P1m);
        CameraPluecker<MyType> camPlR(P2m);

        Camera<MyType> myCamL(P1m);
        Camera<MyType> myCamR(P2m);


        std::cout << "P_L: " << P1 << std::endl;
        std::cout << "CPL: " << myCamL.projM() << std::endl;
        std::cout << "P_R: " << P2 << std::endl;
        std::cout << "CPR: " << myCamR.projM() << std::endl;

        std::cout << "RL: " << myCamL.rotM() << std::endl;
        std::cout << "RR: " << myCamR.rotM() << std::endl;

        std::cout << std::fixed << std::setprecision(9);
        std::vector<LineSegment<MyType>> prjLCV, prjL;
        cv::Mat triLeft = imL.clone();

        int i = 0;
        for_each(bfmatches2.begin(), bfmatches2.end(), [&](const DescriptorMatch<MyType>& match){
            std::cout << match.queryIdx << " <=> " << match.queryIdx << ", idx: " << i++ << std::endl;
            LineSegment<MyType> lsL = lsdL.lineSegments()[match.queryIdx];
            LineSegment<MyType> lsR = lsdR.lineSegments()[match.matchIdx];
            LineSegment<MyType> prjLCV, prjL;

            LineSegment3<MyType> l3sCV = stereoCV.triangulate(lsL,lsR);
            LineSegment3<MyType> l3s = stereoPlane.triangulate(lsL,lsR);

            std::cout << "3dCV  - origin: " << l3sCV.origin() << ", dir: " << l3sCV.direction() << ", start: " << l3sCV.startPoint() << ", end: " << l3sCV.endPoint() << std::endl;
            std::cout << "3dpl  - origin: " << l3s.origin() << ", dir: " << l3s.direction() << ", start: " << l3s.startPoint() << ", end: " << l3s.endPoint() << std::endl;
            std::cout << "difference: dir: " << l3sCV.direction() - l3s.direction() << ", start: " << l3sCV.startPoint() - l3s.startPoint() << ", end: " << l3sCV.endPoint() - l3s.endPoint() << std::endl;

            std::cout << "left  - origin: " << lsL.origin() << ", dir: " << lsL.direction() << ", start: " << lsL.startPoint() << ", end: " << lsL.endPoint() << std::endl;
            std::cout << "right - origin: " << lsR.origin() << ", dir: " << lsR.direction() << ", start: " << lsR.startPoint() << ", end: " << lsR.endPoint() << std::endl;


  /*          prjLCV  = camCVL.project(l3sCV);
            std::cout << "projL - origin: " << prjLCV.origin() << ", dir: " << prjLCV.direction() << ", start: " << prjLCV.startPoint() << ", end: " << prjLCV.endPoint() << std::endl;
            prjLCV  = camPlL.project(l3sCV);
            std::cout << "pPlüL - origin: " << prjLCV.origin() << ", dir: " << prjLCV.direction() << ", start: " << prjLCV.startPoint() << ", end: " << prjLCV.endPoint() << std::endl;
            prjLCV  = camCVR.project(l3sCV);
            std::cout << "projR - origin: " << prjLCV.origin() << ", dir: " << prjLCV.direction() << ", start: " << prjLCV.startPoint() << ", end: " << prjLCV.endPoint() << std::endl;
            prjLCV  = camPlR.project(l3sCV);
            std::cout << "pPlüR - origin: " << prjLCV.origin() << ", dir: " << prjLCV.direction() << ", start: " << prjLCV.startPoint() << ", end: " << prjLCV.endPoint() << std::endl;

            prjL  = camCVL.project(l3s);
            std::cout << "projL - origin: " << prjL.origin() << ", dir: " << prjL.direction() << ", start: " << prjL.startPoint() << ", end: " << prjL.endPoint() << std::endl;
            prjL  = camPlL.project(l3s);
            std::cout << "pPlüL - origin: " << prjL.origin() << ", dir: " << prjL.direction() << ", start: " << prjL.startPoint() << ", end: " << prjL.endPoint() << std::endl;
            prjL  = camCVR.project(l3s);
            std::cout << "projR - origin: " << prjL.origin() << ", dir: " << prjL.direction() << ", start: " << prjL.startPoint() << ", end: " << prjL.endPoint() << std::endl;
            prjL  = camPlR.project(l3s);
            std::cout << "pPlüR - origin: " << prjL.origin() << ", dir: " << prjL.direction() << ", start: " << prjL.startPoint() << ", end: " << prjL.endPoint() << std::endl;*/

            //drawTriLines(triLeft,ls1,prjLCV,prjL);

        });


        // Tracking ---------------------------------------------------------

        //typedef FdcLBD<MyType, LsdCC<MyType>::LineSegment, short, FastRoundNearestInterpolator<MyType, short>> LbdFdc;
        //LbdFdc::FdcPtr fdc1 = LbdFdc::createFdc(lsdL.imageData()[0], lsdL.imageData()[1], 7, 5);
        //LbdFdc::FdcPtr fdc2 = LbdFdc::createFdc(lsdR.imageData()[0], lsdR.imageData()[1], 7, 5);


        typedef FdcMotion<MyType,LsdCC<MyType>::LineSegment> MyMotionFdc;
        MyMotionFdc::FdcPtr fdcM1 = MyMotionFdc::createFdc();
        MyMotionFdc::FdcPtr fdcM2 = MyMotionFdc::createFdc();


        std::cout << "width: " << imL.size().width << " height: " << imL.size().height << std::endl;
        MotionLineMatcher<MyType, LsdCC<MyType>::LineSegmentVector,MyMotionFdc> mlm(fdcM1, fdcM2, imL.cols,  imL.rows);


        double start = double(getTickCount());
        std::pair<MyType, MyType> movement(0,0);

        mlf.create(lsdL.lineSegments(), previousLeftLines, movement, matches, maskN, maskP);

        double end = double(getTickCount());
        std::cout << "stereo line filter time: " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;
        std::cout << "lines left: " << lsdL.lineSegments().size() << ", lines right: " << lsdR.lineSegments().size() << std::endl;
        //size_t num = matches.size();
        size_t num = matches.size();

        std::cout << "reduced candidates from " << lsdL.lineSegments().size() * lsdR.lineSegments().size() << " to " << num << std::endl;


        start = double(getTickCount());
        motionMatches.clear();
        motionMatchesRes.clear();
        mlm.match(lsdL.lineSegments(), previousLeftLines, movement, motionMatches);
        //mlm.match(previousLeftLines, lsd1.lineSegments(), movement, motionMatches);

/*
        PairwiseLineMatcher<MyType, typename LbdFdc::descriptor_type> pmatcher;
        if(motionMatches.size() > 1)
            pmatcher.match1D(lsd1.lineSegments(), lsd2.lineSegments(), mlm.getDscNew(), mlm.getDscPrev(), motionMatches, motionMatchesRes);
*/

        end = double(getTickCount());
        std::cout <<  "mlm matches: " << motionMatches.size() << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;

        imshow("Detected tracking matches", drawMatches<MyType,DescriptorMatch<MyType>>(imL, lsdL.lineSegments(), previousImGrayLeft, previousLeftLines, motionMatches));
        //imshow("Detected matches", drawMatches<MyType,DescriptorMatch<MyType>>(previousImGrayLeft, previousLeftLines, im_grayLeft, lsd1.lineSegments(), motionMatches));


        cv::Mat linesPrevLeft = previousImGrayLeft;
        cv::Mat linesLeft = imL.clone();
        cv::Mat linesRight = imR.clone();
        drawLines<MyType>(linesLeft, lsdL.lineSegments());
        drawLines<MyType>(linesRight, lsdR.lineSegments());
        drawLines<MyType>(linesPrevLeft, previousLeftLines);
        imshow("leftLines", linesLeft);
        imshow("rightLeft", linesRight);
        imshow("prevLeftLines", linesPrevLeft);


        // get new frame
        if(!capture.grab())
            break;
        capture.retrieve(frames, IMREAD_GRAYSCALE);
        cvtColor(frames, frames, cv::COLOR_RGB2GRAY);

        previousImGrayLeft = imL;
        previousLeftLines = lsdL.lineSegments();
        previousRightLines = lsdR.lineSegments();

        last_key_press = waitKey();
        //lsfm::waitKey(100000);
        //last_key_press = cvWaitKey(1)%256;
    }



    return 0;
}
