#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <list>

#define STEREO_FAST
#define USE_CERES_JET
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <slam/line_jet.hpp>
#include <slam/pose_estimator.hpp>
#include <slam/slamDataModel.hpp>
#include <geometry/stereo.hpp>

#include <lfd/StereoLineFilter.hpp>
#include <lfd/StereoLineMatcher.hpp>
#include <lfd/MotionDescriptor.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <lfd/MotionLineMatcher.hpp>


using namespace std;
using namespace lsfm;


typedef double FT;
typedef lsfm::LineSegment<FT> MyLine2;
typedef lsfm::Line3<FT> MyLine3D;

struct observationLine{
    int frameID;
    int lineID;
//    FT endPoints[4];
    lsfm::LineSegment<FT> lineSegment;
};


#define NUM_CAMS 1

template<class Line2, class Line3,class Cam>
void camMode(const std::vector<Line3> &data, Cam cam, const std::string &name) {

    typedef typename Line2::float_type FT;
    vector<Line2> res;
    Vec3<FT> origin = cam.origin();
    Vec3<FT> orientation = cam.orientation();
    cv::Mat img(static_cast<int>(ceil(cam.height())),static_cast<int>(ceil(cam.width())),CV_8UC3);
    img.setTo(cv::Vec3b(0,0,0));

    auto updateMode = [&](int mode, Vec3<FT> val) {
        switch(mode) {
            case 'r':
                swap(val.x(),val.y());
                val *= CV_PI / 180;
                cam.rotate(val);
            break;
            case 'o':
                swap(val.x(),val.y());
                val *= CV_PI / 180;
                cam.rotate(val,Vec3<FT>(0,0,0));
            break;
            case 't':
                val *= 0.1;
                cam.translate(val);
            break;
        }
    };

    int key = 0;
    int mode = 'r';
    while(key != 'q') {
        //cout << key << endl;
        switch(key) {
            case 'r':
            mode = key;
            cout << "switched to rotation mode (rotate camera around world origin)" << endl;
            break;
            case 'o':
            mode = key;
            cout << "switched to orientation mode (rotate camera around camera origin)" << endl;
            break;
            case 't':
                mode = key;
                cout << "switched to translation mode" << endl;
            break;
            case 'c':
                cout << "clear data" << endl;
                cam.origin(origin);
                cam.orientation(orientation);

            break;
            case 81: // left
            case 2424832:
                updateMode(mode,Vec3<FT>(1,0,0));
            break;
            case 82: // up
            case 2490368:
                updateMode(mode,Vec3<FT>(0,1,0));
            break;
            case 83: // right
            case 2555904:
                updateMode(mode,Vec3<FT>(-1,0,0));
            break;
            case 84: // down
            case 2621440:
                updateMode(mode,Vec3<FT>(0,-1,0));
            break;
            case 85: // bup
            case 2162688:
                updateMode(mode,Vec3<FT>(0,0,1));
            break;
            case 86: // bdown
            case 2228224:
                updateMode(mode,Vec3<FT>(0,0,-1));
            break;

        }

        cam.project(data,res);
        imshow(name, drawGeometry(res,img));

    #ifdef WIN32
        key = cv::waitKey();
    #else
        key = cv::waitKey() % 256;
    #endif

    }
}

int main(int argc, char** argv)
{
    std::string filename1 = "../../Datasets/euroc/intrinsics.yml";
    std::string filename2 = "../../Datasets/A350calibration/cubered/lines.txt";

    if (argc > 2) {
        filename1 = argv[1];
        filename2 = argv[2];
    }


    std::vector<lsfm::Vec3<FT>> translationsVec;
    std::vector<lsfm::Matx33<FT>> rotationsMat;
    std::vector<std::vector<observationLine>> imageLineSegments;
    int frameCount = -1;

    std::ifstream infile(filename2);
    std::string line;
    char nextType;
    while (std::getline(infile, line))
    {
        if(line.size() <= 0)
            continue;

        if(line.at(0) == '#')
            continue;


        if(line.at(0) == 'T'){
            nextType = 'T';
            continue;
        }
        if(line.at(0) == 'R'){
            nextType = 'R';
            continue;
        }
        if(line.at(0) == 'L'){
            nextType = 'L';
            frameCount++;
            while(imageLineSegments.size() < frameCount + 1){
                imageLineSegments.push_back(std::vector<observationLine>());
            }
            continue;
        }

        std::istringstream iss(line);

        if(nextType == 'T'){
            FT tmpVal[3];
            iss >> tmpVal[0] >> tmpVal[1] >> tmpVal[2];
            lsfm::Vec3<FT> tmpTrans(tmpVal);
            translationsVec.push_back(tmpTrans);
            //translations.push_back(cT);
            continue;
        }
        if(nextType == 'R'){

            FT cR[9];
            lsfm::Matx33<FT> tmpMat;
            iss >> cR[0] >> cR[1] >> cR[2] >> cR[3] >> cR[4] >> cR[5] >> cR[6] >> cR[7] >> cR[8];
            tmpMat << cR[0] , cR[1] , cR[2] , cR[3] , cR[4] , cR[5] , cR[6] , cR[7] , cR[8];
            rotationsMat.push_back(tmpMat);
            //rotations.push_back(cR);
            continue;
        }
        if(nextType == 'L'){

            observationLine tmpObsLine;
            FT tmpF[4];
            iss >> tmpObsLine.lineID >> tmpF[0] >> tmpF[1] >> tmpF[2] >> tmpF[3];
            --tmpObsLine.lineID;
            tmpObsLine.frameID = frameCount;
            tmpObsLine.lineSegment = lsfm::LineSegment<FT>(tmpF);
            imageLineSegments.at(frameCount).push_back(tmpObsLine);
            continue;
        }

    }


    std::cout << " rodrigues 0 to mat: " << std::endl << lsfm::rodrigues(Vec3<FT> (0,0,0)) << std::endl;
    std::cout << " rodrigues 0 to mat: " << std::endl << lsfm::Matx33<FT> (1,0,0 ,0,1,0  ,0,0,1) << std::endl << std::endl << lsfm::rodrigues(lsfm::Matx33<FT> (1,0,0 ,0,1,0  ,0,0,1)) << std::endl;
  //  std::cout << " rodrigues identity mat to rod: " << lsfm::Matx33<FT> test = lsfm::rodrigues(Vec3<FT> (0,0,0)) << std::endl;

    for(int i = 0; i < translationsVec.size(); ++i){
        std::cout << "trans: "  << translationsVec.at(i) << std::endl;
    }
    for(int i = 0; i < rotationsMat.size(); ++i){
        std::cout << "rotations: "  << rotationsMat.at(i) << std::endl << std::endl;
    }

    for(int i = 0; i < imageLineSegments.size(); ++i){
//        std::cout << "imageLineSegments: "  << imageLineSegments.at(i).lineID << "   "  << imageLineSegments.at(i).frameID << "   "  << imageLineSegments.at(i).lineSegment.startPoint() << std::endl;
    }


    std::vector<lsfm::CameraPlueckerd> cameras;

    for(int i = 0; i < 9; ++i){
        cameras.push_back(lsfm::CameraPlueckerd(lsfm::Vec2<FT>(3639.9100328478412/10.0, 3630.1276155101245/10.0), lsfm::Vec2<FT>(2292.1746556658695/10.0, 1743.4509020150654/10.0),
                              lsfm::Vec2<FT>(4608/10.0, 3456/10.0), translationsVec.at(i), rotationsMat.at(i)));

    }


//    lsfm::CameraPlueckerd cameraShow(lsfm::Vec2<FT>(3639.9100328478412/10.0, 3630.1276155101245/10.0), lsfm::Vec2<FT>(2292.1746556658695/10.0, 1743.4509020150654/10.0),
//                          lsfm::Vec2<FT>(4608/10.0, 3456/10.0), translationsVec.at(0), rotationsMat.at(0));


    //lsfm::LineSegment2d

    /*lsfm::StereoPlane<FT> testStereo(cameras.at(0), cameras.at(1));
    vector<lsfm::LineSegment3<FT>> lines3d1;

    for(int i = 0; i < imageLineSegments.at(0).size(); ++i){
        for(int j = 0; j < imageLineSegments.at(1).size(); ++j){
            if(imageLineSegments.at(0).at(i).lineID == imageLineSegments.at(1).at(j).lineID){
                lsfm::LineSegment3<FT> testLine3 = testStereo.triangulate(imageLineSegments.at(0).at(i).lineSegment, imageLineSegments.at(1).at(j).lineSegment);
                lines3d1.push_back(testLine3);
            }
        }
    }

    vector<lsfm::LineSegment<FT>> set0, set1;
    lsfm::Vec2<FT> size(4608/10.0,3456/10.0);
    cv::Mat img(static_cast<int>(ceil(size.y())),static_cast<int>(ceil(size.x())),CV_8UC3);

    cameras[0].project(lines3d1,set0);
//    imshow("projected lines 1 pluecker cam0", drawGeometry(set0,img));


    cameras[1].project(lines3d1,set1);
//    imshow("projected lines 1 pluecker cam1", drawGeometry(set1,img));


    vector<lsfm::LineSegment<FT>> groundTruthSegments, groundTruthSegmentsA, groundTruthSegmentsB;
    for(int i = 0; i < imageLineSegments.at(0).size(); ++i){
        groundTruthSegments.push_back(imageLineSegments.at(0)[i].lineSegment);
    }
//    imshow("groundTruth lines 1 pluecker cam0", drawGeometry(groundTruthSegments,img));



    std::cout << cameras[0].camM() << std::endl;
    for(int i = 0; i < lines3d1.size(); ++i){
        std::cout << lines3d1[i].startPoint() << ", "  << std::endl << lines3d1[i].endPoint() << std::endl << "len: " << lines3d1[i].length() << std::endl;

    }*/


    // Create some initial Models   --------------------------------------------------

    lsfm::Vec2<FT> size(4608/10.0,3456/10.0);
    cv::Mat img(static_cast<int>(ceil(size.y())),static_cast<int>(ceil(size.x())),CV_8UC3);
    vector<lsfm::LineSegment<FT>> groundTruthSegmentsA, groundTruthSegmentsB;
    //vector<lsfm::Line3<FT>> lines3d2, current3dLines;
    vector<lsfm::LineSegment3<FT>> lines3d2, current3dLines;
    std::vector<int> existingModels;

    for(int k = 0; k < imageLineSegments.size(); ++k){
        groundTruthSegmentsA.clear();
        groundTruthSegmentsB.clear();
        lsfm::StereoPlane<FT> testStereo(cameras.at(k), cameras.at((k+1) % imageLineSegments.size()));
        current3dLines.clear();
        for(int i = 0; i < imageLineSegments.at(k).size(); ++i){
            //groundTruthSegmentsA.push_back(imageLineSegments.at(k)[i].lineSegment);
            for(int j = 0; j < imageLineSegments.at((k+1) % imageLineSegments.size()).size(); ++j){
                //groundTruthSegmentsB.push_back(imageLineSegments.at((k+1) % imageLineSegments.size())[j].lineSegment);

                if(imageLineSegments.at(k).at(i).lineID == imageLineSegments.at((k+1) % imageLineSegments.size()).at(j).lineID){
                    groundTruthSegmentsA.push_back(imageLineSegments.at(k)[i].lineSegment);
                    groundTruthSegmentsB.push_back(imageLineSegments.at((k+1) % imageLineSegments.size())[j].lineSegment);

                    if(imageLineSegments.at(k).at(i).lineID >= existingModels.size()){
                        existingModels.resize(imageLineSegments.at(k).at(i).lineID + 1, 0);
                        lines3d2.resize(imageLineSegments.at(k).at(i).lineID + 1);
                    }
                    if(!existingModels.at(imageLineSegments.at(k).at(i).lineID)){
                        lsfm::LineSegment3<FT> testLine3 = testStereo.triangulate(imageLineSegments.at(k).at(i).lineSegment, imageLineSegments.at((k+1) % imageLineSegments.size()).at(j).lineSegment);
                        //current3dLines.push_back(testLine3);
                        lines3d2.at(imageLineSegments.at(k).at(i).lineID) = testLine3;
                        existingModels.at(imageLineSegments.at(k).at(i).lineID) = 1;
                    }
                    current3dLines.push_back(testStereo.triangulate(groundTruthSegmentsA.back(), groundTruthSegmentsB.back()));
                }
            }
        }

        //current3dLines.push_back(testLine3);
        //lines3d2.at(imageLineSegments.at(k).at(i).lineID) = testLine3;
        //existingModels.at(imageLineSegments.at(k).at(i).lineID) = 1;

        //std::cout << "camA: " << k << std::endl;
        //imshow("gtA", drawGeometry(groundTruthSegmentsA,img));
        //imshow("gtB", drawGeometry(groundTruthSegmentsB,img));

        //vector<lsfm::Line<FT>> lineSet2d;
        vector<lsfm::LineSegment<FT>> lineSet2d;
        cameras.at(k).project(current3dLines,lineSet2d);
        //imshow("projected current lines 2 pluecker camA", drawGeometry(lineSet2d,img));

        cameras.at((k+1) % imageLineSegments.size()).project(current3dLines,lineSet2d);
        //imshow("projected current lines 2 pluecker camB", drawGeometry(lineSet2d,img));


        //cv::waitKey(1000000);
        //camMode<LineSegment<FT>>(current3dLines,cameras.at(k),"projected lines");
    }

    lsfm::StereoPlane<FT> testStereo(cameras.at(0), cameras.at(6));
    lines3d2[4] =  testStereo.triangulate(imageLineSegments.at(0).at(4).lineSegment, imageLineSegments.at(6).at(4).lineSegment);
    lines3d2[10] = testStereo.triangulate(imageLineSegments.at(0).at(6).lineSegment, imageLineSegments.at(6).at(6).lineSegment);



    vector<lsfm::LineSegment<FT>> set2;


    Camera<FT> cam = cameras[0];
    cam.origin(Vec3<FT>(0,0,0));
    cam.orientation(Vec3<FT>(0,0,0));
    typedef lsfm::SlamModel<FT, NUM_CAMS, MyLine2, MyLine3D> MySlam;
    MySlam slam(&cam);

    std::vector<MyLine2> lineSegmentList;
    std::vector<DetectedLine> detectedLineList;
    std::vector<ModelLine<MyLine3D>> modelLineList;
    int detectedLinesCtr = 0;

    for(int i = 0; i < lines3d2.size(); ++i){
        modelLineList.push_back(ModelLine<MyLine3D>(lines3d2.at(i)));
    }

	slam.addData(modelLineList);

    const int numFrames = 9;
	std::vector<int> fms;
    for(int i = 0; i < numFrames; ++i){
        lineSegmentList.clear();
        detectedLineList.clear();
        for(int j = 0; j < imageLineSegments.at(i).size(); ++j){
            //std::cout << i << ":" << j << ":" << imageLineSegments.at(i).at(j).lineID << std::endl;
            lineSegmentList.push_back(imageLineSegments.at(i).at(j).lineSegment);
            detectedLineList.push_back(DetectedLine(imageLineSegments.at(i).at(j).lineID));
        }
        slam.addData(lineSegmentList, detectedLineList, cameras.at(i));
		fms.push_back(i);
    }

    slam.bundleAdjustmentOnFrames(fms);

    std::vector<LineSegment3<FT>> lineSeg3Optimized = slam.getModelLineSegments();
    std::vector<Line3<FT>> line3Optimized = slam.getModelLines();

    int camNum = 0;
    FT error = 0;
    cameras[camNum].pose(slam.getRobotPoses()[camNum]);
	std::ostringstream camNumSS;
	camNumSS << camNum;


    std::vector<LineSegment<FT>> segmentsPnL;
    std::vector<Line3<FT>> modelsPnL;
    for(int i = 0; i < imageLineSegments.at(camNum).size(); ++i){
        segmentsPnL.push_back(imageLineSegments.at(camNum).at(i).lineSegment);
        modelsPnL.push_back(line3Optimized.at(imageLineSegments.at(camNum).at(i).lineID));
//        std::cout << "direction vec: " << modelsPnL.at(i).direction().norm() << std::endl;
    }

    std::cout << "cameras origin: " << cameras.at(camNum).origin() << std::endl;
    std::cout << "orientation: " << cameras.at(camNum).orientation() << std::endl << std::endl;
//    std::cout << "orientation: " << rodrigues(cameras.at(camNum).orientation()) << std::endl;
//    std::cout << "orientation T: " << rodrigues(cameras.at(camNum).orientation()).transpose() << std::endl;

/*
    double rot_cw_data[9], pos_cw_data[3];
    double minimalReprojectionError = PnLpose(segmentsPnL, modelsPnL, cv::Mat(cv::Matx33d(cameras.at(camNum).camM().data())), rot_cw_data, pos_cw_data, false);

    cv::Mat_<FT> rot_cw_data_left(3,3), rodriguesRot(3,1);
    lsfm::Pose<FT> poseLeftPnL;
    poseLeftPnL.origin(lsfm::Vec3<FT>(&pos_cw_data[0]));
    for(int i = 0; i < 9; i++)
        rot_cw_data_left.at<FT>(i) = rot_cw_data[i];
    cv::Rodrigues(rot_cw_data_left, rodriguesRot);
    poseLeftPnL.orientation(lsfm::Vec3<FT>(rodriguesRot(0,0), rodriguesRot(1,0), rodriguesRot(2,0)));

    std::cout << "reproj error: " << minimalReprojectionError << std::endl;
    std::cout << "poseLeftPnL.origin: " << poseLeftPnL.origin() << std::endl;
    std::cout << "poseLeftPnL.orientation: " << poseLeftPnL.orientation() << std::endl;
    std::cout << "rotMat: " << rodrigues(poseLeftPnL.orientation()) << std::endl;
*/
    lsfm::Pose<double> newPosePnL;
    PnLpose(segmentsPnL, modelsPnL, cameras.at(camNum).camM(), newPosePnL, false);
    std::cout << "2 poseL: " << newPosePnL.pose().origin() << std::endl;
    std::cout << "2 rotL: " <<  newPosePnL.pose().orientation()  << std::endl << std::endl;
//    std::cout << "2 rotL: " <<  rodrigues(newPosePnL.pose().orientation())  << std::endl << std::endl;


    // R and T
    /*
    double positionRes[3], rotRes[9];
    cv::Mat_<FT> initRot_RnT(3,3), rodriguesRnT(3,1), initPosition(3,1);
    cv::Mat_<FT> resRot_RnT(3,3), resPosition(3,1);
    rodriguesRnT(0) = -cameras.at(camNum).orientation()[0];
    rodriguesRnT(1) = -cameras.at(camNum).orientation()[1];
    rodriguesRnT(2) = -cameras.at(camNum).orientation()[2];
    initPosition(0) = cameras.at(camNum).origin()[0];
    initPosition(1) = cameras.at(camNum).origin()[1];
    initPosition(2) = cameras.at(camNum).origin()[2];
    cv::Rodrigues(rodriguesRnT, initRot_RnT);
    R_and_Tpose(segmentsPnL, modelsPnL, cv::Mat(cv::Matx33d(cameras.at(camNum).camM().data())), initRot_RnT, initPosition, rotRes, positionRes);

    for(int i = 0; i < 9; i++)
        resRot_RnT.at<FT>(i) = rotRes[i];
    resPosition.at<FT>(0) = positionRes[0]; resPosition.at<double>(1) = positionRes[1]; resPosition.at<double>(2) = positionRes[2];

    std::cout << "R_and_Tpose1 pose: " << resPosition << std::endl;
    std::cout << "R_and_Tpose1 rot: " << resRot_RnT << std::endl;
    */
    lsfm::Pose<double> newPoseRandT;
    R_and_Tpose(segmentsPnL, modelsPnL, cameras.at(camNum).camM(), cameras.at(camNum), newPoseRandT);
    std::cout << "R_and_Tpose2 poseL: " << newPoseRandT.pose().origin() << std::endl;
    std::cout << "R_and_Tpose2 rotL: " <<  newPoseRandT.pose().orientation()  << std::endl << std::endl;
//    std::cout << "R_and_Tpose2 rotL: " << lsfm::rodrigues(newPose.pose().orientation()) << std::endl << std::endl;


//    camMode<LineSegment<FT>>(lineSeg3Optimized,cameras.at(camNum),"optimized lineSegs, cam " + camNumSS.str());
    imshow("projected lines 1 pluecker cam1", drawGeometry(segmentsPnL,img));
    camMode<Line<FT>>(modelsPnL,cameras.at(camNum),"optimized lines, cam " + camNumSS.str());

    camMode<LineSegment<FT>>(lineSeg3Optimized,cameras.at(camNum),"optimized lineSegs, cam " + camNumSS.str());
//    camMode<Line<FT>>(line3Optimized,cameras.at(camNum),"optimized lines, cam " + camNumSS.str());

    std::vector<Line3<FT>> lines3model;
    MySlam::ResidualVector res = slam.getResdiuals();
    for (int i = 0; i != res.size(); ++i) {
        if (res[i].frame != camNum)
            continue;
        lines3model.push_back(line3Optimized[slam.getDetectedLines()[res[i].segment].modelIndex]);
        error += slam.getLineSegments()[res[i].segment].error(cameras[camNum].project(lines3model.back()));
    }

	std::cout << "error via reprojection, cam " << camNum << ": " << error << std::endl;
    camMode<Line<FT>>(lines3model,cameras[camNum], "reprojection lines, cam " + camNumSS.str());


    cv::waitKey();
    return 0;

}
