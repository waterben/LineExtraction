#include <iostream>
#include <map>
#include <geometry/draw.hpp>
#include <geometry/cameracv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

typedef double FT;



int main(int argc, char** argv)
{
    //rodrigues(rodrigues(lsfm::Vec3<FT>(0,0,CV_PI/2)) * rodrigues(lsfm::Vec3<FT>(CV_PI,0,0)));
    // camera parameters
    lsfm::Vec3<FT> originL(0,-0.7,10);
    lsfm::Vec3<FT> originR(0,0.7,10);
    lsfm::Vec3<FT> orientation(rodrigues(lsfm::Matx33<FT>(rodrigues(lsfm::Vec3<FT>(0,0,CV_PI/2)) * rodrigues(lsfm::Vec3<FT>(CV_PI,0,0)))));
    FT fov = 49.134/180.0 * CV_PI;
    lsfm::Vec2<FT> size(1024,1024);


    // data model cube
    vector<lsfm::Vec3<FT>> vertices;
    vertices.push_back(lsfm::Vec3<FT>(0.72685, -0.49649, 4.99326));
    vertices.push_back(lsfm::Vec3<FT>(-0.36723, -0.33389, 4.68104));
    vertices.push_back(lsfm::Vec3<FT>( 0.0076,-0.83574, 5.64467));

    vertices.push_back(lsfm::Vec3<FT>(0.34568, 2.16278, -5.176));
    vertices.push_back(lsfm::Vec3<FT>(-0.13941, 3.20321, -5.12013));
    vertices.push_back(lsfm::Vec3<FT>(-0.55194, 2.47123, -4.52787));

    vertices.push_back(lsfm::Vec3<FT>(0.40223, 8.31917, -14.97695));
    vertices.push_back(lsfm::Vec3<FT>(-0.72169, 8.08349, -15.02374));
    vertices.push_back(lsfm::Vec3<FT>(-0.08276, 7.27818, -15.02236));

    vector<pair<size_t,size_t>> edges;
    edges.push_back(pair<size_t,size_t>(0,1));
    edges.push_back(pair<size_t,size_t>(1,2));
    edges.push_back(pair<size_t,size_t>(2,0));

    edges.push_back(pair<size_t,size_t>(3,4));
    edges.push_back(pair<size_t,size_t>(4,5));
    edges.push_back(pair<size_t,size_t>(5,3));

    edges.push_back(pair<size_t,size_t>(6,7));
    edges.push_back(pair<size_t,size_t>(7,8));
    edges.push_back(pair<size_t,size_t>(8,6));

    // init Cameras
    CameraCV<FT> camLCV(fov,size,originL,orientation);
    CameraCV<FT> camRCV(fov,size,originR,orientation);

    std::cout << "cam L: " << camLCV.projM() << std::endl;
    std::cout << "cam R: " << camRCV.projM() << std::endl;

    /*lsfm::Vec4<FT> q = quaterion(camLCV.rotM());
    lsfm::Vec3<FT> rvec = rodrigues(quaterion(q));
    std::cout << "rot: " << camLCV.orientation() << ", q: " << q << ", rrot: " << rvec << ", rq: " << quaterion(rodrigues(rvec)) << std::endl;

    lsfm::Vec3<FT> e = euler(camLCV.rotM());
    rvec = rodrigues(euler(e));
    std::cout << "rot: " << camLCV.orientation() << ", e: " << e << ", rrot: " << rvec << ", re: " << euler(rodrigues(rvec))  << std::endl;*/

    vector<lsfm::Vec2<FT>> resL, resR;

    camLCV.project(vertices,resL);
    camRCV.project(vertices,resR);

    double n = 1.0 / 1024;
    for_each(edges.begin(),edges.end(),[&](const pair<size_t,size_t> &data){
        std::cout << vertices[data.first] << ", " << vertices[data.second] << endl;
        std::cout << resL[data.first] * n  << ", " << resL[data.second] * n << endl;
        std::cout << resR[data.first] * n << ", " << resR[data.second] * n << endl;
    });

    const char* fleft = "../apps/ressources/stereoanalyser/images/stereo_tria_1024x1024_left.png";
    const char* fright = "../apps/ressources/stereoanalyser/images/stereo_tria_1024x1024_right.png";

    Mat imLeft = imread(fleft, 0);
    Mat imRight = imread(fright, 0);

    if (!imLeft.empty() && !imRight.empty()) {
        imshow("Left projection", drawGeometry(resL,edges,imLeft));
        imshow("Right projection", drawGeometry(resR,edges,imRight));
        waitKey();
    }


    Mat img(static_cast<int>(ceil(size.y())),static_cast<int>(ceil(size.x())),CV_8UC3);
    img.setTo(cv::Vec3<uchar>(0,0,0));

    auto updateMode = [&](int mode, lsfm::Vec3<FT> val) {
        switch(mode) {
            case 'r':
                swap(val.x(),val.y());
                val *= CV_PI / 180;
                camLCV.rotate(val);
                camRCV.rotate(val);
                std::cout << camLCV.orientation() * (180 / CV_PI) << std::endl;
            break;
            case 'o':
                swap(val.x(),val.y());
                val *= CV_PI / 180;
                camLCV.rotate(val,lsfm::Vec3<FT>(0,0,0));
                camRCV.rotate(val,lsfm::Vec3<FT>(0,0,0));
                std::cout << camLCV.orientation() * (180 / CV_PI) << std::endl;
            break;
            case 'w':
                swap(val.x(),val.y());
                val *= CV_PI / 180;
                camLCV.orientation(lsfm::Vec3<FT>(camLCV.orientation()+val));
                camRCV.orientation(lsfm::Vec3<FT>(camRCV.orientation()+val));
                std::cout << camLCV.orientation() * (180 / CV_PI) << std::endl;
            break;
            case 't':
                val *= 0.1;
                camLCV.translate(val);
                camRCV.translate(val);
                std::cout << camLCV.origin() << std::endl;
            break;
        }
    };

    int key = 0;
    int mode = 'r';
    while(key != 'q') {
        switch(key) {
            case 'r':
            mode = key;
            cout << "switched to rotation mode (rotate camera around world origin)" << endl;
            break;
            case 'o':
            mode = key;
            cout << "switched to orientation mode (rotate camera around camera origin)" << endl;
            break;
            case 'w':
            mode = key;
            cout << "switched to world mode (rotate camera around camera origin by world coords)" << endl;
            break;
            case 't':
                mode = key;
                cout << "switched to translation mode" << endl;
            break;
            case 'c':
                cout << "clear data" << endl;
                camLCV.origin(originL);
                camLCV.orientation(orientation);
                camRCV.origin(originR);
                camRCV.orientation(orientation);

            break;
            case 81: // left
                updateMode(mode,lsfm::Vec3<FT>(1,0,0));
            break;
            case 82: // up
                updateMode(mode,lsfm::Vec3<FT>(0,1,0));
            break;
            case 83: // right
                updateMode(mode,lsfm::Vec3<FT>(-1,0,0));
            break;
            case 84: // down
                updateMode(mode,lsfm::Vec3<FT>(0,-1,0));
            break;
            case 85: // bup
                updateMode(mode,lsfm::Vec3<FT>(0,0,1));
            break;
            case 86: // bdown
                updateMode(mode,lsfm::Vec3<FT>(0,0,-1));
            break;

        }

        camLCV.project(vertices,resL);
        imshow("Left projection", drawGeometry(resL,edges,img));

        camRCV.project(vertices,resR);
        imshow("Right projection", drawGeometry(resR,edges,img));

        /*camP.project(vertices,res);
        imshow("projected point own", drawGeometry(res,edges,img));


        camP.project(lineSegments,resLSP);
        imshow("projected line segments pluecker", drawGeometry(resLSP,img));

        camCV.project(lineSegments,resLSCV);
        imshow("projected line segments two points CV", drawGeometry(resLSCV,img));*/

        key = waitKey() % 256;
    }

    return 0;
}
