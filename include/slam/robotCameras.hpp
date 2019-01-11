#ifndef _ROBOT_CAMERAS_HPP_
#define _ROBOT_CAMERAS_HPP_
#ifdef __cplusplus

#include <vector>
#include <geometry/camera.hpp>


template<class FT, int nCams>
class RobotCameras {



protected:
    std::vector<lsfm::Camera<FT>> cameras;


public:
    RobotCameras(std::vector<lsfm::Camera<FT>> cams = std::vector<lsfm::Camera<FT>>()) :
        cameras(cams) { }

/*
    Matx44<FT> getCameraWorldPose(int camNr){
        cv::Camera<FT> cam = cameras.at(camNr);
        return cam.baseH();

    }
*/

    lsfm::Camera<FT> & getCamera(int i){
        return cameras[i];
    }



};



#endif
#endif
