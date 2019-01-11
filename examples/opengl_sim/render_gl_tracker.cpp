/*
 * OGL01Shape3D.cpp: 3D Shapes
 */

#define DEBUG_SWITCH_LINETRACKING

#define USE_CERES_JET
//#define DEBUG_SWITCH

#include <iostream>
#include <string.h>

#include <geometry/render_gl.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry/pose.hpp>

#include <geometry/object3d.hpp>
#include <opengl_sim/lineProcessing.hpp>
 //#include <boost/filesystem/fstream.hpp>
#include <lsd/lsd_el.hpp>
#include <geometry/draw.hpp>
#include <line_tracker/linetracker.hpp>


/* Global variables */
//char title[] = "3D Shapes";


#ifdef WIN32
#include <windows.h>
//#define near zNear
//#define far zFar
#endif

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
//#include <opengl_sim/jitter.h>
#ifdef WIN32
#include <GL/glut.h>    // required for glutSetOption
#else
#include <GL/freeglut.h>    // required for glutSetOption
#endif
#include <opencv2/highgui/highgui.hpp>

#include <geometry/pose.hpp>

#include <geometry/object3d.hpp>
#include <opengl_sim/lineProcessing.hpp>
#include <opengl_sim/lineMatching.hpp>
#include <opengl_sim/TextureLoader.hpp>
#include <slam/slamDataModel.hpp>

#include <line_tracker/image_alignment.hpp>

using namespace std;


#define WIDTH_ 960
#define HEIGHT_ 540
#define Y_ANGLE_DEG 45.0f
#define NUM_CAMS 2
#define DISPARITY 0.3

GLdouble xPos = 0.0f;
GLdouble yPos = 0.0f;
GLdouble zPos = 0.0f;

typedef GLdouble FT;
//typedef GLfloat FT;
typedef lsfm::LineSegment<FT> MyLine2;
typedef lsfm::Line3<FT> MyLine3D;
typedef lsfm::SlamModel<FT, NUM_CAMS, MyLine2, MyLine3D> MySlam;
typedef std::map<int, int> MapType;

//typedef cv::Point_<FT> LPT;
//typedef cv::Point_<FT> PT;
typedef Vec2f LPT;
//typedef Vec2f PT;

MySlam * slam;
lsfm::Camera<FT> cameras[NUM_CAMS];
lsfm::Pose<FT> currentPoseEstimation, optimizationOffset;
lsfm::Pose<FT> cam0Pose, cam1Pose;

//line tracking stuff
lsfm::LineTracker<float, cv::Point2> myTracker;     //cv::Point2 is needed for calculation of the optical flow.
lsfm::MultiScaleImageAlignment<FT> ia(4);
bool doInit = true;
lsfm::Pose<FT> lastPose;

FT xAngle = 0.0;
FT yAngle = 0.0;
MapType gtToModelId;
int window_0, window_1;
lsfm::Object3DList<FT> objects;
int frameViewId = -1;
bool mouseControl = false, firstMouseMove = true;
int keyboardKey = 0;
lsfm::Vec2<int> lastMousePos;
cv::Mat previousImg0;
cv::Mat previousImg1;
std::vector<lsfm::LineSegment3<FT>> lineModels3d;
std::vector<std::string> lineModel3dIDs;
lsfm::Pose<FT> modelPoseDiff;

GLuint texture[1];
struct Image {
    unsigned long sizeX;
    unsigned long sizeY;
    char *data;
};
typedef struct Image Image;

#ifndef WIN32
// Weird Workaround for crash at startup when using glut and string or stream, etc.
// from: http://stackoverflow.com/questions/31579243/segmentation-fault-before-main-when-using-glut-and-stdstring
#include <pthread.h>
void* simpleFunc(void*) { return NULL; }
void forcePThreadLink() { pthread_t t1; pthread_create(&t1, NULL, &simpleFunc, NULL); }
#endif



// quick and dirty bitmap loader...for 24 bit bitmaps with 1 plane only.
// See http://www.dcs.ed.ac.uk/~mxr/gfx/2d/BMP.txt for more info.
int ImageLoad(char *filename, Image *image) {
    FILE *file;
    unsigned long size;                 // size of the image in bytes.
    unsigned long i;                    // standard counter.
    unsigned short int planes;          // number of planes in image (must be 1)
    unsigned short int bpp;             // number of bits per pixel (must be 24)
    char temp;                          // temporary color storage for bgr-rgb conversion.

    // make sure the file is there.
    if ((file = fopen(filename, "rb"))==NULL)
    {
    printf("File Not Found : %s\n",filename);
    return 0;
    }

    // seek through the bmp header, up to the width/height:
    fseek(file, 18, SEEK_CUR);

    // read the width
    if ((i = fread(&image->sizeX, 4, 1, file)) != 1) {
    printf("Error reading width from %s.\n", filename);
    return 0;
    }
    printf("Width of %s: %lu\n", filename, image->sizeX);

    // read the height
    if ((i = fread(&image->sizeY, 4, 1, file)) != 1) {
    printf("Error reading height from %s.\n", filename);
    return 0;
    }
    printf("Height of %s: %lu\n", filename, image->sizeY);

    // calculate the size (assuming 24 bits or 3 bytes per pixel).
    size = image->sizeX * image->sizeY * 3;

    // read the planes
    if ((fread(&planes, 2, 1, file)) != 1) {
    printf("Error reading planes from %s.\n", filename);
    return 0;
    }
    if (planes != 1) {
    printf("Planes from %s is not 1: %u\n", filename, planes);
    return 0;
    }

    // read the bpp
    if ((i = fread(&bpp, 2, 1, file)) != 1) {
    printf("Error reading bpp from %s.\n", filename);
    return 0;
    }
    if (bpp != 24) {
    printf("Bpp from %s is not 24: %u\n", filename, bpp);
    return 0;
    }

    // seek past the rest of the bitmap header.
    fseek(file, 24, SEEK_CUR);

    // read the data.
    image->data = (char *) malloc(size);
    if (image->data == NULL) {
    printf("Error allocating memory for color-corrected image data");
    return 0;
    }

    if ((i = fread(image->data, size, 1, file)) != 1) {
    printf("Error reading image data from %s.\n", filename);
    return 0;
    }
/*
    for (i=0;i<size;i+=3) { // reverse all of the colors. (bgr -> rgb)
    temp = image->data[i];
    image->data[i] = image->data[i+2];
    image->data[i+2] = temp;
    }
*/

    return 1;
}


// Load Bitmaps And Convert To Textures
void LoadGLTextures(std::string textureFile) {
    // Load Texture
    Image *image1;

    // allocate space for texture
    //image1 = (Image *) malloc(sizeof(Image));
    image1 = new Image();
    if (image1 == NULL) {
    printf("Error allocating space for image");
    exit(0);
    }

    char * f = strdup(textureFile.c_str());
    if (!ImageLoad(f, image1)) {
    exit(1);
    }

    // Create Texture
    glGenTextures(1, &texture[0]);
    glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)

    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly when image smalled than texture

    // 2d texture, level of detail 0 (normal), 3 components (red, green, blue), x size from image, y size from image,
    // border 0 (normal), rgb color data, unsigned byte data, and finally the data itself.
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);
}

/* Initialize OpenGL Graphics */
void initGL() {

   glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping
   glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
   glClearDepth(1.0f);                   // Set background depth to farthest
   glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
   glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
   glShadeModel(GL_SMOOTH);   // Enable smooth shading
   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

   // Enable lighting

   GLfloat	ambientProperties0[] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat	diffuseProperties0[] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat	specularProperties0[] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat lightpos0[] = {100., 200., 300., 0.};
   glLightfv( GL_LIGHT0, GL_AMBIENT, ambientProperties0);
   glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuseProperties0);
   glLightfv( GL_LIGHT0, GL_SPECULAR, specularProperties0);
   glLightfv(GL_LIGHT0, GL_POSITION, lightpos0);

   GLfloat	ambientProperties1[] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat	diffuseProperties1[] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat	specularProperties1[] = {1.0f, 1.0f, 1.0f, 1.0f};
   GLfloat lightpos1[] = {-100., -200., -300., 0.};
   glLightfv( GL_LIGHT1, GL_AMBIENT, ambientProperties1);
   glLightfv( GL_LIGHT1, GL_DIFFUSE, diffuseProperties1);
   glLightfv( GL_LIGHT1, GL_SPECULAR, specularProperties1);
   glLightfv(GL_LIGHT1, GL_POSITION, lightpos1);

   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_LIGHT1);

   glDisable(GL_LINE_SMOOTH);
   glDisable(GL_LINE_SMOOTH_HINT);
}

void initSlam(){

    // TODO: Only works if camera starts at 0,0,0 ?

    lsfm::Pose<FT> tmpPose;// = lsfm::gl2cvPose(cam0Pose);

    lsfm::Camera<FT> c0(2 * atan(tan(Y_ANGLE_DEG / 2 / 180 * M_PI ) * WIDTH_/HEIGHT_) , lsfm::Vec2<FT>(WIDTH_,HEIGHT_), tmpPose.origin(), tmpPose.orientation() );
    cameras[0] = c0;

    currentPoseEstimation = cameras[0];

    lsfm::Pose<FT> tmpPose2;
    tmpPose2.translate(lsfm::Vec3<FT>(+DISPARITY,0,0));     // careful here if anything is changed
    tmpPose2.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
    cam0Pose.translate(tmpPose2.origin());

    lsfm::Pose<FT> tmpPose3 = cam0Pose ;

    lsfm::Camera<FT> c1(2 * atan(tan(Y_ANGLE_DEG / 2 / 180 * M_PI ) * WIDTH_/HEIGHT_) , lsfm::Vec2<FT>(WIDTH_,HEIGHT_), tmpPose3.origin(), tmpPose3.orientation() );
    cameras[1] = c1;

    cam0Pose.origin(lsfm::Vec3<FT>(0,0,0));

    slam = new MySlam(cameras);

}

void filterLineModels3d(){

    std::vector<lsfm::LineSegment3<FT>> modelLineSegments = slam->getModelLineSegments();
    lineModel3dIDs.clear();
    lineModels3d.clear();
    // Filter out lines which have less than 5 observations or have a high variance
    const FT varianceCap = 1000.0;

//    std::vector<std::string> modelIds;

    for (int i = 0; i < modelLineSegments.size(); ++i){
        FT var, maxE;
        std::vector<std::pair<int,FT>> obsErr;
        slam->reprojectionErrorModel(i, var, maxE, obsErr);

        if(obsErr.size() > 4 && var < varianceCap){
            lineModel3dIDs.push_back(std::to_string(i));
            lineModels3d.push_back(modelLineSegments[i]);
        }
    }

}
void displayCVwindows(){
    extern bool doInit;
    extern lsfm::Pose<FT> lastPose;

    cv::Mat cam0img = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3);
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam0img);
    lsfm::Pose<FT> cam0PoseCV = lsfm::gl2cvPose(cam0Pose) ;
    lsfm::Camera<FT> cam0gt(cameras[0]);
    cam0gt.pose(cam0PoseCV);
    lsfm::CameraCV<FT>camProj0(cam0gt);
    //cv::Mat modelImage = cam0img.clone();
    cv::Mat dirMap, trackerImg, lastImg;
    cv::Mat blurImg, trackerImgColor, lastImgColor;
    lastImg = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3);
    std::vector<lsfm::LineSegment<FT>> models2d0;

    camProj0.project(lineModels3d,models2d0);

    cv::cvtColor(cam0img, trackerImg, CV_BGR2GRAY);
    //cv::GaussianBlur(trackerImg, blurImg, cv::Size(11,11), 0, 0);

    if(doInit) {
        myTracker.init(trackerImg);
        ia.createNextPyramid(trackerImg);
        doInit = false;
    }
    else {
        if(cam0PoseCV.origin() != lastPose.origin() || cam0PoseCV.orientation() != lastPose.orientation()) {
            myTracker.track(trackerImg);
            lastPose.origin(cam0PoseCV.origin());
            lastPose.orientation(cam0PoseCV.orientation());
            //ia.createNextPyramid(trackerImg);
            cv::Point2f imgShift;
            //ia.getShift(imgShift);
            //std::cout<<"Shift: "<<imgShift.x<<" "<<imgShift.y<<std::endl;
        }
    }

    lastImg = myTracker.getLastImg();


    cv::cvtColor(trackerImg, trackerImgColor, CV_GRAY2BGR);
    cv::cvtColor(lastImg, lastImgColor, CV_GRAY2BGR);

    //modelImage = lsfm::drawLines<FT>(cam0img, models2d0, lineModel3dIDs);
    myTracker.drawCurrLines(trackerImgColor);
//#ifdef DEBUG_SWITCH_LINETRACKING
    myTracker.drawValidPredPoints(trackerImgColor);

//#endif

    myTracker.drawDirMap(lastImgColor);
    myTracker.drawPrevLines(lastImgColor);
    //ia.drawLastShift(lastImgColor, cv::Scalar(255,255,0), 1, 1);
//#ifdef DEBUG_SWITCH_LINETRACKING
    //myTracker.drawOpticalFlow(lastImgColor);
//#endif

    //cv::imshow("cam0", modelImage);
    cv::imshow("current frame", trackerImgColor);
    cv::imshow("previous frame", lastImgColor);


    //std::cout << "preCPose: " << camProj0.origin() << std::endl << camProj0.orientation() << std::endl;

    //cv::Mat modelImageCorrected = cam0img.clone();

    lsfm::Pose<FT> cam0PoseCVc = lsfm::gl2cvPose(cam0Pose) ;
    lsfm::Vec3<FT> tmpVec = cam0PoseCVc.orientation() + modelPoseDiff.orientation();
    cam0PoseCVc.orientation(tmpVec);
    cam0PoseCVc.origin(cam0PoseCVc.origin() + modelPoseDiff.origin());

    lsfm::Camera<FT> cam0gtC(cameras[0]);
    cam0gtC.pose(cam0PoseCVc);
    lsfm::CameraCV<FT>camProj0C(cam0gtC);

    std::vector<lsfm::LineSegment<FT>> models2d0C;
    camProj0C.project(lineModels3d,models2d0C);
    //modelImageCorrected = lsfm::drawLines<FT>(cam0img, models2d0C, lineModel3dIDs);
    //cv::imshow("cam0corrected", modelImageCorrected);

    //std::cout << "CorrPose: " << camProj0C.origin() << std::endl << camProj0C.orientation() << std::endl;

/*
    // calculate and plot 3d GT Lines
    lsfm::Camera<FT> camera2(cameras[0]);
    camera2.pose(lsfm::gl2cvPose(cam0Pose));

    cv::Mat camera0View(cv::Mat::zeros(cv::Size(WIDTH_,HEIGHT_), CV_8UC3 ));
    lsfm::renderObjects3dCV(objects, camera2, camera0View);
    cv::imshow("camera0View", camera0View);
*/
    cv::waitKey(1);
}

/* Handler for window-repaint event. Called back when the window first appears and
   whenever the window needs to be re-painted. */
void display0() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);  // Clear color, depth and accumulation buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    GLfloat white[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, white);
    std::vector<std::pair<lsfm::Vec3<FT>, lsfm::Vec3<FT>>> endPoints;
    lsfm::renderWireIdObjects3d(objects, endPoints);

    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}
void display1() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);  // Clear color, depth and accumulation buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    GLfloat white[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, white);
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    lsfm::renderObjects3d(objects);

    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)

    displayCVwindows();
}

/* Handler for window re-size event. Called back when the window first appears and
   whenever the window is re-sized with its new width and height */
void reshape0(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
   // Compute aspect ratio of the new window
   if (height == 0) height = 1;                // To prevent division by 0
   GLfloat aspect = (GLfloat)width / (GLfloat)height;

   // Set the viewport to cover the new window
   glViewport(0, 0, width, height);

   // Set the aspect ratio of the clipping volume to match the viewport
   glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
   glLoadIdentity();             // Reset
   // Enable perspective projection with fovy, aspect, zNear and zFar
   gluPerspective(Y_ANGLE_DEG, aspect, 0.1f, 1000.0f);

   lsfm::Pose<FT> tmpPose;
   tmpPose.rotate(lsfm::Vec3<FT>(xAngle*M_PI/180.0f,0,0));
   lsfm::Matx44<FT> m1 = tmpPose.homM();

   tmpPose.orientation(lsfm::Vec3<FT>(0,0,0));
   tmpPose.rotate(lsfm::Vec3<FT>(0,yAngle*M_PI/180.0f,0));
   lsfm::Matx44<FT> m2 = tmpPose.homM();
   lsfm::Matx44<FT> m3 = m2 * m1;   // multiply in correct order!

   lsfm::Matx33<FT> rotMat;
   for(int i = 0; i < 3; ++i)
       for(int j = 0; j < 3; ++j)
           rotMat(i,j) = m3(i,j);
   cam0Pose.orientation(lsfm::rodrigues(rotMat));

   glMultMatrixd( m3.data());
   glTranslatef(cam0Pose.origin().x(), cam0Pose.origin().y(), cam0Pose.origin().z());
}

void keyframe(){

    // read gl image into opencv
    cv::Mat cam0img = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3), cam0GtImg = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3), cam1img = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3), cam1GtImg = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3);

    // move right for stereo:
    lsfm::Pose<FT> tmpPose;
    tmpPose.translate(lsfm::Vec3<FT>(-DISPARITY,0,0));
    tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
    cam0Pose.translate(tmpPose.origin());

    // wire image for IDs
    glutSetWindow(window_0);
    reshape0(WIDTH_, HEIGHT_);
    display0();
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam1GtImg);

    // scene image
    glutSetWindow(window_1);
    reshape0(WIDTH_, HEIGHT_);
    display1();
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam1img);

    cam1Pose = cam0Pose;

    // move back
//    lsfm::Pose<FT> tmpPose2;
//    tmpPose2.translate(lsfm::Vec3<FT>(DISPARITY,0,0));
//    tmpPose2.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
    cam0Pose.translate(-tmpPose.origin());

    // wire image for IDs
    glutSetWindow(window_0);
    reshape0(WIDTH_, HEIGHT_);
    display0();
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam0GtImg);

    // scene image
    glutSetWindow(window_1);
    reshape0(WIDTH_, HEIGHT_);
    display1();
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam0img);

    lsfm::Camera<FT> cam0gt(cameras[0]), cam1gt(cameras[1]);
    lsfm::Pose<FT> cam0PoseCV = lsfm::gl2cvPose(cam0Pose), cam1PoseCV = lsfm::gl2cvPose(cam1Pose);
    cam0gt.pose(cam0PoseCV);
    cam1gt.pose(cam1PoseCV);

    lsfm::lineProcessingGt<FT, MySlam, MapType>(cam0img, cam1img, cam0GtImg, cam1GtImg, gtToModelId, slam, currentPoseEstimation, cam0gt, cam1gt);

//    lsfm::lineMatchingCV(cam0img, cam1img);
//    lsfm::lineMatchingLsdAndCV(cam0img, cam1img);

    if(!previousImg0.empty()){
//        lsfm::lineMatchingLsdAndCV(previousImg0, cam1img);
    }

    previousImg0 = cam0img;
    previousImg1 = cam1img;
}

void keyboardUp(unsigned char key, int x, int y){
    keyboardKey = 0;
}

// Keyboard callback routine
void keyboard(unsigned char key, int x, int y){

    if(key == 'q'){
        exit(0);
    } else if(key == 'n'){                                          // display next frame
        ++frameViewId;
        cv::Mat frameImg;
        slam->showFrameLineData(frameViewId, frameImg);
        cv::imshow("frameView", frameImg);
        std::cout << "frameViewId " << frameViewId << std::endl;
    } else if(key == 'b'){                                          // display previous frame (before)
        --frameViewId;
        cv::Mat frameImg;
        slam->showFrameLineData(frameViewId, frameImg);
        cv::imshow("frameView", frameImg);
        std::cout << "frameViewId " << frameViewId << std::endl;
    }else if(key == 'z'){                                           // Bundle Adjustment
        lsfm::Pose<FT> tmpPose0, tmpPose1;
        if(slam->getRobotPoses().size() > 0)
            tmpPose0 = slam->getRobotPoses().back();
        std::vector<int> optiFrames;
        for(int i = 0; i < slam->getFrameNum(); ++i){
            optiFrames.push_back(i);
        }
        //std::vector<int> fixedCycle = {0};
        //slam->bundleAdjustmentOnFramesFixed(optiFrames, fixedCycle);
        filterLineModels3d();


        std::cout << "prePose: " << tmpPose0.origin() << std::endl << tmpPose0.orientation() << std::endl;
        tmpPose1 = slam->getRobotPoses().back();
        std::cout << "postPose: " << tmpPose1.origin() << std::endl << tmpPose1.orientation() << std::endl;

        lsfm::Vec3<FT> tmpVec = tmpPose1.orientation() - tmpPose0.orientation();
        modelPoseDiff.orientation(tmpVec);
        modelPoseDiff.origin(tmpPose1.origin() - tmpPose0.origin());

        std::cout << "diffPose: " << modelPoseDiff.origin() << std::endl << modelPoseDiff.orientation() << std::endl << std::endl;

    }else if(key == 'o'){                                           // Outliers
        int numModels = slam->getModeledLines().size();
        for(int i = 0; i < numModels; ++i){
            FT var, maxE;
            slam->reprojectionErrorModel(i, var, maxE);

            std::cout << "model: " << i << " var: " << var << " maxE: " << maxE << std::endl;

        }
    } else {
        keyboardKey = key;
    }

//    glutPostRedisplay();
    // scene image
    glutSetWindow(window_1);
    reshape0(WIDTH_, HEIGHT_);
    //display1();
    glutPostRedisplay();


    // make keyframe
    if(key == ' '){
//        delete(slam);
//        slam = new MySlam(cameras);
//        gtToModelId.clear();
        keyframe();
        filterLineModels3d();
    }
}

// MouseButton callback routine
void mouseButton(int button, int state, int x, int y){
    if(button == GLUT_LEFT_BUTTON && state == GLUT_UP){
        mouseControl = !mouseControl;
        if(mouseControl)
            glutSetCursor(GLUT_CURSOR_NONE);
        else
            glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
    }
}

// Mouse callback routine
void mouse(int x, int y){
    if(!mouseControl)
        return;
    int middleX = glutGet(GLUT_WINDOW_WIDTH) / 2;
    int middleY = glutGet(GLUT_WINDOW_HEIGHT) / 2;

    if(firstMouseMove){
        glutSetCursor(GLUT_CURSOR_NONE);
        glutWarpPointer(middleX, middleY);
        lastMousePos[0] = middleX;
        lastMousePos[1] = middleY;
        firstMouseMove = false;
        return;
    }

    float xMove = lastMousePos[0] - x;
    float yMove = lastMousePos[1] - y;
    if( std::abs(middleX - lastMousePos[0]) > 3 || std::abs(middleY - lastMousePos[1]) > 3){
        lastMousePos[0] = middleX;
        lastMousePos[1] = middleY;
        glutWarpPointer(middleX, middleY);
    } else {
        lastMousePos[0] = x;
        lastMousePos[1] = y;
    }
    xAngle += 0.1 * yMove;
    yAngle += 0.1 * xMove;
}

void idleCalc(){
    if(keyboardKey){
        if(keyboardKey == 'd'){
            lsfm::Pose<FT> tmpPose;
            tmpPose.translate(lsfm::Vec3<FT>(0,0,-0.3));
            tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
            cam0Pose.translate(tmpPose.origin());
        } else if(keyboardKey == 'e'){
            lsfm::Pose<FT> tmpPose;
            tmpPose.translate(lsfm::Vec3<FT>(0,0,+0.3));
            tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
            cam0Pose.translate(tmpPose.origin());
        } else if(keyboardKey == 's'){
            lsfm::Pose<FT> tmpPose;
            tmpPose.translate(lsfm::Vec3<FT>(0.3,0,0));
            tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
            cam0Pose.translate(tmpPose.origin());
        } else if(keyboardKey == 'f'){
            lsfm::Pose<FT> tmpPose;
            tmpPose.translate(lsfm::Vec3<FT>(-0.3,0,0));
            tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
            cam0Pose.translate(tmpPose.origin());
        } else if(keyboardKey == 'w'){
            lsfm::Pose<FT> tmpPose;
            tmpPose.translate(lsfm::Vec3<FT>(0,-0.3,0));
            tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
            cam0Pose.translate(tmpPose.origin());
        } else if(keyboardKey == 'r'){
            lsfm::Pose<FT> tmpPose;
            tmpPose.translate(lsfm::Vec3<FT>(0,+0.3,0));
            tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
            cam0Pose.translate(tmpPose.origin());
        } else if(keyboardKey == 'i'){
            xAngle -= 1.5;
        } else if(keyboardKey == 'k'){
            xAngle += 1.5;
        } else if(keyboardKey == 'j'){
            yAngle += 1.5;
        } else if(keyboardKey == 'l'){
            yAngle -= 1.5;
        } else if(keyboardKey == 'p'){
            extern bool doInit;
            doInit = true;
        }

    }
    reshape0(WIDTH_, HEIGHT_);
    glutPostRedisplay();
    cv::waitKey(10);
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {

#ifdef WIN32
    std::string objectFile = "../../../meshes/teapot.obj";
#else
    std::string objectFile = "../../../Datasets/Blender/QubesPyramidTexture.obj";
#endif
    std::string textureFile = "../../../Datasets/Blender/Textures/Textures.bmp";

    lsfm::loadObjects(objectFile,objects);


//    texture[0] = LoadTexture( "../../Datasets/Blender/textureNumbers.bmp" );

    glutInit(&argc, argv);            // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_ACCUM | GLUT_DEPTH ); // GLUT_MULTISAMPLE // Enable double buffered mode
    glutInitWindowSize(WIDTH_, HEIGHT_);   // Set the window's initial width & height
    //   glutInitWindowPosition(50, 50); // Position the window's initial top-left corner

    initSlam();

    window_0 = glutCreateWindow("No Multisampling");        // Create window with the given title

    glutReshapeFunc(reshape0);       // Register callback handler for window re-size event
    glutDisplayFunc(display0);       // Register callback handler for window re-paint event
    glutKeyboardFunc(keyboard);
    initGL();                       // Our own OpenGL initialization

    // Window 1 ----------
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_ACCUM | GLUT_DEPTH | GLUT_MULTISAMPLE); // GLUT_MULTISAMPLE // Enable double buffered mode
#ifndef WIN32
    glutSetOption(GLUT_MULTISAMPLE, 16);
#endif

    window_1 = glutCreateWindow("Do Multisampling");        // Create window with the given title

    glutReshapeFunc(reshape0);       // Register callback handler for window re-size event
    glutDisplayFunc(display1);       // Register callback handler for window re-paint event
    glutIgnoreKeyRepeat(1);          // disable auto-repeat
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutMotionFunc(mouse);
    glutPassiveMotionFunc(mouse);
    glutMouseFunc(mouseButton);
    glutIdleFunc(idleCalc);         // Idle used for keyboard and cv redisplay
    // ------------------

    LoadGLTextures(textureFile);
    initGL();                       // Our own OpenGL initialization
    cv::waitKey(25);                // run once to preopen OpenCV windows

    cv::Mat cam0img = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3);
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam0img);
//    myTracker.init(cam0img);


    glutMainLoop();                 // Enter the infinite event-processing loop

    return 0;
}
