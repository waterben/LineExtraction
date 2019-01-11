/*
 * OGL01Shape3D.cpp: 3D Shapes
 */

#define USE_CERES_JET

#include <geometry/render_gl.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry/pose.hpp>

#include <geometry/object3d.hpp>
#include <opengl_sim/lineProcessing.hpp>
 //#include <boost/filesystem/fstream.hpp>
/*


/* Global variables */
//char title[] = "3D Shapes";


#ifdef WIN32
#include <windows.h>
//#define near zNear
//#define far zFar
#endif

#include <iostream>
#include <string.h>

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
#include <slam/slamDataModel.hpp>


#define WIDTH_ 960
#define HEIGHT_ 540
#define Y_ANGLE_DEG 45.0f
#define NUM_CAMS 2
#define DISPARITY 0.3

GLdouble xPos = 0.0f;
GLdouble yPos = 0.0f;
GLdouble zPos = 0.0f;

typedef GLdouble FT;
typedef lsfm::LineSegment<FT> MyLine2;
typedef lsfm::Line3<FT> MyLine3D;
typedef lsfm::SlamModel<FT, NUM_CAMS, MyLine2, MyLine3D> MySlam;
typedef std::map<int, int> MapType;
MySlam * slam;
lsfm::Camera<FT> cameras[NUM_CAMS];
lsfm::Pose<FT> currentPoseEstimation, optimizationOffset;
lsfm::Pose<FT> cam0Pose, cam1Pose;
FT xAngle = 0.0;
FT yAngle = 0.0;
MapType gtToModelId;
int window_0, window_1;
lsfm::Object3DList<FT> objects;
int frameViewId = -1;
bool mouseControl = false, firstMouseMove = true;
int keyboardKey = 0;
lsfm::Vec2<int> lastMousePos;


#ifndef WIN32
// Weird Workaround for crash at startup when using glut and string or stream, etc.
// from: http://stackoverflow.com/questions/31579243/segmentation-fault-before-main-when-using-glut-and-stdstring
#include <pthread.h>
void* simpleFunc(void*) { return NULL; }
void forcePThreadLink() { pthread_t t1; pthread_create(&t1, NULL, &simpleFunc, NULL); }
#endif

/* Initialize OpenGL Graphics */
void initGL() {

   glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
   glClearDepth(1.0f);                   // Set background depth to farthest
   glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
   glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
   glShadeModel(GL_SMOOTH);   // Enable smooth shading
   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

   // Enable lighting

   GLfloat	ambientProperties0[] = {0.0f, 0.5f, 1.0f, 1.0f};
   GLfloat	diffuseProperties0[] = {0.0f, 0.5f, 1.0f, 1.0f};
   GLfloat	specularProperties0[] = {0.0f, 0.5f, 1.0f, 1.0f};
   GLfloat lightpos0[] = {100., 200., 300., 0.};
   glLightfv( GL_LIGHT0, GL_AMBIENT, ambientProperties0);
   glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuseProperties0);
   glLightfv( GL_LIGHT0, GL_SPECULAR, specularProperties0);
   glLightfv(GL_LIGHT0, GL_POSITION, lightpos0);

   GLfloat	ambientProperties1[] = {1.0f, 0.5f, 0.0f, 1.0f};
   GLfloat	diffuseProperties1[] = {1.0f, 0.5f, 0.0f, 1.0f};
   GLfloat	specularProperties1[] = {1.0f, 0.5f, 0.0f, 1.0f};
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

void displayCVwindows(){

    cv::Mat cam0img = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3);
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam0img);
    lsfm::Pose<FT> cam0PoseCV = lsfm::gl2cvPose(cam0Pose) ;
    lsfm::Camera<FT> cam0gt(cameras[0]);
    cam0gt.pose(cam0PoseCV);
    lsfm::CameraCV<FT>camProj0(cam0gt);
    cv::Mat modelImage = cam0img.clone();
    std::vector<lsfm::LineSegment<FT>> models2d0;
    camProj0.project(slam->getModelLineSegments(),models2d0);

    /*
    std::vector<std::string> modelIds;
    for(int i = 0; i < models2d0.size(); ++i){

      typename MapType::iterator it;
      for ( it = gtToModelId.begin(); it != gtToModelId.end(); ++it )
        if (it->second == i)
          break;

      if(it != gtToModelId.end()){
          //detectedLines0[i].modelIndex = it->second;
          modelIds.push_back(std::to_string(it->first));
      } else {
          modelIds.push_back("-1");
      }
    }
    modelImage = lsfm::drawLines<FT>(cam0img, models2d0, modelIds);
    */

    modelImage = lsfm::drawLines<FT>(cam0img, models2d0);
    cv::imshow("cam0", modelImage);

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
        std::vector<int> optiFrames;
        for(int i = 0; i < slam->getFrameNum(); ++i){
            optiFrames.push_back(i);
        }
        std::vector<int> fixedCycle = {0};
        slam->bundleAdjustmentOnFramesFixedPoses(optiFrames, fixedCycle);
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
            xAngle -= 1.0;
        } else if(keyboardKey == 'k'){
            xAngle += 1.0;
        } else if(keyboardKey == 'j'){
            yAngle += 1.0;
        } else if(keyboardKey == 'l'){
            yAngle -= 1.0;
        }

    }
    reshape0(WIDTH_, HEIGHT_);
    glutPostRedisplay();
    cv::waitKey(10);
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {

#ifdef WIN32
    std::string f = "../../../meshes/teapot.obj";
#else
    std::string f = "../../../Datasets/Blender/QubesPyramid.obj";
#endif
    lsfm::loadObjects(f,objects);

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

    initGL();                       // Our own OpenGL initialization
    cv::waitKey(25);                // run once to preopen OpenCV windows


    glutMainLoop();                 // Enter the infinite event-processing loop

    return 0;
}
