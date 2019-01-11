/*
 * OGL01Shape3D.cpp: 3D Shapes
 */
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
//#include <boost/filesystem/fstream.hpp>



#define WIDTH_ 960
#define HEIGHT_ 540
#define Y_ANGLE_DEG 45.0f

GLfloat xPos = 0.0f;
GLfloat yPos = 0.0f;
GLfloat zPos = 0.0f;

//typedef GLfloat FT;

lsfm::Pose<FT> cam0Pose;
FT xAngle = 0.0;
FT yAngle = 0.0;

int window;

lsfm::Object3DList<FT> objects;

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

void display() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);  // Clear color, depth and accumulation buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    GLfloat white[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, white);
    lsfm::renderObjects3d(objects);

    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)


    // calculate and plot 3d Lines
    lsfm::Pose<FT> tmpPose(cam0Pose);
    tmpPose.origin(lsfm::Vec3<FT>(-tmpPose.origin().x(), -tmpPose.origin().y(), -tmpPose.origin().z()));
    tmpPose.orientation(lsfm::Vec3<FT>(tmpPose.orientation().x(), tmpPose.orientation().y(), tmpPose.orientation().z()));
    lsfm::Matx33<FT> currentRot, rotToCv(1,0,0,0,-1,0,0,0,-1);  // rotate around y-axis
    currentRot = tmpPose.rotM();
    tmpPose.orientation(lsfm::Vec3<FT>(0,0,0));
    currentRot = currentRot * rotToCv;
    tmpPose.rotate(currentRot);

    lsfm::Camera<FT> camera0(2 * atan(tan(Y_ANGLE_DEG / 2 / 180 * M_PI ) * WIDTH_/HEIGHT_) , lsfm::Vec2<FT>(WIDTH_,HEIGHT_), tmpPose.origin(), tmpPose.orientation() );
    cv::Mat camera0View(cv::Mat::zeros(cv::Size(WIDTH_,HEIGHT_), CV_8UC3 ));
    lsfm::renderObjects3dCV(objects, camera0, camera0View);
    cv::imshow("camera0View", camera0View);

}

/* Handler for window re-size event. Called back when the window first appears and
   whenever the window is re-sized with its new width and height */
void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
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
   glTranslated(cam0Pose.origin().x(), cam0Pose.origin().y(), cam0Pose.origin().z());
}

// Keyboard callback routine
void keyboard(unsigned char key, int x, int y){
    if(key == 'd'){
        lsfm::Pose<FT> tmpPose;
        tmpPose.translate(lsfm::Vec3<FT>(0,0,-0.3));
        tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
        cam0Pose.translate(tmpPose.origin());
    }
    if(key == 'e'){
        lsfm::Pose<FT> tmpPose;
        tmpPose.translate(lsfm::Vec3<FT>(0,0,+0.3));
        tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
        cam0Pose.translate(tmpPose.origin());
    }
    if(key == 's'){
        lsfm::Pose<FT> tmpPose;
        tmpPose.translate(lsfm::Vec3<FT>(0.3,0,0));
        tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
        cam0Pose.translate(tmpPose.origin());
    }
    if(key == 'f'){
        lsfm::Pose<FT> tmpPose;
        tmpPose.translate(lsfm::Vec3<FT>(-0.3,0,0));
        tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
        cam0Pose.translate(tmpPose.origin());
    }
    if(key == 'w'){
        lsfm::Pose<FT> tmpPose;
        tmpPose.translate(lsfm::Vec3<FT>(0,-0.3,0));
        tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
        cam0Pose.translate(tmpPose.origin());
    }
    if(key == 'r'){
        lsfm::Pose<FT> tmpPose;
        tmpPose.translate(lsfm::Vec3<FT>(0,+0.3,0));
        tmpPose.rotate(cam0Pose.rotM(), lsfm::Vec3<FT>(0,0,0));
        cam0Pose.translate(tmpPose.origin());
    }
    if(key == 'i'){
        xAngle -= 1.0;
    }
    if(key == 'k'){
        xAngle += 1.0;
    }
    if(key == 'j'){
        yAngle += 1.0;
    }
    if(key == 'l'){
        yAngle -= 1.0;
    }
    if(key == 'q'){
        exit(0);
    }
//    glutPostRedisplay();


    // read gl image into opencv
    cv::Mat cam = cv::Mat::zeros(HEIGHT_, WIDTH_, CV_8UC3);

    glutSetWindow(window);
    reshape(WIDTH_, HEIGHT_);
    display();
    lsfm::getGlImageAsOpenCvMat(WIDTH_, HEIGHT_, cam);
    cv::Mat depth = lsfm::getGlBufferAsOpenCvMat<float>(WIDTH_, HEIGHT_,GL_DEPTH_COMPONENT);

    GLfloat range[2];
    GLint res;
    glGetFloatv(GL_DEPTH_RANGE,range);
    glGetIntegerv(GL_DEPTH_BITS, &res);
    std::cout << "znear: " << range[0] << ", zfar: " << range[1] << ", depth bits: " << res << std::endl;

    double dmin, dmax;
    cv::minMaxIdx(depth,&dmin,&dmax);
    std::cout << "dmin: " << dmin << ", dmax: " << dmax << std::endl;
    double f = 1000.0, n = 0.1;
    double a = (-f-n)/(f-n);
    double b = -2*n*f / (f-n);
    double zmin = b / (dmin + a);
    double zmax = b / (dmax + a);
    std::cout << "a: " << a << ", b: " << b << std::endl;
    std::cout << "zmin: " << zmin << ", zmax: " << zmax << std::endl;


    GLdouble projection[16];
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    a = projection[10];
    b = projection[14];
    zmin = b / (dmin + a);
    zmax = b / (dmax + a);
    std::cout << "a: " << a << ", b: " << b << std::endl;
    std::cout << "zmin: " << zmin << ", zmax: " << zmax << std::endl;

    depth -= dmin;
    depth /= dmax - dmin;

    cv::imshow("gl", cam);
    cv::imshow("depth", depth);
    cv::waitKey(1);
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {

    std::cout << CV_8UC4 << std::endl;
    std::string f = "../../data/cubes.obj";
    lsfm::loadObjects(f,objects);

    glutInit(&argc, argv);            // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_ACCUM | GLUT_DEPTH ); // GLUT_MULTISAMPLE // Enable double buffered mode
    glutInitWindowSize(WIDTH_, HEIGHT_);   // Set the window's initial width & height
    //   glutInitWindowPosition(50, 50); // Position the window's initial top-left corner

    window = glutCreateWindow("Display");        // Create window with the given title

    glutReshapeFunc(reshape);       // Register callback handler for window re-size event
    glutDisplayFunc(display);       // Register callback handler for window re-paint event
    glutKeyboardFunc(keyboard);
    initGL();                       // Our own OpenGL initialization

    glutMainLoop();                 // Enter the infinite event-processing loop

    return 0;
}
