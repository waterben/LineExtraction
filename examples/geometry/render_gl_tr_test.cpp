/// @file render_gl_tr_test.cpp
/// @brief OpenGL-based geometry rendering test.
///
/// Demonstrates OpenGL rendering of 3D geometry:
/// - GLUT window initialization
/// - 3D shape rendering with transformations
/// - Camera view setup
///
/// @usage ./render_gl_tr_test
/// @note Requires OpenGL and GLUT libraries.

/*
 * OGL01Shape3D.cpp: 3D Shapes
 */
#include <geometry/render_gl.hpp>
#ifdef WIN32
#  include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glut.h>


int main(int argc, char** argv) { lsfm::testRenderingTR(); }
