/*
* Copyright (c) 1993-1997, Silicon Graphics, Inc.
* ALL RIGHTS RESERVED 
* Permission to use, copy, modify, and distribute this software for 
* any purpose and without fee is hereby granted, provided that the above
* copyright notice appear in all copies and that both the copyright notice
* and this permission notice appear in supporting documentation, and that 
* the name of Silicon Graphics, Inc. not be used in advertising
* or publicity pertaining to distribution of the software without specific,
* written prior permission. 
*
* THE MATERIAL EMBODIED ON THIS SOFTWARE IS PROVIDED TO YOU "AS-IS"
* AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR OTHERWISE,
* INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR
* FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL SILICON
* GRAPHICS, INC.  BE LIABLE TO YOU OR ANYONE ELSE FOR ANY DIRECT,
* SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY
* KIND, OR ANY DAMAGES WHATSOEVER, INCLUDING WITHOUT LIMITATION,
* LOSS OF PROFIT, LOSS OF USE, SAVINGS OR REVENUE, OR THE CLAIMS OF
* THIRD PARTIES, WHETHER OR NOT SILICON GRAPHICS, INC.  HAS BEEN
* ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE
* POSSESSION, USE OR PERFORMANCE OF THIS SOFTWARE.
* 
* US Government Users Restricted Rights 
* Use, duplication, or disclosure by the Government is subject to
* restrictions set forth in FAR 52.227.19(c)(2) or subparagraph
* (c)(1)(ii) of the Rights in Technical Data and Computer Software
* clause at DFARS 252.227-7013 and/or in similar or successor
* clauses in the FAR or the DOD or NASA FAR Supplement.
* Unpublished-- rights reserved under the copyright laws of the
* United States.  Contractor/manufacturer is Silicon Graphics,
* Inc., 2011 N.  Shoreline Blvd., Mountain View, CA 94039-7311.
*
* OpenGL(R) is a registered trademark of Silicon Graphics, Inc.
*/

/*  accpersp.c
*  Use the accumulation buffer to do full-scene antialiasing
*  on a scene with perspective projection, using the special
*  routines accFrustum() and accPerspective().
*/
#ifdef WIN32
#include <windows.h>
#endif

#include <GL/glut.h>
#include <GL/glext.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <opengl_sim/jitter.h>

#include <GL/freeglut.h>    // required for glutSetOption

#include <string.h>


#ifdef WIN32
#define near zNear
#define far zFar
#endif

#define PI_ 3.14159265358979323846

#define ACSIZE	8

// Constants used for menu callback
#define	ANTI_ALIAS		0
#define	MULTI_SAMPLE	1
#define ROTATE			2
#define POLYGON_SMOOTH	3
#define FORCE_REDISPLAY	4

int		giAnimate	= 0;
float	gfRotAngle	= 0.0;

// antialiasing
jitter_point	*gpJitter = j8;
int	giAntiAlias			= 0;
int	giNumSamples		= ACSIZE;

// Flag for enabling/disableing multisampling
int	giMultiSample	= 1;

// Flag for enabling/disabling polygon smooth
int giPolygonSmooth	= 0;

// Timing variables
long	glStartTime	= 0;
long	glEndTime	= 0;

int	giWindowWidth = 600;
int giWindowHeight = 400;
int	giHalfWindowWidth = giWindowWidth / 2;
int	giHalfWindowHeight = giWindowHeight / 2;

int		giTextPos		= 0;
int		giTextPosDelta	= 30;
float	gfTextScale		= 0.15;


//
// Draw text to screen
//
void DrawText(char *strText)
{   
  char *pChar = NULL;
 
  // Switch to screenspace to draw text
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, giWindowWidth, 0, giWindowHeight);

  glDisable(GL_LIGHTING);

  glPushMatrix();
  glTranslatef(10.0, giTextPos, 0.0);
  glScalef(gfTextScale, gfTextScale, gfTextScale);

  for(pChar = strText; *pChar; pChar++) {
	  glutStrokeCharacter(GLUT_STROKE_ROMAN, *pChar); 
  }
  glPopMatrix();

  // Update text position
  giTextPos -= giTextPosDelta;

  glEnable(GL_LIGHTING);
}

/* accFrustum()
* The first 6 arguments are identical to the glFrustum() call.
*  
* pixdx and pixdy are anti-alias jitter in pixels. 
* Set both equal to 0.0 for no anti-alias jitter.
* eyedx and eyedy are depth-of field jitter in pixels. 
* Set both equal to 0.0 for no depth of field effects.
*
* focus is distance from eye to plane in focus. 
* focus must be greater than, but not equal to 0.0.
*
* Note that accFrustum() calls glTranslatef().  You will 
* probably want to insure that your ModelView matrix has been 
* initialized to identity before calling accFrustum().
*/
void accFrustum(GLdouble left, GLdouble right, GLdouble bottom, 
				GLdouble top, GLdouble near, GLdouble far, GLdouble pixdx, 
				GLdouble pixdy, GLdouble eyedx, GLdouble eyedy, GLdouble focus)
{
	GLdouble xwsize, ywsize; 
	GLdouble dx, dy;
	GLint viewport[4];

	glGetIntegerv (GL_VIEWPORT, viewport);

	xwsize = right - left;
	ywsize = top - bottom;

	dx = -(pixdx*xwsize/(GLdouble) viewport[2] + eyedx*near/focus);
	dy = -(pixdy*ywsize/(GLdouble) viewport[3] + eyedy*near/focus);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum (left + dx, right + dx, bottom + dy, top + dy, near, far);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef (-eyedx, -eyedy, 0.0);
}

/* accPerspective()
* 
* The first 4 arguments are identical to the gluPerspective() call.
* pixdx and pixdy are anti-alias jitter in pixels. 
* Set both equal to 0.0 for no anti-alias jitter.
* eyedx and eyedy are depth-of field jitter in pixels. 
* Set both equal to 0.0 for no depth of field effects.
*
* focus is distance from eye to plane in focus. 
* focus must be greater than, but not equal to 0.0.
*
* Note that accPerspective() calls accFrustum().
*/
void accPerspective(GLdouble fovy, GLdouble aspect, 
					GLdouble near, GLdouble far, GLdouble pixdx, GLdouble pixdy, 
					GLdouble eyedx, GLdouble eyedy, GLdouble focus)
{
	GLdouble fov2,left,right,bottom,top;

	fov2 = ((fovy*PI_) / 180.0) / 2.0;

	top = near / (cos(fov2) / sin(fov2));
	bottom = -top;

	right = top * aspect;
	left = -right;

	accFrustum (left, right, bottom, top, near, far,
		pixdx, pixdy, eyedx, eyedy, focus);
}

/*  Initialize lighting and other values.
*/
void init(void)
{
	GLfloat mat_ambient[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 0.0, 0.0, 10.0, 1.0 };
	GLfloat lm_ambient[] = { 0.2, 0.2, 0.2, 1.0 };

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lm_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glShadeModel (GL_FLAT);

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearAccum(0.0, 0.0, 0.0, 0.0);
}

void displayObjects(void) 
{
	GLfloat torus_diffuse[] = { 0.7, 0.7, 0.0, 1.0 };
	GLfloat cube_diffuse[] = { 0.0, 0.7, 0.7, 1.0 };
	GLfloat sphere_diffuse[] = { 0.7, 0.0, 0.7, 1.0 };
	GLfloat octa_diffuse[] = { 0.7, 0.4, 0.4, 1.0 };

	glPushMatrix ();
	glTranslatef (0.0, 0.0, -5.0); 

	if (giAnimate == 1) {
		glRotatef(-gfRotAngle, 0.0, 0.0, 1.0);
		gfRotAngle += 1.0;
		if (gfRotAngle > 360.0) {
			gfRotAngle = 0.0;
		}
	}

	glRotatef (30.0, 1.0, 0.0, 0.0);

	glPushMatrix ();
	glTranslatef (-0.80, 0.35, 0.0); 
	glRotatef (100.0, 1.0, 0.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, torus_diffuse);
	glutSolidTorus (0.275, 0.85, 16, 16);
	glPopMatrix ();

	glPushMatrix ();
	glTranslatef (-0.75, -0.50, 0.0); 
	glRotatef (45.0, 0.0, 0.0, 1.0);
	glRotatef (45.0, 1.0, 0.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, cube_diffuse);
	glutSolidCube (1.5);
	glPopMatrix ();

	glPushMatrix ();
	glTranslatef (0.75, 0.60, 0.0); 
	glRotatef (30.0, 1.0, 0.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, sphere_diffuse);
	glutSolidSphere (1.0, 16, 16);
	glPopMatrix ();

	glPushMatrix ();
	glTranslatef (0.70, -0.90, 0.25); 
	glMaterialfv(GL_FRONT, GL_DIFFUSE, octa_diffuse);
	glutSolidOctahedron ();
	glPopMatrix ();

	glPopMatrix ();
}


void display(void)
{
	GLint	viewport[4];
	GLint	iMultiSample	= 0;
	GLint	iNumSamples		= 0;
	int		jitter;
	long	lDeltaTime = 0;
	char	strMsg[256];

	memset(strMsg, 0, sizeof(strMsg));
		
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);

	// Restore font location
	giTextPos = giWindowHeight - giTextPosDelta;

	lDeltaTime = glEndTime - glStartTime;

    //glStartTime = timeGetTime();

	glGetIntegerv (GL_VIEWPORT, viewport);

	// MultiSampling
	glGetIntegerv(GL_SAMPLE_BUFFERS, &iMultiSample);
	glGetIntegerv(GL_SAMPLES, &iNumSamples);

	if (giMultiSample == 1) {
		glEnable(GL_MULTISAMPLE);
		glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
	} else {
		glDisable(GL_MULTISAMPLE);
	}

	if (giPolygonSmooth == 1) {
		glEnable(GL_POLYGON_SMOOTH);
		glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	} else {
		glDisable(GL_POLYGON_SMOOTH);
	}

	if (giAntiAlias == 1) {
		for (jitter = 0; jitter < giNumSamples; jitter++) {
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			accPerspective (50.0, 
				(GLdouble) viewport[2]/(GLdouble) viewport[3], 
				1.0, 15.0, gpJitter[jitter].x, gpJitter[jitter].y, 0.0, 0.0, 1.0);
			displayObjects ();
			glAccum(GL_ACCUM, 1.0/giNumSamples);
		}
		glAccum (GL_RETURN, 1.0);
	} else {
		accPerspective (50.0, 
			(GLdouble) viewport[2]/(GLdouble) viewport[3], 
			1.0, 15.0, 0.0, 0.0, 0.0, 0.0, 1.0);
		displayObjects ();
	}

	// Draw stats and info
	// do this here so the accumulation buffer process above doesn't clear
	// this info from the screen.
	memset(strMsg, 0, sizeof(strMsg));
	sprintf(strMsg, "Render time:  %d  milliseconds", lDeltaTime);
	DrawText(strMsg);

	memset(strMsg, 0, sizeof(strMsg));
	if (giMultiSample == 1) {
		sprintf(strMsg, "Multisample Enabled  : Number_of_Samples(%d)", iNumSamples);
		DrawText(strMsg);
	} else {
		DrawText("Multisample Disabled");
	}

	if (giPolygonSmooth == 1) {
		DrawText("Polygon Smooth Enabled");
	} else {
		DrawText("Polygon Smooth Disabled");
	}
	
	memset(strMsg, 0, sizeof(strMsg));
	if (giAntiAlias == 1) {
		sprintf(strMsg, "Accum AntiAliasing Enabled  : Number_of_Jitter_Points(%d)", giNumSamples);
		DrawText(strMsg);
	} else {
		DrawText("Accum AntiAliasing Disabled");
	}

	glFlush();
	glutSwapBuffers();

	if (giAnimate == 1) {
		glutPostRedisplay();
	}

	// use finish as we want to wait until above gl commands are
	// done executing before calculating time to execute.
	glFinish();

    //glEndTime = timeGetTime();

	printf("Render Time (%d)\n", glEndTime - glStartTime);
}

void reshape(int width, int height)
{
	giWindowWidth = width;
	giWindowHeight = height;

	glViewport(0, 0, width, height);
}

// Simple handler for toggling animation
void ToggleAnimation(void)
{
	giAnimate = !giAnimate;
	glutPostRedisplay();
}

// Simple handler for toggling antialiasing
void ToggleAntiAliasing(void)
{
	giAntiAlias = !giAntiAlias;
	glutPostRedisplay();	
}

// Simple handler for toggling multisampling
void ToggleMultiSampling(void)
{
	giMultiSample = !giMultiSample;
	glutPostRedisplay();
}

// Simple handler for toggling polygon smooth
void TogglePolygonSmooth(void)
{
	giPolygonSmooth = !giPolygonSmooth;
	glutPostRedisplay();
}

// Keyboard callback routine
void keyboard(unsigned char key, int x, int y)
{
	switch (key) {
		case 'a':
		case 'A':
			ToggleAntiAliasing();
			break;

		case '1':
			gpJitter = j2;
			giNumSamples = 2;
			glutPostRedisplay();
			break;
		case '2':
			gpJitter = j4;
			giNumSamples = 4;
			glutPostRedisplay();
			break;
		case '3':
			gpJitter = j8;
			giNumSamples = 8;
			glutPostRedisplay();
			break;
		case '4':
			gpJitter = j15;
			giNumSamples = 15;
			glutPostRedisplay();
			break;

		case 'm':
		case 'M':
			ToggleMultiSampling();
			break;

		case 'r':
		case 'R':
			ToggleAnimation();
			break;

		case 'p':
		case 'P':
			TogglePolygonSmooth();
			break;

		case 'd':
		case 'D':
			glutPostRedisplay();
			break;

		case 27:
			exit(0);
			break;
	}
}
// 
// GlutMenu Function
//
void MenuFunc(int iValue)
{
	switch(iValue) {
		case ANTI_ALIAS:
			ToggleAntiAliasing();
			break;
		case MULTI_SAMPLE:
			ToggleMultiSampling();
			break;
		case ROTATE:
			ToggleAnimation();
			break;
		case POLYGON_SMOOTH:
			TogglePolygonSmooth();
			break;
		case FORCE_REDISPLAY:
			glutPostRedisplay();
			break;

		default:
			break;

	}
}

/*  Main Loop
*  Be certain you request an accumulation buffer.
*/
int main(int argc, char** argv)
{
	glutInit(&argc, argv);
    glutSetOption(GLUT_MULTISAMPLE, 16);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_ACCUM | GLUT_DEPTH | GLUT_MULTISAMPLE);

	glutInitWindowSize(giWindowWidth, giWindowHeight);
	glutInitWindowPosition (100, 100);
	glutCreateWindow (argv[0]);
	init();
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);

	// Create popup menu
	glutCreateMenu(MenuFunc);
	glutAddMenuEntry("Toggle Accum AntiAliasing [a]", ANTI_ALIAS);
	glutAddMenuEntry("Toggle Multisampling [m]", MULTI_SAMPLE);
	glutAddMenuEntry("Toggle Polygon Smooth [p]", POLYGON_SMOOTH);
	glutAddMenuEntry("Rotate Objects [r]", ROTATE);
	glutAddMenuEntry("Force a Redisplay [d]", FORCE_REDISPLAY);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glutMainLoop();
	return 0;
}
