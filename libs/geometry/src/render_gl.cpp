

#include <geometry/render_gl.hpp>

#ifdef WIN32
#  include <windows.h>
#endif
#include "tr.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>

#include <string.h>


namespace lsfm {

#define TILESIZE 100
#define BORDER 0

#define NUMBALLS 30
static GLfloat BallPos[NUMBALLS][3];
static GLfloat BallSize[NUMBALLS];
static GLfloat BallColor[NUMBALLS][4];

static GLboolean Perspective = GL_TRUE;

static int WindowWidth = 500, WindowHeight = 500;

#ifndef WIN32
// Crazy Workaround for crash at startup when using glut and string or stream, etc.
// from: http://stackoverflow.com/questions/31579243/segmentation-fault-before-main-when-using-glut-and-stdstring
#  include <pthread.h>
void* simpleFunc(void*) { return NULL; }
void forcePThreadLink() {
  pthread_t t1;
  pthread_create(&t1, NULL, &simpleFunc, NULL);
}
#endif

/* Return random float in [0,1] */
float Random() {
  int i = rand();
  return static_cast<float>(i % 1000) / 1000.0f;
}


void DrawBallNumber(int k) {
  char str[100];
  int i, n;
  sprintf(str, "%d", k);
  n = static_cast<int>(strlen(str));  // Safe: string length from sprintf is always small
  for (i = 0; i < n; i++) {
    glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
  }
}


/* Draw my stuff */
void DrawScene(TRcontext* tr) {
  int i;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (i = 0; i < NUMBALLS; i++) {
    int t;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, BallColor[i]);
    glPushMatrix();
    glTranslatef(BallPos[i][0], BallPos[i][1], BallPos[i][2]);
    t = 12 + static_cast<int>(BallSize[i] * 12);
    glutSolidSphere(BallSize[i], t, t);

    glDisable(GL_LIGHTING);
    glColor3ub(255, 255, 255);
    trRasterPos3f(tr, 0, 0, BallSize[i]); /* front of ball */
    DrawBallNumber(i);
    glEnable(GL_LIGHTING);

    glPopMatrix();
  }
}


/* Do a demonstration of tiled rendering */
void Display(void) {
  GLubyte* image;
  int tileCount = 0;
  TRcontext* tr;
  int i;

  /* Generate random balls */
  for (i = 0; i < NUMBALLS; i++) {
    BallPos[i][0] = -2.0f + 4.0f * Random();
    BallPos[i][1] = -2.0f + 4.0f * Random();
    BallPos[i][2] = -2.0f + 4.0f * Random();
    BallSize[i] = Random();
    BallColor[i][0] = Random();
    BallColor[i][1] = Random();
    BallColor[i][2] = Random();
    BallColor[i][3] = 1.0;
  }

  /* allocate final image buffer */
  // image = malloc(WindowWidth * WindowHeight * 4 * sizeof(GLubyte));
  image = static_cast<uint8_t*>(
      std::malloc(static_cast<size_t>(WindowWidth) * static_cast<size_t>(WindowHeight) * 4 * sizeof(GLubyte)));
  if (!image) {
    printf("Malloc failed!\n");
    return;
  }

  /* Setup tiled rendering */
  tr = trNew();
  trTileSize(tr, TILESIZE, TILESIZE, BORDER);
  /* We don't call trTileBuffer() since we're not interested in getting
   * each individual tile's data.
   */
  trImageSize(tr, WindowWidth, WindowHeight);
  trImageBuffer(tr, GL_RGB, GL_UNSIGNED_BYTE, image);

  if (Perspective)
    trFrustum(tr, -1.0, 1.0, -1.0, 1.0, 5.0, 25.0);
  else
    trOrtho(tr, -3.0, 3.0, -3.0, 3.0, -3.0, 3.0);

  /* Draw tiles */
  do {
    trBeginTile(tr);
    DrawScene(tr);
    tileCount++;
  } while (trEndTile(tr));
  printf("%d tiles drawn\n", tileCount);

  trDelete(tr);

  /* Show final image, might otherwise write it to a file */
  /* Do this ugliness to set rasterpos to lower-left corner of window */
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0, WindowWidth, 0.0, WindowHeight, -1.0, 1.0);
  glRasterPos2i(0, 0);

  /* Disable dithering since dithering was done when each tile was generated*/
  glDisable(GL_DITHER);
  glDrawPixels(WindowWidth, WindowHeight, GL_RGB, GL_UNSIGNED_BYTE, image);
  glEnable(GL_DITHER);

  /* restore matrices*/
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glFlush();

  free(image);
}


void Reshape(int width, int height) {
  WindowWidth = width;
  WindowHeight = height;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (Perspective)
    glFrustum(-1.0, 1.0, -1.0, 1.0, 5.0, 25.0);
  else
    glOrtho(-3.0, 3.0, -3.0, 3.0, -3.0, 3.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if (Perspective) glTranslatef(0.0, 0.0, -15.0);
}


void Key(unsigned char key, int x, int y) {
  (void)x;
  (void)y;
  switch (key) {
    case 27:
      exit(0);
      break;
  }
  glutPostRedisplay();
}


void Init(void) {
  static GLfloat pos[4] = {0.0, 0.0, 10.0, 0.0};
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_POSITION, pos);
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);

  /* Enable this to test wide lines and borders
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glLineWidth(4.0);
  */
}


void testRenderingTR() {
  char* myargv[1];
  int myargc = 1;
  myargv[0] = strdup("RenderingTest");

  glutInit(&myargc, myargv);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(WindowWidth, WindowHeight);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH);
  glutCreateWindow(myargv[0]);
  Init();
  glutReshapeFunc(Reshape);
  glutKeyboardFunc(Key);
  glutDisplayFunc(Display);
  glutMainLoop();
}


}  // namespace lsfm
