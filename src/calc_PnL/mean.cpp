//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mean.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "mean.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *x
//                double y[3]
// Return Type  : void
//
void b_mean(const emxArray_real_T *x, double y[3])
{
  int iy;
  int ixstart;
  int j;
  int ix;
  double s;
  int k;
  iy = -1;
  ixstart = -1;
  for (j = 0; j < 3; j++) {
    ixstart++;
    ix = ixstart;
    s = x->data[ixstart];
    for (k = 2; k <= x->size[1]; k++) {
      ix += 3;
      s += x->data[ix];
    }

    iy++;
    y[iy] = s;
  }

  iy = x->size[1];
  for (ixstart = 0; ixstart < 3; ixstart++) {
    y[ixstart] /= (double)iy;
  }
}

//
// Arguments    : const double x[72]
//                double y[3]
// Return Type  : void
//
void mean(const double x[72], double y[3])
{
  int iy;
  int ixstart;
  int j;
  int ix;
  double s;
  int k;
  iy = -1;
  ixstart = -1;
  for (j = 0; j < 3; j++) {
    ixstart++;
    ix = ixstart;
    s = x[ixstart];
    for (k = 0; k < 23; k++) {
      ix += 3;
      s += x[ix];
    }

    iy++;
    y[iy] = s;
  }

  for (iy = 0; iy < 3; iy++) {
    y[iy] /= 24.0;
  }
}

//
// File trailer for mean.cpp
//
// [EOF]
//
