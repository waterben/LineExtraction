//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: logGaborFilter.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rt_nonfinite.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "logGaborFilter_emxutil.h"
#include "exp.h"
#include "rdivide.h"
#include "power.h"
#include "lowpassfilter.h"
#include "filtergrid.h"
#include "logGaborFilter_rtwutil.h"

// Function Definitions

//
// Generate grid data for constructing filters in the frequency domain
// Arguments    : double rows
//                double cols
//                double nscale
//                double minW
//                double mult
//                double sigmaOnf
//                emxArray_real_T *lgf
//                emxArray_creal_T *H
// Return Type  : void
//
void logGaborFilter(double rows, double cols, double nscale, double minW, double
                    mult, double sigmaOnf, emxArray_real_T *lgf,
                    emxArray_creal_T *H)
{
  emxArray_real_T *radius;
  emxArray_real_T *lp;
  emxArray_real_T *logGabor;
  emxArray_creal_T *r0;
  int nx;
  int k;
  double b_rows[2];
  int s;
  emxArray_real_T *A;
  double x;
  int loop_ub;
  int i0;
  emxInit_real_T(&radius, 2);
  emxInit_real_T(&lp, 2);
  emxInit_real_T(&logGabor, 2);
  emxInit_creal_T(&r0, 2);
  filtergrid(rows, cols, radius, logGabor, lp);
  radius->data[0] = 1.0;
  nx = r0->size[0] * r0->size[1];
  r0->size[0] = logGabor->size[0];
  r0->size[1] = logGabor->size[1];
  emxEnsureCapacity((emxArray__common *)r0, nx, (int)sizeof(creal_T));
  k = logGabor->size[0] * logGabor->size[1];
  for (nx = 0; nx < k; nx++) {
    r0->data[nx].re = 0.0 * logGabor->data[nx] - lp->data[nx];
    r0->data[nx].im = logGabor->data[nx];
  }

  rdivide(r0, radius, H);
  nx = lgf->size[0] * lgf->size[1] * lgf->size[2];
  lgf->size[0] = (int)rows;
  lgf->size[1] = (int)cols;
  lgf->size[2] = (int)nscale;
  emxEnsureCapacity((emxArray__common *)lgf, nx, (int)sizeof(double));
  k = (int)rows * (int)cols * (int)nscale;
  emxFree_creal_T(&r0);
  for (nx = 0; nx < k; nx++) {
    lgf->data[nx] = 0.0;
  }

  b_rows[0] = rows;
  b_rows[1] = cols;
  lowpassfilter(b_rows, lp);

  //  Radius .4, 'sharpness' 15
  s = 0;
  emxInit_real_T(&A, 2);
  while (s <= (int)nscale - 1) {
    //  Centre frequency of filter.
    b_rdivide(radius, 1.0 / (minW * rt_powd_snf(mult, (1.0 + (double)s) - 1.0)),
              logGabor);
    nx = logGabor->size[0] * logGabor->size[1];
    for (k = 0; k + 1 <= nx; k++) {
      logGabor->data[k] = log(logGabor->data[k]);
    }

    x = log(sigmaOnf);
    power(logGabor, A);
    nx = A->size[0] * A->size[1];
    emxEnsureCapacity((emxArray__common *)A, nx, (int)sizeof(double));
    nx = A->size[0];
    k = A->size[1];
    k *= nx;
    for (nx = 0; nx < k; nx++) {
      A->data[nx] = -A->data[nx];
    }

    b_rdivide(A, 2.0 * (x * x), logGabor);
    b_exp(logGabor);
    nx = logGabor->size[0] * logGabor->size[1];
    emxEnsureCapacity((emxArray__common *)logGabor, nx, (int)sizeof(double));
    nx = logGabor->size[0];
    k = logGabor->size[1];
    k *= nx;
    for (nx = 0; nx < k; nx++) {
      logGabor->data[nx] *= lp->data[nx];
    }

    //  Apply low-pass filter
    logGabor->data[0] = 0.0;

    //  Set the value at the 0 frequency point of the
    k = logGabor->size[0] - 1;
    loop_ub = logGabor->size[1] - 1;
    for (nx = 0; nx <= loop_ub; nx++) {
      for (i0 = 0; i0 <= k; i0++) {
        lgf->data[(i0 + lgf->size[0] * nx) + lgf->size[0] * lgf->size[1] * s] =
          logGabor->data[i0 + logGabor->size[0] * nx];
      }
    }

    s++;
  }

  emxFree_real_T(&A);
  emxFree_real_T(&logGabor);
  emxFree_real_T(&lp);
  emxFree_real_T(&radius);

  //  For each scale
}

//
// File trailer for logGaborFilter.cpp
//
// [EOF]
//
