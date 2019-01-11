//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//
#ifndef __SVD_H__
#define __SVD_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "calc_PnL_types.h"

// Function Declarations
extern void b_svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *
                  S, double V[36]);
extern void svd(const double A[138], double U[529], double S[138], double V[36]);

#endif

//
// File trailer for svd.h
//
// [EOF]
//
