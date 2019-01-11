//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: calcampose.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//
#ifndef __CALCAMPOSE_H__
#define __CALCAMPOSE_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "calc_PnL_types.h"

// Function Declarations
extern void b_calcampose(const emxArray_real_T *XXc, const emxArray_real_T *XXw,
  double R2[9], double t2[3]);
extern void calcampose(const double XXc[72], const double XXw[72], double R2[9],
  double t2[3]);

#endif

//
// File trailer for calcampose.h
//
// [EOF]
//
