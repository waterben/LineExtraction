//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: logGaborFilter.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//
#pragma once

// Include Files
#include "logGaborFilter_types.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// Function Declarations
extern void logGaborFilter(double rows,
                           double cols,
                           double nscale,
                           double minW,
                           double mult,
                           double sigmaOnf,
                           emxArray_real_T* lgf,
                           emxArray_creal_T* H);


//
// File trailer for logGaborFilter.h
//
// [EOF]
//
