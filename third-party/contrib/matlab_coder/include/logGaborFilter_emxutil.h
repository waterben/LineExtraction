//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: logGaborFilter_emxutil.h
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
extern void emxEnsureCapacity(emxArray__common* emxArray, int oldNumel, int elementSize);
extern void emxFree_creal_T(emxArray_creal_T** pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T** pEmxArray);
extern void emxFree_real_T(emxArray_real_T** pEmxArray);
extern void emxInit_creal_T(emxArray_creal_T** pEmxArray, int numDimensions);
extern void emxInit_int32_T(emxArray_int32_T** pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T** pEmxArray, int numDimensions);
extern void emxInit_real_T1(emxArray_real_T** pEmxArray, int numDimensions);
extern void emxInit_real_T2(emxArray_real_T** pEmxArray, int numDimensions);


//
// File trailer for logGaborFilter_emxutil.h
//
// [EOF]
//
