//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cos.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "cos.h"

#include "logGaborFilter.h"
#include "phasecong.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : emxArray_real_T *x
// Return Type  : void
//
void b_cos(emxArray_real_T* x) {
  int nx;
  int k;
  nx = x->size[0] * x->size[1];
  for (k = 0; k + 1 <= nx; k++) {
    x->data[k] = cos(x->data[k]);
  }
}

//
// File trailer for cos.cpp
//
// [EOF]
//
