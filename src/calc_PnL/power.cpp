//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: power.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "power.h"
#include "calc_PnL_emxutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *a
//                emxArray_real_T *y
// Return Type  : void
//
void power(const emxArray_real_T *a, emxArray_real_T *y)
{
  unsigned int uv1[2];
  int i15;
  int k;
  for (i15 = 0; i15 < 2; i15++) {
    uv1[i15] = (unsigned int)a->size[i15];
  }

  i15 = y->size[0] * y->size[1];
  y->size[0] = 3;
  y->size[1] = (int)uv1[1];
  emxEnsureCapacity((emxArray__common *)y, i15, (int)sizeof(double));
  i15 = 3 * (int)uv1[1];
  for (k = 0; k < i15; k++) {
    y->data[k] = a->data[k] * a->data[k];
  }
}

//
// File trailer for power.cpp
//
// [EOF]
//
