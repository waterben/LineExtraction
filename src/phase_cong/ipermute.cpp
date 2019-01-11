//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ipermute.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rt_nonfinite.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "ipermute.h"
#include "logGaborFilter_emxutil.h"

// Function Definitions

//
// Arguments    : const emxArray_creal_T *b
//                emxArray_creal_T *a
// Return Type  : void
//
void ipermute(const emxArray_creal_T *b, emxArray_creal_T *a)
{
  int i6;
  int loop_ub;
  int b_loop_ub;
  int i7;
  i6 = a->size[0] * a->size[1];
  a->size[0] = b->size[1];
  a->size[1] = b->size[0];
  emxEnsureCapacity((emxArray__common *)a, i6, (int)sizeof(creal_T));
  loop_ub = b->size[0];
  for (i6 = 0; i6 < loop_ub; i6++) {
    b_loop_ub = b->size[1];
    for (i7 = 0; i7 < b_loop_ub; i7++) {
      a->data[i7 + a->size[0] * i6] = b->data[i6 + b->size[0] * i7];
    }
  }
}

//
// File trailer for ipermute.cpp
//
// [EOF]
//
