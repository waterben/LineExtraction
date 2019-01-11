//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: meshgrid.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rt_nonfinite.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "meshgrid.h"
#include "logGaborFilter_emxutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *x
//                const emxArray_real_T *y
//                emxArray_real_T *xx
//                emxArray_real_T *yy
// Return Type  : void
//
void meshgrid(const emxArray_real_T *x, const emxArray_real_T *y,
              emxArray_real_T *xx, emxArray_real_T *yy)
{
  emxArray_real_T *a;
  int ibtile;
  int outsize_idx_1;
  int varargin_1_idx_0;
  int k;
  int y_idx_0;
  emxInit_real_T(&a, 2);
  if ((x->size[1] == 0) || (y->size[1] == 0)) {
    ibtile = xx->size[0] * xx->size[1];
    xx->size[0] = 0;
    xx->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)xx, ibtile, (int)sizeof(double));
    ibtile = yy->size[0] * yy->size[1];
    yy->size[0] = 0;
    yy->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)yy, ibtile, (int)sizeof(double));
  } else {
    outsize_idx_1 = x->size[1];
    ibtile = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = outsize_idx_1;
    emxEnsureCapacity((emxArray__common *)a, ibtile, (int)sizeof(double));
    for (ibtile = 0; ibtile < outsize_idx_1; ibtile++) {
      a->data[a->size[0] * ibtile] = x->data[ibtile];
    }

    varargin_1_idx_0 = y->size[1];
    outsize_idx_1 = a->size[1];
    ibtile = xx->size[0] * xx->size[1];
    xx->size[0] = varargin_1_idx_0;
    xx->size[1] = outsize_idx_1;
    emxEnsureCapacity((emxArray__common *)xx, ibtile, (int)sizeof(double));
    for (outsize_idx_1 = 0; outsize_idx_1 + 1 <= a->size[1]; outsize_idx_1++) {
      ibtile = outsize_idx_1 * varargin_1_idx_0;
      for (k = 1; k <= varargin_1_idx_0; k++) {
        xx->data[(ibtile + k) - 1] = a->data[outsize_idx_1];
      }
    }

    varargin_1_idx_0 = x->size[1];
    y_idx_0 = y->size[1];
    ibtile = yy->size[0] * yy->size[1];
    yy->size[0] = y_idx_0;
    yy->size[1] = varargin_1_idx_0;
    emxEnsureCapacity((emxArray__common *)yy, ibtile, (int)sizeof(double));
    y_idx_0 = y->size[1];
    for (outsize_idx_1 = 1; outsize_idx_1 <= varargin_1_idx_0; outsize_idx_1++)
    {
      ibtile = (outsize_idx_1 - 1) * y_idx_0;
      for (k = 1; k <= y_idx_0; k++) {
        yy->data[(ibtile + k) - 1] = y->data[k - 1];
      }
    }
  }

  emxFree_real_T(&a);
}

//
// File trailer for meshgrid.cpp
//
// [EOF]
//
