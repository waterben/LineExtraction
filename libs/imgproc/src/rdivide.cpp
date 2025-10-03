
#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rdivide.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rdivide.h"

#include "logGaborFilter.h"
#include "logGaborFilter_emxutil.h"
#include "phasecong.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *x
//                double y
//                emxArray_real_T *z
// Return Type  : void
//
void b_rdivide(const emxArray_real_T* x, double y, emxArray_real_T* z) {
  int i2;
  int loop_ub;
  i2 = z->size[0] * z->size[1];
  z->size[0] = x->size[0];
  z->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common*)z, i2, (int)sizeof(double));
  loop_ub = x->size[0] * x->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    z->data[i2] = x->data[i2] / y;
  }
}

//
// Arguments    : const emxArray_real_T *y
//                emxArray_real_T *z
// Return Type  : void
//
void c_rdivide(const emxArray_real_T* y, emxArray_real_T* z) {
  int i3;
  int loop_ub;
  i3 = z->size[0] * z->size[1];
  z->size[0] = y->size[0];
  z->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common*)z, i3, (int)sizeof(double));
  loop_ub = y->size[0] * y->size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    z->data[i3] = 1.0 / y->data[i3];
  }
}

//
// Arguments    : const emxArray_real_T *x
//                const emxArray_real_T *y
//                emxArray_real_T *z
// Return Type  : void
//
void d_rdivide(const emxArray_real_T* x, const emxArray_real_T* y, emxArray_real_T* z) {
  int i8;
  int loop_ub;
  i8 = z->size[0] * z->size[1];
  z->size[0] = x->size[0];
  z->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common*)z, i8, (int)sizeof(double));
  loop_ub = x->size[0] * x->size[1];
  for (i8 = 0; i8 < loop_ub; i8++) {
    z->data[i8] = x->data[i8] / y->data[i8];
  }
}

//
// Arguments    : const emxArray_creal_T *x
//                const emxArray_real_T *y
//                emxArray_creal_T *z
// Return Type  : void
//
void rdivide(const emxArray_creal_T* x, const emxArray_real_T* y, emxArray_creal_T* z) {
  int i1;
  int loop_ub;
  double x_re;
  double x_im;
  double y_re;
  i1 = z->size[0] * z->size[1];
  z->size[0] = x->size[0];
  z->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common*)z, i1, (int)sizeof(creal_T));
  loop_ub = x->size[0] * x->size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    x_re = x->data[i1].re;
    x_im = x->data[i1].im;
    y_re = y->data[i1];
    if (x_im == 0.0) {
      z->data[i1].re = x_re / y_re;
      z->data[i1].im = 0.0;
    } else if (x_re == 0.0) {
      z->data[i1].re = 0.0;
      z->data[i1].im = x_im / y_re;
    } else {
      z->data[i1].re = x_re / y_re;
      z->data[i1].im = x_im / y_re;
    }
  }
}

//
// File trailer for rdivide.cpp
//
// [EOF]
//

#if defined(__clang__)
#  pragma clang diagnostic pop
#elif defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif
