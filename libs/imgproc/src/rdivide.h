//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rdivide.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//
#ifndef __RDIVIDE_H__
#  define __RDIVIDE_H__

// Include Files
#  include "logGaborFilter_types.h"
#  include "rt_nonfinite.h"
#  include "rtwtypes.h"

#  include <math.h>
#  include <stddef.h>
#  include <stdlib.h>
#  include <string.h>

// Function Declarations
extern void b_rdivide(const emxArray_real_T* x, double y, emxArray_real_T* z);
extern void c_rdivide(const emxArray_real_T* y, emxArray_real_T* z);
extern void d_rdivide(const emxArray_real_T* x, const emxArray_real_T* y, emxArray_real_T* z);
extern void rdivide(const emxArray_creal_T* x, const emxArray_real_T* y, emxArray_creal_T* z);

#endif

//
// File trailer for rdivide.h
//
// [EOF]
//
