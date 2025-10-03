//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: lowpassfilter.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//
#ifndef __LOWPASSFILTER_H__
#  define __LOWPASSFILTER_H__

// Include Files
#  include "logGaborFilter_types.h"
#  include "rt_nonfinite.h"
#  include "rtwtypes.h"

#  include <math.h>
#  include <stddef.h>
#  include <stdlib.h>
#  include <string.h>

// Function Declarations
extern void eml_ifftshift(emxArray_real_T* x, int dim);
extern void lowpassfilter(const double sze[2], emxArray_real_T* f);

#endif

//
// File trailer for lowpassfilter.h
//
// [EOF]
//
