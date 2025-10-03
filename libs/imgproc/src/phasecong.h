//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: phasecong.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//
#ifndef __PHASECONG_H__
#  define __PHASECONG_H__

// Include Files
#  include "logGaborFilter_types.h"
#  include "rt_nonfinite.h"
#  include "rtwtypes.h"

#  include <math.h>
#  include <stddef.h>
#  include <stdlib.h>
#  include <string.h>

// Function Declarations
extern void phasecong(const emxArray_real_T* im,
                      const emxArray_real_T* lgf,
                      const emxArray_creal_T* H,
                      double mult,
                      double k,
                      double cutOff,
                      double g,
                      double deviationGain,
                      double noiseMethod,
                      emxArray_real_T* sumf,
                      emxArray_real_T* sumh1,
                      emxArray_real_T* sumh2,
                      emxArray_real_T* e,
                      emxArray_real_T* pc);
extern double rayleighmode(const emxArray_real_T* data);

#endif

//
// File trailer for phasecong.h
//
// [EOF]
//
