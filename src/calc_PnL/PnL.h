//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PnL.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//
#ifndef __PNL_H__
#define __PNL_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "calc_PnL_types.h"

// Function Declarations
extern void PnL(emxArray_real_T *xs, emxArray_real_T *xe, emxArray_real_T *Vw,
                emxArray_real_T *Pw, boolean_T autoChooseLines, double
                rot_cw_data[], int rot_cw_size[2], double pos_cw[3], double
                *minimalReprojectionError);
extern void b_PnL(double xs[36], double xe[36], double Vw[36], double Pw[36],
                  double rot_cw_data[], int rot_cw_size[2], double pos_cw[3]);

#endif

//
// File trailer for PnL.h
//
// [EOF]
//
