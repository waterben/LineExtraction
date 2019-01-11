//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: camParams.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//
#ifndef __CAMPARAMS_H__
#define __CAMPARAMS_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "calc_PnL_types.h"

// Function Declarations
extern void b_eml_xgesvd(const double A[9], double U[9], double S[3], double V[9]);
extern void camParams(const double M[12], const double p3D[4], double R[9],
                      double T[3], double *ox, double *oy, double *fx, double
                      *fy);
extern void eml_xrotg(double *a, double *b, double *c, double *s);

#endif

//
// File trailer for camParams.h
//
// [EOF]
//
