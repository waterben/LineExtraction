//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: R_and_T.h
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//
#ifndef __R_AND_T_H__
#define __R_AND_T_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "calc_PnL_types.h"

// Function Declarations
extern void R_and_T(const emxArray_real_T *xs, const emxArray_real_T *xe, const
                    emxArray_real_T *P1w, const emxArray_real_T *P2w, const
                    double initRot_cw[9], const double initPos_cw[3], double
                    maxIterNum, double TerminateTh, double Rot_cw[9], double
                    Pos_cw[3]);
extern void b_R_and_T(const double xs[36], const double xe[36], const double
                      P1w[36], const double P2w[36], const double
                      initRot_cw_data[], const int initRot_cw_size[2], const
                      double initPos_cw[3], double Rot_cw_data[], int
                      Rot_cw_size[2], double Pos_cw_data[], int Pos_cw_size[1]);
extern void b_eml_xscal(double a, double x[36], int ix0);
extern double c_eml_xdotc(int n, const double x[36], int ix0, const double y[36],
  int iy0);
extern void e_eml_xaxpy(int n, double a, int ix0, double y[36], int iy0);
extern void eml_xrot(double x[36], int ix0, int iy0, double c, double s);
extern void eml_xswap(double x[36], int ix0, int iy0);

#endif

//
// File trailer for R_and_T.h
//
// [EOF]
//
