//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: cross.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "cross.h"

// Function Definitions

//
// Arguments    : const double a_data[]
//                const double b[3]
//                double c[3]
// Return Type  : void
//
void b_cross(const double a_data[], const double b[3], double c[3])
{
  c[0] = a_data[1] * b[2] - a_data[2] * b[1];
  c[1] = a_data[2] * b[0] - a_data[0] * b[2];
  c[2] = a_data[0] * b[1] - a_data[1] * b[0];
}

//
// Arguments    : const double a[3]
//                const double b[3]
//                double c[3]
// Return Type  : void
//
void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

//
// File trailer for cross.cpp
//
// [EOF]
//
