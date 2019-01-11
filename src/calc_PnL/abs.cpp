//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: abs.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "abs.h"

// Function Definitions

//
// Arguments    : const double x_data[]
//                const int x_size[1]
//                double y_data[]
//                int y_size[1]
// Return Type  : void
//
void b_abs(const double x_data[], const int x_size[1], double y_data[], int
           y_size[1])
{
  int k;
  y_size[0] = (signed char)x_size[0];
  for (k = 0; k < x_size[0]; k++) {
    y_data[k] = fabs(x_data[k]);
  }
}

//
// File trailer for abs.cpp
//
// [EOF]
//
