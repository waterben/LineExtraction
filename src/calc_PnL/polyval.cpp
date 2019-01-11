//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: polyval.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "polyval.h"

// Function Definitions

//
// Arguments    : const double p[15]
//                const double x_data[]
//                const int x_size[1]
//                double y_data[]
//                int y_size[1]
// Return Type  : void
//
void polyval(const double p[15], const double x_data[], const int x_size[1],
             double y_data[], int y_size[1])
{
  int loop_ub;
  int i6;
  int k;
  y_size[0] = (signed char)x_size[0];
  if (!((signed char)x_size[0] == 0)) {
    y_size[0] = (signed char)x_size[0];
    loop_ub = (signed char)x_size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      y_data[i6] = p[0];
    }

    for (k = 0; k < 14; k++) {
      y_size[0] = x_size[0];
      loop_ub = x_size[0];
      for (i6 = 0; i6 < loop_ub; i6++) {
        y_data[i6] = x_data[i6] * y_data[i6] + p[k + 1];
      }
    }
  }
}

//
// File trailer for polyval.cpp
//
// [EOF]
//
