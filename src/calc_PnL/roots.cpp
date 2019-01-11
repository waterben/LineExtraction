//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: roots.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "roots.h"
#include "eig.h"

// Function Definitions

//
// Arguments    : const double c[16]
//                creal_T r_data[]
//                int r_size[1]
// Return Type  : void
//
void roots(const double c[16], creal_T r_data[], int r_size[1])
{
  int k1;
  int k2;
  int companDim;
  double ctmp[16];
  boolean_T exitg1;
  int j;
  boolean_T exitg2;
  int a_size[2];
  creal_T a_data[225];
  int eiga_size[1];
  creal_T eiga_data[15];
  memset(&r_data[0], 0, 15U * sizeof(creal_T));
  k1 = 1;
  while ((k1 <= 16) && (!(c[k1 - 1] != 0.0))) {
    k1++;
  }

  k2 = 16;
  while ((k2 >= k1) && (!(c[k2 - 1] != 0.0))) {
    k2--;
  }

  if (k1 < k2) {
    companDim = k2 - k1;
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp[j] = c[k1 + j] / c[k1 - 1];
        if (rtIsInf(fabs(ctmp[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }

      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }

    if (companDim < 1) {
      if (1 > 16 - k2) {
        r_size[0] = 0;
      } else {
        r_size[0] = 16 - k2;
      }
    } else {
      a_size[0] = companDim;
      a_size[1] = companDim;
      k1 = companDim * companDim;
      for (j = 0; j < k1; j++) {
        a_data[j].re = 0.0;
        a_data[j].im = 0.0;
      }

      for (k1 = 0; k1 + 1 < companDim; k1++) {
        a_data[companDim * k1].re = -ctmp[k1];
        a_data[companDim * k1].im = 0.0;
        a_data[(k1 + companDim * k1) + 1].re = 1.0;
        a_data[(k1 + companDim * k1) + 1].im = 0.0;
      }

      a_data[companDim * (companDim - 1)].re = -ctmp[companDim - 1];
      a_data[companDim * (companDim - 1)].im = 0.0;
      for (k1 = 1; k1 <= 16 - k2; k1++) {
        r_data[k1 - 1].re = 0.0;
        r_data[k1 - 1].im = 0.0;
      }

      eig(a_data, a_size, eiga_data, eiga_size);
      for (k1 = 1; k1 <= companDim; k1++) {
        r_data[(k1 - k2) + 15] = eiga_data[k1 - 1];
      }

      r_size[0] = (companDim - k2) + 16;
    }
  } else if (1 > 16 - k2) {
    r_size[0] = 0;
  } else {
    r_size[0] = 16 - k2;
  }
}

//
// File trailer for roots.cpp
//
// [EOF]
//
