//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mod.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rt_nonfinite.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "mod.h"

// Function Definitions

//
// Arguments    : double x
// Return Type  : double
//
double b_mod(double x)
{
  return x - floor(x / 2.0) * 2.0;
}

//
// File trailer for mod.cpp
//
// [EOF]
//
