//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "main.h"
#include "calc_PnL_terminate.h"
#include "calc_PnL_emxAPI.h"
#include "calc_PnL_initialize.h"

// Function Declarations
static void argInit_3x1_real_T(double result[3]);
static void argInit_3x3_real_T(double result[9]);
static emxArray_real_T *argInit_3xUnbounded_real_T();
static boolean_T argInit_boolean_T();
static double argInit_real_T();
static void main_PnL();
static void main_R_and_T();
static void main_calc_PnL();

// Function Definitions

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  int b_j0;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 3; b_j0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[b_j0] = argInit_real_T();
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_3x3_real_T(double result[9])
{
  int b_j0;
  int b_j1;

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 3; b_j0++) {
    for (b_j1 = 0; b_j1 < 3; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[b_j0 + 3 * b_j1] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : emxArray_real_T *
//
static emxArray_real_T *argInit_3xUnbounded_real_T()
{
  emxArray_real_T *result;
  static int iv10[2] = { 3, 2 };

  int b_j0;
  int b_j1;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result = emxCreateND_real_T(2, *(int (*)[2])&iv10[0]);

  // Loop over the array to initialize each element.
  for (b_j0 = 0; b_j0 < 3; b_j0++) {
    for (b_j1 = 0; b_j1 < result->size[1U]; b_j1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result->data[b_j0 + result->size[0] * b_j1] = argInit_real_T();
    }
  }

  return result;
}

//
// Arguments    : void
// Return Type  : boolean_T
//
static boolean_T argInit_boolean_T()
{
  return false;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_PnL()
{
  emxArray_real_T *xs;
  emxArray_real_T *xe;
  emxArray_real_T *Vw;
  emxArray_real_T *Pw;
  double minimalReprojectionError;
  double pos_cw[3];
  int rot_cw_size[2];
  double rot_cw_data[9];

  // Initialize function 'PnL' input arguments.
  // Initialize function input argument 'xs'.
  xs = argInit_3xUnbounded_real_T();

  // Initialize function input argument 'xe'.
  xe = argInit_3xUnbounded_real_T();

  // Initialize function input argument 'Vw'.
  Vw = argInit_3xUnbounded_real_T();

  // Initialize function input argument 'Pw'.
  Pw = argInit_3xUnbounded_real_T();

  // Call the entry-point 'PnL'.
  PnL(xs, xe, Vw, Pw, argInit_boolean_T(), rot_cw_data, rot_cw_size, pos_cw,
      &minimalReprojectionError);
  emxDestroyArray_real_T(Pw);
  emxDestroyArray_real_T(Vw);
  emxDestroyArray_real_T(xe);
  emxDestroyArray_real_T(xs);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_R_and_T()
{
  emxArray_real_T *xs;
  emxArray_real_T *xe;
  emxArray_real_T *P1w;
  emxArray_real_T *P2w;
  double dv4[9];
  double dv5[3];
  double Pos_cw[3];
  double Rot_cw[9];

  // Initialize function 'R_and_T' input arguments.
  // Initialize function input argument 'xs'.
  xs = argInit_3xUnbounded_real_T();

  // Initialize function input argument 'xe'.
  xe = argInit_3xUnbounded_real_T();

  // Initialize function input argument 'P1w'.
  P1w = argInit_3xUnbounded_real_T();

  // Initialize function input argument 'P2w'.
  P2w = argInit_3xUnbounded_real_T();

  // Initialize function input argument 'initRot_cw'.
  // Initialize function input argument 'initPos_cw'.
  // Call the entry-point 'R_and_T'.
  argInit_3x3_real_T(dv4);
  argInit_3x1_real_T(dv5);
  R_and_T(xs, xe, P1w, P2w, dv4, dv5, argInit_real_T(), argInit_real_T(), Rot_cw,
          Pos_cw);
  emxDestroyArray_real_T(P2w);
  emxDestroyArray_real_T(P1w);
  emxDestroyArray_real_T(xe);
  emxDestroyArray_real_T(xs);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_calc_PnL()
{
  // Call the entry-point 'calc_PnL'.
  calc_PnL();
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  calc_PnL_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_calc_PnL();
  main_PnL();
  main_R_and_T();

  // Terminate the application.
  // You do not need to do this more than one time.
  calc_PnL_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
