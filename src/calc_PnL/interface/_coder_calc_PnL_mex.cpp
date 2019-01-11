/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_calc_PnL_mex.cpp
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 22-Jul-2015 17:51:38
 */

/* Include Files */
#include "_coder_calc_PnL_api.h"
#include "_coder_calc_PnL_mex.h"

/* Variable Definitions */
static const char * emlrtEntryPoints[3] = { "calc_PnL", "PnL", "R_and_T" };

/* Function Declarations */
static void PnL_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs, const
  mxArray *prhs[5]);
static void R_and_T_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
  const mxArray *prhs[8]);
static void calc_PnL_mexFunction(int32_T nlhs, int32_T nrhs);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                const mxArray *plhs[3]
 *                int32_T nrhs
 *                const mxArray *prhs[5]
 * Return Type  : void
 */
static void PnL_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs, const
  mxArray *prhs[5])
{
  int32_T n;
  const mxArray *inputs[5];
  const mxArray *outputs[3];
  int32_T b_nlhs;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 5, 4, 3,
                        "PnL");
  }

  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 3,
                        "PnL");
  }

  /* Temporary copy for mex inputs. */
  for (n = 0; n < nrhs; n++) {
    inputs[n] = prhs[n];
  }

  /* Call the function. */
  PnL_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  calc_PnL_terminate();
}

/*
 * Arguments    : int32_T nlhs
 *                const mxArray *plhs[2]
 *                int32_T nrhs
 *                const mxArray *prhs[8]
 * Return Type  : void
 */
static void R_and_T_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
  const mxArray *prhs[8])
{
  int32_T n;
  const mxArray *inputs[8];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 8, 4, 7,
                        "R_and_T");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 7,
                        "R_and_T");
  }

  /* Temporary copy for mex inputs. */
  for (n = 0; n < nrhs; n++) {
    inputs[n] = prhs[n];
  }

  /* Call the function. */
  R_and_T_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  calc_PnL_terminate();
}

/*
 * Arguments    : int32_T nlhs
 *                int32_T nrhs
 * Return Type  : void
 */
static void calc_PnL_mexFunction(int32_T nlhs, int32_T nrhs)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 0, 4, 8,
                        "calc_PnL");
  }

  if (nlhs > 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 8,
                        "calc_PnL");
  }

  /* Call the function. */
  calc_PnL_api();

  /* Module termination. */
  calc_PnL_terminate();
}

/*
 * Arguments    : int32_T nlhs
 *                const mxArray * const plhs[]
 *                int32_T nrhs
 *                const mxArray * const prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(calc_PnL_atexit);

  /* Module initialization. */
  calc_PnL_initialize();

  /* Dispatch the entry-point. */
  switch (emlrtGetEntryPointIndex(nrhs, prhs, emlrtEntryPoints, 3)) {
   case 0:
    calc_PnL_mexFunction(nlhs, nrhs - 1);
    break;

   case 1:
    PnL_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 2:
    R_and_T_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  }
}

/*
 * File trailer for _coder_calc_PnL_mex.cpp
 *
 * [EOF]
 */
