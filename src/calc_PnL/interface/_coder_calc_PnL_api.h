/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_calc_PnL_api.h
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 22-Jul-2015 17:51:38
 */

#ifndef ___CODER_CALC_PNL_API_H__
#define ___CODER_CALC_PNL_API_H__

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_calc_PnL_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void PnL(emxArray_real_T *xs, emxArray_real_T *xe, emxArray_real_T *Vw,
                emxArray_real_T *Pw, boolean_T autoChooseLines, real_T
                rot_cw_data[], int32_T rot_cw_size[2], real_T pos_cw[3], real_T *
                minimalReprojectionError);
extern void PnL_api(const mxArray *prhs[5], const mxArray *plhs[3]);
extern void R_and_T(emxArray_real_T *xs, emxArray_real_T *xe, emxArray_real_T
                    *P1w, emxArray_real_T *P2w, real_T initRot_cw[9], real_T
                    initPos_cw[3], real_T maxIterNum, real_T TerminateTh, real_T
                    Rot_cw[9], real_T Pos_cw[3]);
extern void R_and_T_api(const mxArray *prhs[8], const mxArray *plhs[2]);
extern void calc_PnL(void);
extern void calc_PnL_api(void);
extern void calc_PnL_atexit(void);
extern void calc_PnL_initialize(void);
extern void calc_PnL_terminate(void);
extern void calc_PnL_xil_terminate(void);

#endif

/*
 * File trailer for _coder_calc_PnL_api.h
 *
 * [EOF]
 */
