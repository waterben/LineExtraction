/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_calc_PnL_api.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 22-Jul-2015 17:51:38
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_calc_PnL_api.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true, false, 131418U, NULL, "calc_PnL", NULL,
  false, { 2045744189U, 2170104910U, 2743257031U, 4284093946U }, NULL };

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static const mxArray *b_emlrt_marshallOut(const real_T u[3]);
static boolean_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *autoChooseLines, const char_T *identifier);
static const mxArray *c_emlrt_marshallOut(const real_T u);
static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static const mxArray *d_emlrt_marshallOut(const real_T u[9]);
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *initRot_cw, const char_T *identifier))[9];
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *xs, const
  char_T *identifier, emxArray_real_T *y);
static const mxArray *emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2]);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9];
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *initPos_cw, const char_T *identifier))[3];
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *maxIterNum,
  const char_T *identifier);
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static boolean_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9];
static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static real_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                emxArray_real_T *y
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  k_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real_T u[3])
{
  const mxArray *y;
  static const int32_T iv1[1] = { 0 };

  const mxArray *m1;
  static const int32_T iv2[1] = { 3 };

  y = NULL;
  m1 = emlrtCreateNumericArray(1, iv1, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, *(int32_T (*)[1])&iv2[0], 1);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *autoChooseLines
 *                const char_T *identifier
 * Return Type  : boolean_T
 */
static boolean_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *autoChooseLines, const char_T *identifier)
{
  boolean_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(autoChooseLines), &thisId);
  emlrtDestroyArray(&autoChooseLines);
  return y;
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m2;
  y = NULL;
  m2 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : boolean_T
 */
static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real_T u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(const real_T u[9])
{
  const mxArray *y;
  static const int32_T iv3[2] = { 0, 0 };

  const mxArray *m3;
  static const int32_T iv4[2] = { 3, 3 };

  y = NULL;
  m3 = emlrtCreateNumericArray(2, iv3, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m3, (void *)u);
  emlrtSetDimensions((mxArray *)m3, *(int32_T (*)[2])&iv4[0], 2);
  emlrtAssign(&y, m3);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *initRot_cw
 *                const char_T *identifier
 * Return Type  : real_T (*)[9]
 */
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *initRot_cw, const char_T *identifier))[9]
{
  real_T (*y)[9];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(initRot_cw), &thisId);
  emlrtDestroyArray(&initRot_cw);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *xs
 *                const char_T *identifier
 *                emxArray_real_T *y
 * Return Type  : void
 */
  static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *xs, const
  char_T *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  b_emlrt_marshallIn(sp, emlrtAlias(xs), &thisId, y);
  emlrtDestroyArray(&xs);
}

/*
 * Arguments    : const real_T u_data[]
 *                const int32_T u_size[2]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2])
{
  const mxArray *y;
  static const int32_T iv0[2] = { 0, 0 };

  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u_data);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[2])&u_size[0], 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((void *)(*pEmxArray)->data);
    }

    emlrtFreeMex((void *)(*pEmxArray)->size);
    emlrtFreeMex((void *)*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                emxArray_real_T **pEmxArray
 *                int32_T numDimensions
 *                boolean_T doPush
 * Return Type  : void
 */
static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocMex(sizeof(emxArray_real_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(sp, (void *)pEmxArray, (void (*)(void *))
      emxFree_real_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex((uint32_T)(sizeof(int32_T)
    * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[9]
 */
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9]
{
  real_T (*y)[9];
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *initPos_cw
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
  static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *initPos_cw, const char_T *identifier))[3]
{
  real_T (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = h_emlrt_marshallIn(sp, emlrtAlias(initPos_cw), &thisId);
  emlrtDestroyArray(&initPos_cw);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3]
{
  real_T (*y)[3];
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *maxIterNum
 *                const char_T *identifier
 * Return Type  : real_T
 */
  static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *maxIterNum, const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = j_emlrt_marshallIn(sp, emlrtAlias(maxIterNum), &thisId);
  emlrtDestroyArray(&maxIterNum);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                emxArray_real_T *ret
 * Return Type  : void
 */
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  int32_T iv5[2];
  int32_T i0;
  int32_T iv6[2];
  boolean_T bv0[2] = { false, true };

  for (i0 = 0; i0 < 2; i0++) {
    iv5[i0] = 3 + -4 * i0;
  }

  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv5, &bv0[0],
    iv6);
  ret->size[0] = iv6[0];
  ret->size[1] = iv6[1];
  ret->allocatedSize = ret->size[0] * ret->size[1];
  ret->data = (real_T *)mxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : boolean_T
 */
static boolean_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "logical", false, 0U, 0);
  ret = *mxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[9]
 */
static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9]
{
  real_T (*ret)[9];
  int32_T iv7[2];
  int32_T i;
  for (i = 0; i < 2; i++) {
    iv7[i] = 3;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv7);
  ret = (real_T (*)[9])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3]
 */
  static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  real_T (*ret)[3];
  int32_T iv8[1];
  iv8[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv8);
  ret = (real_T (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray *prhs[5]
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
void PnL_api(const mxArray *prhs[5], const mxArray *plhs[3])
{
  real_T (*pos_cw)[3];
  real_T (*rot_cw_data)[9];
  emxArray_real_T *xs;
  emxArray_real_T *xe;
  emxArray_real_T *Vw;
  emxArray_real_T *Pw;
  boolean_T autoChooseLines;
  real_T minimalReprojectionError;
  int32_T rot_cw_size[2];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  pos_cw = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));
  rot_cw_data = (real_T (*)[9])mxMalloc(sizeof(real_T [9]));
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &xs, 2, true);
  emxInit_real_T(&st, &xe, 2, true);
  emxInit_real_T(&st, &Vw, 2, true);
  emxInit_real_T(&st, &Pw, 2, true);
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[3] = emlrtProtectR2012b(prhs[3], 3, false, -1);

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "xs", xs);
  emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "xe", xe);
  emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "Vw", Vw);
  emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "Pw", Pw);
  autoChooseLines = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]),
    "autoChooseLines");

  /* Invoke the target function */
  PnL(xs, xe, Vw, Pw, autoChooseLines, *rot_cw_data, rot_cw_size, *pos_cw,
      &minimalReprojectionError);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*rot_cw_data, rot_cw_size);
  plhs[1] = b_emlrt_marshallOut(*pos_cw);
  plhs[2] = c_emlrt_marshallOut(minimalReprojectionError);
  Pw->canFreeData = false;
  emxFree_real_T(&Pw);
  Vw->canFreeData = false;
  emxFree_real_T(&Vw);
  xe->canFreeData = false;
  emxFree_real_T(&xe);
  xs->canFreeData = false;
  emxFree_real_T(&xs);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/*
 * Arguments    : const mxArray *prhs[8]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void R_and_T_api(const mxArray *prhs[8], const mxArray *plhs[2])
{
  real_T (*Rot_cw)[9];
  real_T (*Pos_cw)[3];
  emxArray_real_T *xs;
  emxArray_real_T *xe;
  emxArray_real_T *P1w;
  emxArray_real_T *P2w;
  real_T (*initRot_cw)[9];
  real_T (*initPos_cw)[3];
  real_T maxIterNum;
  real_T TerminateTh;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  Rot_cw = (real_T (*)[9])mxMalloc(sizeof(real_T [9]));
  Pos_cw = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &xs, 2, true);
  emxInit_real_T(&st, &xe, 2, true);
  emxInit_real_T(&st, &P1w, 2, true);
  emxInit_real_T(&st, &P2w, 2, true);
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[3] = emlrtProtectR2012b(prhs[3], 3, false, -1);
  prhs[4] = emlrtProtectR2012b(prhs[4], 4, false, -1);
  prhs[5] = emlrtProtectR2012b(prhs[5], 5, false, -1);

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "xs", xs);
  emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "xe", xe);
  emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "P1w", P1w);
  emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "P2w", P2w);
  initRot_cw = e_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "initRot_cw");
  initPos_cw = g_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "initPos_cw");
  maxIterNum = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "maxIterNum");
  TerminateTh = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "TerminateTh");

  /* Invoke the target function */
  R_and_T(xs, xe, P1w, P2w, *initRot_cw, *initPos_cw, maxIterNum, TerminateTh,
          *Rot_cw, *Pos_cw);

  /* Marshall function outputs */
  plhs[0] = d_emlrt_marshallOut(*Rot_cw);
  plhs[1] = b_emlrt_marshallOut(*Pos_cw);
  P2w->canFreeData = false;
  emxFree_real_T(&P2w);
  P1w->canFreeData = false;
  emxFree_real_T(&P1w);
  xe->canFreeData = false;
  emxFree_real_T(&xe);
  xs->canFreeData = false;
  emxFree_real_T(&xs);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calc_PnL_api(void)
{
  /* Invoke the target function */
  calc_PnL();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calc_PnL_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  calc_PnL_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calc_PnL_initialize(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calc_PnL_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_calc_PnL_api.c
 *
 * [EOF]
 */
