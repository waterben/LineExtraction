//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PnL.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "norm.h"
#include "calcampose.h"
#include "svd.h"
#include "sign.h"
#include "polyval.h"
#include "abs.h"
#include "roots.h"
#include "calc_PnL_emxutil.h"
#include <slam/pose_estimator.hpp>
#include <iostream>

// Function Declarations
static void b_eml_null_assignment(double x_data[], int x_size[1], const
  boolean_T idx_data[], const int idx_size[1]);
static void eml_null_assignment(creal_T x_data[], int x_size[1], const boolean_T
  idx_data[], const int idx_size[1]);
static double rt_powd_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double x_data[]
//                int x_size[1]
//                const boolean_T idx_data[]
//                const int idx_size[1]
// Return Type  : void
//
static void b_eml_null_assignment(double x_data[], int x_size[1], const
  boolean_T idx_data[], const int idx_size[1])
{
  int nxin;
  int k0;
  int k;
  int nxout;
  double b_x_data[15];
  nxin = x_size[0];
  k0 = 0;
  for (k = 1; k <= idx_size[0]; k++) {
    k0 += idx_data[k - 1];
  }

  nxout = x_size[0] - k0;
  k0 = -1;
  for (k = 1; k <= nxin; k++) {
    if ((k > idx_size[0]) || (!idx_data[k - 1])) {
      k0++;
      x_data[k0] = x_data[k - 1];
    }
  }

  if (x_size[0] != 1) {
    if (1 > nxout) {
      k0 = 0;
    } else {
      k0 = nxout;
    }

    for (nxout = 0; nxout < k0; nxout++) {
      b_x_data[nxout] = x_data[nxout];
    }

    x_size[0] = k0;
    for (nxout = 0; nxout < k0; nxout++) {
      x_data[nxout] = b_x_data[nxout];
    }
  } else if (1 > nxout) {
    x_size[0] = 0;
  } else {
    x_size[0] = nxout;
  }
}

//
// Arguments    : creal_T x_data[]
//                int x_size[1]
//                const boolean_T idx_data[]
//                const int idx_size[1]
// Return Type  : void
//
static void eml_null_assignment(creal_T x_data[], int x_size[1], const boolean_T
  idx_data[], const int idx_size[1])
{
  int nxin;
  int k0;
  int k;
  int nxout;
  creal_T b_x_data[15];
  nxin = x_size[0];
  k0 = 0;
  for (k = 1; k <= idx_size[0]; k++) {
    k0 += idx_data[k - 1];
  }

  nxout = x_size[0] - k0;
  k0 = -1;
  for (k = 1; k <= nxin; k++) {
    if ((k > idx_size[0]) || (!idx_data[k - 1])) {
      k0++;
      x_data[k0] = x_data[k - 1];
    }
  }

  if (x_size[0] != 1) {
    if (1 > nxout) {
      k0 = 0;
    } else {
      k0 = nxout;
    }

    for (nxout = 0; nxout < k0; nxout++) {
      b_x_data[nxout] = x_data[nxout];
    }

    x_size[0] = k0;
    for (nxout = 0; nxout < k0; nxout++) {
      x_data[nxout] = b_x_data[nxout];
    }
  } else if (1 > nxout) {
    x_size[0] = 0;
  } else {
    x_size[0] = nxout;
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d5;
  double d6;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d5 = fabs(u0);
    d6 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d5 == 1.0) {
        y = rtNaN;
      } else if (d5 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d6 == 0.0) {
      y = 1.0;
    } else if (d6 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// This function follows the framework of our RPnL algorithm:
//  Robust and Efficient Pose Estimation from Line Correspondences, ACCV2012.
// input: xs(:, i) = the start point of the ith image line [startpointx, startpointy, 1];
//        xe(:, i) = the end point of the ith image line   [endpointx,   endpointy, 1];
//        Vw(:, i) = the direction of ith line in the world frame
//        Pw(:, i) = a point of ith line in the world frame
// output: rot_cw = the orientation of camera in rotation matrix parametrization
//                  (V_w = rot_cw * V_c)
//         pos_cw = the position of camera in global frame;
//                  (P_w = rot_cw * P_c + pos_cw;
// Arguments    : emxArray_real_T *xs
//                emxArray_real_T *xe
//                emxArray_real_T *Vw
//                emxArray_real_T *Pw
//                boolean_T autoChooseLines
//                double rot_cw_data[]
//                int rot_cw_size[2]
//                double pos_cw[3]
//                double *minimalReprojectionError
// Return Type  : void
//
void PnL(emxArray_real_T *xs, emxArray_real_T *xe, emxArray_real_T *Vw,
         emxArray_real_T *Pw, boolean_T autoChooseLines, double rot_cw_data[],
         int rot_cw_size[2], double pos_cw[3], double *minimalReprojectionError)
{
  int n;
  int optimumrot_cw_size_idx_0;
  int optimumrot_cw_size_idx_1;
  double optimumpos_cw[3];
  int i;
  emxArray_real_T *lineLenVec;
  int ix;
  int ixstart;
  int LineID;
  double l1[3];
  double mtmp;
  int HowToChooseFixedTwoLines;
  emxArray_real_T *nc_bar;
  emxArray_real_T *Vw_bar;
  emxArray_real_T *n_c;
  emxArray_real_T *Mat;
  emxArray_real_T *Pc_new;
  emxArray_real_T *Pc2_new;
  emxArray_real_T *UMat;
  emxArray_real_T *SMat;
  emxArray_real_T *b_Pc_new;
  emxArray_real_T *b_Pw;
  boolean_T guard1 = false;
  int32_T exitg1;
  double b_xs[3];
  int b_n;
  int itmp;
  boolean_T exitg5;
  double temp[3];
  boolean_T exitg3;
  boolean_T exitg4;
  static const double dv2[3] = { 1.1, 0.9659, 0.866 };

  double nc1[3];
  double Xm[3];
  double Rot[9];
  double b_temp;
  double sinpsi;
  static const signed char iv5[3] = { 1, 0, 0 };

  double Rx[9];
  double b_Rx[9];
  double A2;
  double B2;
  double C2;
  double x2;
  double y2;
  double z2;
  double coef[9];
  double polyDF[16];
  double u11;
  double u12;
  double u13;
  double u14;
  double u15;
  double u21;
  double u22;
  double u23;
  double u24;
  double u25;
  double u31;
  double u32;
  double u33;
  double u34;
  double u35;
  double u36;
  double a4;
  double a3;
  double a2;
  double a1;
  double a0;
  double b3;
  double b2;
  double b1;
  double b0;
  double d0;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double b_a4[9];
  int rs_size[1];
  creal_T rs_data[15];
  double b_rs_data[15];
  int b_rs_size[1];
  int minRoots_size[1];
  double minRoots_data[15];
  boolean_T exitg2;
  int c_rs_size[1];
  boolean_T b_minRoots_data[15];
  int b_minRoots_size[1];
  double PolyVal_data[15];
  int PolyVal_size[1];
  double b_d3;
  double sinphi;
  double Rz[9];
  static const signed char iv6[3] = { 0, 0, 1 };

  double RzRxRot[9];
  double b_nc1;
  double VMat[36];
  double vec[6];
  double normalizeTheta;
  double costheta;
  double sintheta;
  static const signed char iv7[3] = { 0, 1, 0 };

  double b_costheta[9];
  double c_costheta[9];
  double conditionErr;
  double numLineInFrontofCamera;
  double b_RzRxRot[3];
  double reprojectionError;
  double c_temp[3];
  double optimumrot_cw_data[9];
  n = xs->size[1];
  rot_cw_size[0] = 3;
  rot_cw_size[1] = 3;
  memset(&rot_cw_data[0], 0, 9U * sizeof(double));
  *minimalReprojectionError = 9.007199254740992E+15;
  optimumrot_cw_size_idx_0 = 0;
  optimumrot_cw_size_idx_1 = 0;
  for (i = 0; i < 3; i++) {
    pos_cw[i] = 0.0;
    optimumpos_cw[i] = 0.0;
  }

  emxInit_real_T(&lineLenVec, 1);
  ix = lineLenVec->size[0];
  lineLenVec->size[0] = xs->size[1];
  emxEnsureCapacity((emxArray__common *)lineLenVec, ix, (int)sizeof(double));
  ixstart = xs->size[1];
  for (ix = 0; ix < ixstart; ix++) {
    lineLenVec->data[ix] = 0.0;
  }

  //  added to satisfy Matlab Coder:
  LineID = -2;
  for (ix = 0; ix < 3; ix++) {
    l1[ix] = xs->data[ix] - xe->data[ix];
  }

  mtmp = norm(l1);
  for (ix = 0; ix < 3; ix++) {
    l1[ix] /= mtmp;
  }

  HowToChooseFixedTwoLines = 1;
  b_emxInit_real_T(&nc_bar, 2);
  b_emxInit_real_T(&Vw_bar, 2);
  b_emxInit_real_T(&n_c, 2);
  b_emxInit_real_T(&Mat, 2);
  b_emxInit_real_T(&Pc_new, 2);
  b_emxInit_real_T(&Pc2_new, 2);
  b_emxInit_real_T(&UMat, 2);
  b_emxInit_real_T(&SMat, 2);
  b_emxInit_real_T(&b_Pc_new, 2);
  b_emxInit_real_T(&b_Pw, 2);
  guard1 = false;
  do {
    exitg1 = 0;
    if (HowToChooseFixedTwoLines - 1 < 3) {
      if (autoChooseLines) {
        if (HowToChooseFixedTwoLines == 1) {

          // Calculate line length
          for (i = 0; i < n; i++) {
            for (ix = 0; ix < 3; ix++) {
              b_xs[ix] = xs->data[ix + xs->size[0] * i] - xe->data[ix + xe->
                size[0] * i];
            }

            lineLenVec->data[i] = norm(b_xs);
          }

          // choose the line with longest length in the image plane;
          ixstart = 1;
          b_n = lineLenVec->size[0];
          mtmp = lineLenVec->data[0];
          itmp = 0;
          if (lineLenVec->size[0] > 1) {
            if (rtIsNaN(lineLenVec->data[0])) {
              ix = 1;
              exitg5 = false;
              while ((!exitg5) && (ix + 1 <= b_n)) {
                ixstart = ix + 1;
                if (!rtIsNaN(lineLenVec->data[ix])) {
                  mtmp = lineLenVec->data[ix];
                  itmp = ix;
                  exitg5 = true;
                } else {
                  ix++;
                }
              }
            }

            if (ixstart < lineLenVec->size[0]) {
              while (ixstart + 1 <= b_n) {
                if (lineLenVec->data[ixstart] > mtmp) {
                  mtmp = lineLenVec->data[ixstart];
                  itmp = ixstart;
                }

                ixstart++;
              }
            }
          }

          LineID = itmp;
          for (ix = 0; ix < 3; ix++) {
            temp[ix] = xs->data[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            b_xs[ix] = xs->data[ix + xs->size[0] * itmp];
          }

          for (ix = 0; ix < 3; ix++) {
            xs->data[ix] = b_xs[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            xs->data[ix + xs->size[0] * itmp] = temp[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            temp[ix] = xe->data[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            b_xs[ix] = xe->data[ix + xe->size[0] * itmp];
          }

          for (ix = 0; ix < 3; ix++) {
            xe->data[ix] = b_xs[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            xe->data[ix + xe->size[0] * itmp] = temp[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            temp[ix] = Vw->data[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            b_xs[ix] = Vw->data[ix + Vw->size[0] * itmp];
          }

          for (ix = 0; ix < 3; ix++) {
            Vw->data[ix] = b_xs[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            Vw->data[ix + Vw->size[0] * itmp] = temp[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            temp[ix] = Pw->data[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            b_xs[ix] = Pw->data[ix + Pw->size[0] * itmp];
          }

          for (ix = 0; ix < 3; ix++) {
            Pw->data[ix] = b_xs[ix];
          }

          for (ix = 0; ix < 3; ix++) {
            Pw->data[ix + Pw->size[0] * itmp] = temp[ix];
          }

          lineLenVec->data[itmp] = lineLenVec->data[0];
          lineLenVec->data[0] = 0.0;
          for (ix = 0; ix < 3; ix++) {
            l1[ix] = xs->data[ix] - xe->data[ix];
          }

          mtmp = norm(l1);
          for (ix = 0; ix < 3; ix++) {
            l1[ix] /= mtmp;
          }
        }

        // first line is fixed. Find the second line
        i = 0;
        exitg3 = false;
        while ((!exitg3) && (i <= n - 2)) {
          ixstart = 1;
          b_n = lineLenVec->size[0];
          mtmp = lineLenVec->data[0];
          itmp = 0;
          if (lineLenVec->size[0] > 1) {
            if (rtIsNaN(lineLenVec->data[0])) {
              ix = 1;
              exitg4 = false;
              while ((!exitg4) && (ix + 1 <= b_n)) {
                ixstart = ix + 1;
                if (!rtIsNaN(lineLenVec->data[ix])) {
                  mtmp = lineLenVec->data[ix];
                  itmp = ix;
                  exitg4 = true;
                } else {
                  ix++;
                }
              }
            }

            if (ixstart < lineLenVec->size[0]) {
              while (ixstart + 1 <= b_n) {
                if (lineLenVec->data[ixstart] > mtmp) {
                  mtmp = lineLenVec->data[ixstart];
                  itmp = ixstart;
                }

                ixstart++;
              }
            }
          }

          LineID = itmp;

          // the current lonest line
          for (ix = 0; ix < 3; ix++) {
            temp[ix] = xs->data[ix + xs->size[0] * itmp] - xe->data[ix +
              xe->size[0] * itmp];
          }

          mtmp = norm(temp);
          for (ix = 0; ix < 3; ix++) {
            temp[ix] /= mtmp;
          }

          lineLenVec->data[itmp] = 0.0;
          mtmp = 0.0;
          for (ix = 0; ix < 3; ix++) {
            mtmp += l1[ix] * temp[ix];
          }

          if (fabs(mtmp) < dv2[HowToChooseFixedTwoLines - 1]) {
            //  0<angle<180, 15<angle<165,or 30<angle<150
            exitg3 = true;
          } else {
            i++;
          }
        }

        for (ix = 0; ix < 3; ix++) {
          temp[ix] = xs->data[ix + xs->size[0]];
        }

        for (ix = 0; ix < 3; ix++) {
          b_xs[ix] = xs->data[ix + xs->size[0] * LineID];
        }

        for (ix = 0; ix < 3; ix++) {
          xs->data[ix + xs->size[0]] = b_xs[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          xs->data[ix + xs->size[0] * LineID] = temp[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          temp[ix] = xe->data[ix + xe->size[0]];
        }

        for (ix = 0; ix < 3; ix++) {
          b_xs[ix] = xe->data[ix + xe->size[0] * LineID];
        }

        for (ix = 0; ix < 3; ix++) {
          xe->data[ix + xe->size[0]] = b_xs[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          xe->data[ix + xe->size[0] * LineID] = temp[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          temp[ix] = Vw->data[ix + Vw->size[0]];
        }

        for (ix = 0; ix < 3; ix++) {
          b_xs[ix] = Vw->data[ix + Vw->size[0] * LineID];
        }

        for (ix = 0; ix < 3; ix++) {
          Vw->data[ix + Vw->size[0]] = b_xs[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          Vw->data[ix + Vw->size[0] * LineID] = temp[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          temp[ix] = Pw->data[ix + Pw->size[0]];
        }

        for (ix = 0; ix < 3; ix++) {
          b_xs[ix] = Pw->data[ix + Pw->size[0] * LineID];
        }

        for (ix = 0; ix < 3; ix++) {
          Pw->data[ix + Pw->size[0]] = b_xs[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          Pw->data[ix + Pw->size[0] * LineID] = temp[ix];
        }

        lineLenVec->data[LineID] = lineLenVec->data[1];
        lineLenVec->data[1] = 0.0;
      }

      //  The rotation matrix R_wc is decomposed in way which is slightly different from the description in the paper, 
      //  but the framework is the same.
      //  R_wc = (Rot') * R * Rot =  (Rot') * (Ry(theta) * Rz(phi) * Rx(psi)) * Rot 
      nc1[0] = xs->data[1] * xe->data[2] - xs->data[2] * xe->data[1];
      nc1[1] = xs->data[2] * xe->data[0] - xs->data[0] * xe->data[2];
      nc1[2] = xs->data[0] * xe->data[1] - xs->data[1] * xe->data[0];
      mtmp = norm(nc1);
      for (ix = 0; ix < 3; ix++) {
        nc1[ix] /= mtmp;
      }

      mtmp = norm(*(double (*)[3])&Vw->data[0]);
      for (ix = 0; ix < 3; ix++) {
        temp[ix] = Vw->data[ix] / mtmp;
      }

      Xm[0] = nc1[1] * temp[2] - nc1[2] * temp[1];
      Xm[1] = nc1[2] * temp[0] - nc1[0] * temp[2];
      Xm[2] = nc1[0] * temp[1] - nc1[1] * temp[0];
      mtmp = norm(Xm);
      for (ix = 0; ix < 3; ix++) {
        Xm[ix] /= mtmp;
      }

      // the X axis of Model frame
      // the Y axis of Model frame
      temp[0] = Xm[1] * nc1[2] - Xm[2] * nc1[1];
      temp[1] = Xm[2] * nc1[0] - Xm[0] * nc1[2];
      temp[2] = Xm[0] * nc1[1] - Xm[1] * nc1[0];
      mtmp = norm(temp);
      for (ix = 0; ix < 3; ix++) {
        b_temp = temp[ix] / mtmp;

        // the Z axis of Model frame;
        Rot[3 * ix] = Xm[ix];
        Rot[1 + 3 * ix] = nc1[ix];
        Rot[2 + 3 * ix] = b_temp;
        temp[ix] = b_temp;
      }

      //  Rot * [Xm, Ym, Zm] = I.
      // rotate all the vector by Rot.
      ix = nc_bar->size[0] * nc_bar->size[1];
      nc_bar->size[0] = 3;
      nc_bar->size[1] = n;
      emxEnsureCapacity((emxArray__common *)nc_bar, ix, (int)sizeof(double));
      ixstart = 3 * n;
      for (ix = 0; ix < ixstart; ix++) {
        nc_bar->data[ix] = 0.0;
      }

      //  nc_bar(:,i) = Rot * nc(:,i)
      ix = Vw_bar->size[0] * Vw_bar->size[1];
      Vw_bar->size[0] = 3;
      Vw_bar->size[1] = n;
      emxEnsureCapacity((emxArray__common *)Vw_bar, ix, (int)sizeof(double));
      ixstart = 3 * n;
      for (ix = 0; ix < ixstart; ix++) {
        Vw_bar->data[ix] = 0.0;
      }

      //  Vw_bar(:,i) = Rot * Vw(:,i)
      ix = n_c->size[0] * n_c->size[1];
      n_c->size[0] = 3;
      n_c->size[1] = n;
      emxEnsureCapacity((emxArray__common *)n_c, ix, (int)sizeof(double));
      ixstart = 3 * n;
      for (ix = 0; ix < ixstart; ix++) {
        n_c->data[ix] = 0.0;
      }

      for (i = 0; i < n; i++) {
        nc1[0] = xs->data[1 + xs->size[0] * i] * xe->data[2 + xe->size[0] * i] -
          xs->data[2 + xs->size[0] * i] * xe->data[1 + xe->size[0] * i];
        nc1[1] = xs->data[2 + xs->size[0] * i] * xe->data[xe->size[0] * i] -
          xs->data[xs->size[0] * i] * xe->data[2 + xe->size[0] * i];
        nc1[2] = xs->data[xs->size[0] * i] * xe->data[1 + xe->size[0] * i] -
          xs->data[1 + xs->size[0] * i] * xe->data[xe->size[0] * i];
        mtmp = norm(nc1);
        for (ix = 0; ix < 3; ix++) {
          nc1[ix] /= mtmp;
        }

        for (ix = 0; ix < 3; ix++) {
          n_c->data[ix + n_c->size[0] * i] = nc1[ix];
        }

        for (ix = 0; ix < 3; ix++) {
          nc_bar->data[ix + nc_bar->size[0] * i] = 0.0;
          for (itmp = 0; itmp < 3; itmp++) {
            nc_bar->data[ix + nc_bar->size[0] * i] += Rot[ix + 3 * itmp] *
              nc1[itmp];
          }
        }

        for (ix = 0; ix < 3; ix++) {
          Vw_bar->data[ix + Vw_bar->size[0] * i] = 0.0;
          for (itmp = 0; itmp < 3; itmp++) {
            Vw_bar->data[ix + Vw_bar->size[0] * i] += Rot[ix + 3 * itmp] *
              Vw->data[itmp + Vw->size[0] * i];
          }
        }
      }

      //  Determine the angle psi, it is the angle between z axis and Vw_bar(:,1). 
      //  The rotation matrix Rx(psi) rotates Vw_bar(:,1) to z axis
      // the angle between z axis and Vw_bar(:,1); cospsi=[0,0,1] * Vw_bar(:,1);. 
      sinpsi = sqrt(1.0 - Vw_bar->data[2] * Vw_bar->data[2]);
      for (ix = 0; ix < 3; ix++) {
        Rx[3 * ix] = iv5[ix];
      }

      Rx[1] = 0.0;
      Rx[4] = Vw_bar->data[2];
      Rx[7] = -sinpsi;
      Rx[2] = 0.0;
      Rx[5] = sinpsi;
      Rx[8] = Vw_bar->data[2];

      //  should be the Z axis, i.e. [0, 0, 1]';
      for (ix = 0; ix < 3; ix++) {
        b_xs[ix] = 0.0;
        for (itmp = 0; itmp < 3; itmp++) {
          mtmp = b_xs[ix] + Rx[ix + 3 * itmp] * Vw_bar->data[itmp];
          b_xs[ix] = mtmp;
        }
      }

      if (1.0 - fabs(b_xs[2]) > 1.0E-5) {
        for (ix = 0; ix < 3; ix++) {
          for (itmp = 0; itmp < 3; itmp++) {
            b_Rx[itmp + 3 * ix] = Rx[ix + 3 * itmp];
          }
        }

        for (ix = 0; ix < 3; ix++) {
          for (itmp = 0; itmp < 3; itmp++) {
            Rx[itmp + 3 * ix] = b_Rx[itmp + 3 * ix];
          }
        }
      }

      // estimate the rotation angle phi by least square residual.
      // i.e the rotation matrix Rz(phi)
      for (ix = 0; ix < 3; ix++) {
        temp[ix] = 0.0;
        for (itmp = 0; itmp < 3; itmp++) {
          b_temp = temp[ix] + Rx[ix + 3 * itmp] * Vw_bar->data[itmp +
            Vw_bar->size[0]];
          temp[ix] = b_temp;
        }
      }

      A2 = temp[0];
      B2 = temp[1];
      C2 = temp[2];
      x2 = nc_bar->data[nc_bar->size[0]];
      y2 = nc_bar->data[1 + nc_bar->size[0]];
      z2 = nc_bar->data[2 + nc_bar->size[0]];
      memset(&coef[0], 0, 9U * sizeof(double));

      // coefficients of equation (7)
      memset(&polyDF[0], 0, sizeof(double) << 4);

      // dF = ployDF(1) * t^15 + ployDF(2) * t^14 + ... + ployDF(15) * t + ployDF(16); 
      //  construct the  polynomial F'
      for (i = 2; i - 2 <= n - 3; i++) {
        // The first two lines are included in every triplet.
        for (ix = 0; ix < 3; ix++) {
          temp[ix] = 0.0;
          for (itmp = 0; itmp < 3; itmp++) {
            b_temp = temp[ix] + Rx[ix + 3 * itmp] * Vw_bar->data[itmp +
              Vw_bar->size[0] * i];
            temp[ix] = b_temp;
          }
        }

        u11 = -z2 * A2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[1] + y2 *
          B2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[0];
        u12 = -y2 * A2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[1] + z2 *
          B2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[0];
        u13 = ((-y2 * B2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[1] + z2 *
                B2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[1]) + y2 * A2 *
               nc_bar->data[2 + nc_bar->size[0] * i] * temp[0]) - z2 * A2 *
          nc_bar->data[1 + nc_bar->size[0] * i] * temp[0];
        u14 = -y2 * B2 * nc_bar->data[nc_bar->size[0] * i] * temp[2] + x2 * C2 *
          nc_bar->data[1 + nc_bar->size[0] * i] * temp[1];
        u15 = x2 * C2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[0] - y2 *
          A2 * nc_bar->data[nc_bar->size[0] * i] * temp[2];
        u21 = -x2 * A2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[1] + y2 *
          B2 * nc_bar->data[nc_bar->size[0] * i] * temp[0];
        u22 = -y2 * A2 * nc_bar->data[nc_bar->size[0] * i] * temp[1] + x2 * B2 *
          nc_bar->data[1 + nc_bar->size[0] * i] * temp[0];
        u23 = ((x2 * B2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[1] - y2 *
                B2 * nc_bar->data[nc_bar->size[0] * i] * temp[1]) - x2 * A2 *
               nc_bar->data[1 + nc_bar->size[0] * i] * temp[0]) + y2 * A2 *
          nc_bar->data[nc_bar->size[0] * i] * temp[0];
        u24 = y2 * B2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[2] - z2 *
          C2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[1];
        u25 = y2 * A2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[2] - z2 *
          C2 * nc_bar->data[1 + nc_bar->size[0] * i] * temp[0];
        u31 = -x2 * A2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[0] + z2 *
          A2 * nc_bar->data[nc_bar->size[0] * i] * temp[0];
        u32 = -x2 * B2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[1] + z2 *
          B2 * nc_bar->data[nc_bar->size[0] * i] * temp[1];
        u33 = ((x2 * A2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[1] - z2 *
                A2 * nc_bar->data[nc_bar->size[0] * i] * temp[1]) + x2 * B2 *
               nc_bar->data[2 + nc_bar->size[0] * i] * temp[0]) - z2 * B2 *
          nc_bar->data[nc_bar->size[0] * i] * temp[0];
        u34 = ((z2 * A2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[2] + x2 *
                A2 * nc_bar->data[nc_bar->size[0] * i] * temp[2]) - z2 * C2 *
               nc_bar->data[2 + nc_bar->size[0] * i] * temp[0]) - x2 * C2 *
          nc_bar->data[nc_bar->size[0] * i] * temp[0];
        u35 = ((-z2 * B2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[2] - x2 *
                B2 * nc_bar->data[nc_bar->size[0] * i] * temp[2]) + z2 * C2 *
               nc_bar->data[2 + nc_bar->size[0] * i] * temp[1]) + x2 * C2 *
          nc_bar->data[nc_bar->size[0] * i] * temp[1];
        u36 = -x2 * C2 * nc_bar->data[2 + nc_bar->size[0] * i] * temp[2] + z2 *
          C2 * nc_bar->data[nc_bar->size[0] * i] * temp[2];
        a4 = ((((((((((u11 * u11 + u12 * u12) - u13 * u13) - 2.0 * u11 * u12) +
                    u21 * u21) + u22 * u22) - u23 * u23) - 2.0 * u21 * u22) -
                u31 * u31) - u32 * u32) + u33 * u33) + 2.0 * u31 * u32;
        a3 = 2.0 * ((((((((u11 * u14 - u13 * u15) - u12 * u14) + u21 * u24) -
                        u23 * u25) - u22 * u24) - u31 * u34) + u33 * u35) + u32 *
                    u34);
        a2 = (((((((((((((((-2.0 * u12 * u12 + u13 * u13) + u14 * u14) - u15 *
                          u15) + 2.0 * u11 * u12) - 2.0 * u22 * u22) + u23 * u23)
                      + u24 * u24) - u25 * u25) + 2.0 * u21 * u22) + 2.0 * u32 *
                   u32) - u33 * u33) - u34 * u34) + u35 * u35) - 2.0 * u31 * u32)
              - 2.0 * u31 * u36) + 2.0 * u32 * u36;
        a1 = 2.0 * ((((((u12 * u14 + u13 * u15) + u22 * u24) + u23 * u25) - u32 *
                      u34) - u33 * u35) - u34 * u36);
        a0 = ((((((u12 * u12 + u15 * u15) + u22 * u22) + u25 * u25) - u32 * u32)
               - u35 * u35) - u36 * u36) - 2.0 * u32 * u36;
        b3 = 2.0 * (((((u11 * u13 - u12 * u13) + u21 * u23) - u22 * u23) - u31 *
                     u33) + u32 * u33);
        b2 = 2.0 * ((((((((u11 * u15 - u12 * u15) + u13 * u14) + u21 * u25) -
                        u22 * u25) + u23 * u24) - u31 * u35) + u32 * u35) - u33 *
                    u34);
        b1 = 2.0 * ((((((u12 * u13 + u14 * u15) + u22 * u23) + u24 * u25) - u32 *
                      u33) - u34 * u35) - u33 * u36);
        b0 = 2.0 * (((u12 * u15 + u22 * u25) - u32 * u35) - u35 * u36);
        d0 = a0 * a0 - b0 * b0;
        d1 = 2.0 * (a0 * a1 - b0 * b1);
        d2 = (((a1 * a1 + 2.0 * a0 * a2) + b0 * b0) - b1 * b1) - 2.0 * b0 * b2;
        d3 = 2.0 * ((((a0 * a3 + a1 * a2) + b0 * b1) - b1 * b2) - b0 * b3);
        d4 = (((((a2 * a2 + 2.0 * a0 * a4) + 2.0 * a1 * a3) + b1 * b1) + 2.0 *
               b0 * b2) - b2 * b2) - 2.0 * b1 * b3;
        d5 = 2.0 * ((((a1 * a4 + a2 * a3) + b1 * b2) + b0 * b3) - b2 * b3);
        d6 = (((a3 * a3 + 2.0 * a2 * a4) + b2 * b2) - b3 * b3) + 2.0 * b1 * b3;
        d7 = 2.0 * (a3 * a4 + b2 * b3);
        d8 = a4 * a4 + b3 * b3;
        b_a4[0] = a4;
        b_a4[1] = a3;
        b_a4[2] = a2;
        b_a4[3] = a1;
        b_a4[4] = a0;
        b_a4[5] = b3;
        b_a4[6] = b2;
        b_a4[7] = b1;
        b_a4[8] = b0;
        for (ix = 0; ix < 9; ix++) {
          coef[ix] += b_a4[ix];
        }

        polyDF[0] += 8.0 * d8 * d8;
        polyDF[1] += 15.0 * d7 * d8;
        polyDF[2] = (polyDF[2] + 14.0 * d6 * d8) + 7.0 * d7 * d7;
        polyDF[3] += 13.0 * (d5 * d8 + d6 * d7);
        polyDF[4] = (polyDF[4] + 12.0 * (d4 * d8 + d5 * d7)) + 6.0 * d6 * d6;
        polyDF[5] += 11.0 * ((d3 * d8 + d4 * d7) + d5 * d6);
        polyDF[6] = (polyDF[6] + 10.0 * ((d2 * d8 + d3 * d7) + d4 * d6)) + 5.0 *
          d5 * d5;
        polyDF[7] += 9.0 * (((d1 * d8 + d2 * d7) + d3 * d6) + d4 * d5);
        polyDF[8] = ((polyDF[8] + 8.0 * ((d1 * d7 + d2 * d6) + d3 * d5)) + 4.0 *
                     d4 * d4) + 8.0 * d0 * d8;
        polyDF[9] = (polyDF[9] + 7.0 * ((d1 * d6 + d2 * d5) + d3 * d4)) + 7.0 *
          d0 * d7;
        polyDF[10] = ((polyDF[10] + 6.0 * (d1 * d5 + d2 * d4)) + 3.0 * d3 * d3)
          + 6.0 * d0 * d6;
        polyDF[11] = (polyDF[11] + 5.0 * (d1 * d4 + d2 * d3)) + 5.0 * d0 * d5;
        polyDF[12] = ((polyDF[12] + 4.0 * d1 * d3) + 2.0 * d2 * d2) + 4.0 * d0 *
          d4;
        polyDF[13] = (polyDF[13] + 3.0 * d1 * d2) + 3.0 * d0 * d3;
        polyDF[14] = (polyDF[14] + d1 * d1) + 2.0 * d0 * d2;
        polyDF[15] += d0 * d1;
      }

      // solve polyDF
      roots(polyDF, rs_data, rs_size);

      //  retriving the local minima of the cost function.
      b_rs_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (ix = 0; ix < ixstart; ix++) {
        b_rs_data[ix] = rs_data[ix].re;
      }

      b_abs(b_rs_data, b_rs_size, minRoots_data, minRoots_size);
      ixstart = 1;
      b_n = minRoots_size[0];
      mtmp = minRoots_data[0];
      if (minRoots_size[0] > 1) {
        if (rtIsNaN(minRoots_data[0])) {
          ix = 2;
          exitg2 = false;
          while ((!exitg2) && (ix <= b_n)) {
            ixstart = ix;
            if (!rtIsNaN(minRoots_data[ix - 1])) {
              mtmp = minRoots_data[ix - 1];
              exitg2 = true;
            } else {
              ix++;
            }
          }
        }

        if (ixstart < minRoots_size[0]) {
          while (ixstart + 1 <= b_n) {
            if (minRoots_data[ixstart] > mtmp) {
              mtmp = minRoots_data[ixstart];
            }

            ixstart++;
          }
        }
      }

      c_rs_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (ix = 0; ix < ixstart; ix++) {
        b_rs_data[ix] = rs_data[ix].im;
      }

      b_abs(b_rs_data, c_rs_size, minRoots_data, minRoots_size);
      b_minRoots_size[0] = minRoots_size[0];
      ixstart = minRoots_size[0];
      for (ix = 0; ix < ixstart; ix++) {
        b_minRoots_data[ix] = (minRoots_data[ix] / mtmp > 0.001);
      }

      eml_null_assignment(rs_data, rs_size, b_minRoots_data, b_minRoots_size);
      minRoots_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (ix = 0; ix < ixstart; ix++) {
        minRoots_data[ix] = rs_data[ix].re;
      }

      for (ix = 0; ix < 15; ix++) {
        b_rs_data[ix] = (15.0 - (double)ix) * polyDF[ix];
      }

      polyval(b_rs_data, minRoots_data, minRoots_size, PolyVal_data, rs_size);
      PolyVal_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (ix = 0; ix < ixstart; ix++) {
        b_minRoots_data[ix] = (PolyVal_data[ix] <= 0.0);
      }

      b_eml_null_assignment(minRoots_data, minRoots_size, b_minRoots_data,
                            PolyVal_size);
      if (minRoots_size[0] == 0) {
        exitg1 = 1;
      } else {
        // for each minimum, we try to find a solution of the camera pose, then
        // choose the one with the least reprojection residual as the optimum of the solution. 
        *minimalReprojectionError = 100.0;
        if (!autoChooseLines) {
          *minimalReprojectionError = 9.007199254740992E+15;
        }

        //  In general, there are two solutions which yields small re-projection error 
        //  or condition error:"n_c * R_wc * V_w=0". One of the solution transforms the 
        //  world scene behind the camera center, the other solution transforms the world 
        //  scene in front of camera center. While only the latter one is correct. 
        //  This can easily be checked by verifying their Z coordinates in the camera frame. 
        //  P_c(Z) must be larger than 0 if it's in front of the camera.
        for (b_n = 0; b_n < minRoots_size[0]; b_n++) {
          mtmp = (((coef[0] * rt_powd_snf(minRoots_data[b_n], 4.0) + coef[1] *
                    rt_powd_snf(minRoots_data[b_n], 3.0)) + coef[2] *
                   (minRoots_data[b_n] * minRoots_data[b_n])) + coef[3] *
                  minRoots_data[b_n]) + coef[4];
          b_sign(&mtmp);
          b_d3 = ((coef[5] * rt_powd_snf(minRoots_data[b_n], 3.0) + coef[6] *
                   (minRoots_data[b_n] * minRoots_data[b_n])) + coef[7] *
                  minRoots_data[b_n]) + coef[8];
          b_sign(&b_d3);
          sinphi = -mtmp * b_d3 * sqrt(fabs(1.0 - minRoots_data[b_n] *
            minRoots_data[b_n]));
          Rz[0] = minRoots_data[b_n];
          Rz[3] = -sinphi;
          Rz[6] = 0.0;
          Rz[1] = sinphi;
          Rz[4] = minRoots_data[b_n];
          Rz[7] = 0.0;
          for (ix = 0; ix < 3; ix++) {
            Rz[2 + 3 * ix] = iv6[ix];
          }

          // now, according to Sec4.3, we estimate the rotation angle theta
          // and the translation vector at a time.
          for (ix = 0; ix < 3; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_Rx[ix + 3 * itmp] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                b_Rx[ix + 3 * itmp] += Rz[ix + 3 * ixstart] * Rx[ixstart + 3 *
                  itmp];
              }
            }

            for (itmp = 0; itmp < 3; itmp++) {
              RzRxRot[ix + 3 * itmp] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                RzRxRot[ix + 3 * itmp] += b_Rx[ix + 3 * ixstart] * Rot[ixstart +
                  3 * itmp];
              }
            }
          }

          // According to the fact that n_i^C should be orthogonal to Pi^c and Vi^c, we  
          // have: scalarproduct(Vi^c, ni^c) = 0  and scalarproduct(Pi^c, ni^c) = 0. 
          // where Vi^c = Rwc * Vi^w,  Pi^c = Rwc *(Pi^w - pos_cw) = Rwc * Pi^w - pos; 
          // Using the above two constraints to construct linear equation system Mat about
          // [costheta, sintheta, tx, ty, tz, 1].
          ix = Mat->size[0] * Mat->size[1];
          Mat->size[0] = (int)(2.0 * (double)n - 1.0);
          Mat->size[1] = 6;
          emxEnsureCapacity((emxArray__common *)Mat, ix, (int)sizeof(double));
          ixstart = (int)(2.0 * (double)n - 1.0) * 6;
          for (ix = 0; ix < ixstart; ix++) {
            Mat->data[ix] = 0.0;
          }

          for (i = 0; i < n; i++) {
            for (ix = 0; ix < 3; ix++) {
              temp[ix] = 0.0;
              for (itmp = 0; itmp < 3; itmp++) {
                b_temp = temp[ix] + RzRxRot[ix + 3 * itmp] * Vw->data[itmp +
                  Vw->size[0] * i];
                temp[ix] = b_temp;
              }
            }

            for (ix = 0; ix < 3; ix++) {
              nc1[ix] = 0.0;
              for (itmp = 0; itmp < 3; itmp++) {
                b_nc1 = nc1[ix] + RzRxRot[ix + 3 * itmp] * Pw->data[itmp +
                  Pw->size[0] * i];
                nc1[ix] = b_nc1;
              }
            }

            //  apply the constraint scalarproduct(Vi^c, ni^c) = 0
            if (1 + i > 1) {
              // if i=1, then scalarproduct(Vi^c, ni^c) always be 0
              Mat->data[(int)((unsigned int)(1 + i) << 1) - 3] = nc_bar->
                data[nc_bar->size[0] * i] * temp[0] + nc_bar->data[2 +
                nc_bar->size[0] * i] * temp[2];
              Mat->data[((int)((unsigned int)(1 + i) << 1) + Mat->size[0]) - 3] =
                nc_bar->data[nc_bar->size[0] * i] * temp[2] - nc_bar->data[2 +
                nc_bar->size[0] * i] * temp[0];
              Mat->data[((int)((unsigned int)(1 + i) << 1) + Mat->size[0] * 5) -
                3] = nc_bar->data[1 + nc_bar->size[0] * i] * temp[1];
            }

            //  apply the constraint scalarproduct(Pi^c, ni^c) = 0
            Mat->data[(int)((unsigned int)(1 + i) << 1) - 2] = nc_bar->
              data[nc_bar->size[0] * i] * nc1[0] + nc_bar->data[2 + nc_bar->
              size[0] * i] * nc1[2];
            Mat->data[((int)((unsigned int)(1 + i) << 1) + Mat->size[0]) - 2] =
              nc_bar->data[nc_bar->size[0] * i] * nc1[2] - nc_bar->data[2 +
              nc_bar->size[0] * i] * nc1[0];
            Mat->data[((int)((unsigned int)(1 + i) << 1) + (Mat->size[0] << 1))
              - 2] = -nc_bar->data[nc_bar->size[0] * i];
            Mat->data[((int)((unsigned int)(1 + i) << 1) + Mat->size[0] * 3) - 2]
              = -nc_bar->data[1 + nc_bar->size[0] * i];
            Mat->data[((int)((unsigned int)(1 + i) << 1) + (Mat->size[0] << 2))
              - 2] = -nc_bar->data[2 + nc_bar->size[0] * i];
            Mat->data[((int)((unsigned int)(1 + i) << 1) + Mat->size[0] * 5) - 2]
              = nc_bar->data[1 + nc_bar->size[0] * i] * nc1[1];
          }

          // solve the linear system Mat * [costheta, sintheta, tx, ty, tz, 1]' = 0  using SVD,
          b_svd(Mat, UMat, SMat, VMat);

          //  the last column of Vmat;
          for (ix = 0; ix < 6; ix++) {
            vec[ix] = VMat[30 + ix] / VMat[35];
          }

          // the condition that the last element of vec should be 1.
          normalizeTheta = 1.0 / sqrt(vec[0] * vec[0] + vec[1] * vec[1]);

          // the condition costheta^2+sintheta^2 = 1;
          costheta = vec[0] * normalizeTheta;
          sintheta = vec[1] * normalizeTheta;

          // now, we get the rotation matrix rot_wc and translation pos_wc
          b_Rx[0] = costheta;
          b_Rx[3] = 0.0;
          b_Rx[6] = sintheta;
          for (ix = 0; ix < 3; ix++) {
            b_Rx[1 + 3 * ix] = iv7[ix];
          }

          b_Rx[2] = -sintheta;
          b_Rx[5] = 0.0;
          b_Rx[8] = costheta;
          for (ix = 0; ix < 3; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_costheta[ix + 3 * itmp] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                b_costheta[ix + 3 * itmp] += b_Rx[ix + 3 * ixstart] * Rz[ixstart
                  + 3 * itmp];
              }
            }

            for (itmp = 0; itmp < 3; itmp++) {
              c_costheta[ix + 3 * itmp] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                c_costheta[ix + 3 * itmp] += b_costheta[ix + 3 * ixstart] *
                  Rx[ixstart + 3 * itmp];
              }
            }
          }

          for (ix = 0; ix < 3; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_Rx[ix + 3 * itmp] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                b_Rx[ix + 3 * itmp] += Rot[ixstart + 3 * ix] *
                  c_costheta[ixstart + 3 * itmp];
              }
            }

            for (itmp = 0; itmp < 3; itmp++) {
              RzRxRot[ix + 3 * itmp] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                RzRxRot[ix + 3 * itmp] += b_Rx[ix + 3 * ixstart] * Rot[ixstart +
                  3 * itmp];
              }

              b_costheta[itmp + 3 * ix] = -Rot[ix + 3 * itmp];
            }
          }

          for (ix = 0; ix < 3; ix++) {
            Xm[ix] = 0.0;
            for (itmp = 0; itmp < 3; itmp++) {
              Xm[ix] += b_costheta[ix + 3 * itmp] * vec[2 + itmp];
            }
          }

          // now normalize the camera pose by 3D alignment. We first translate the points 
          // on line in the world frame Pw to points in the camera frame Pc. Then we project 
          // Pc onto the line interpretation plane as Pc_new. So we could call the point 
          // alignment algorithm to normalize the camera by aligning Pc_new and Pw. 
          // In order to improve the accuracy of the aligment step, we choose two points for each 
          // lines. The first point is Pwi, the second point is  the closest point on line i to camera center.  
          // (Pw2i = Pwi - (Pwi'*Vwi)*Vwi.)
          ix = Vw_bar->size[0] * Vw_bar->size[1];
          Vw_bar->size[0] = 3;
          Vw_bar->size[1] = n;
          emxEnsureCapacity((emxArray__common *)Vw_bar, ix, (int)sizeof(double));
          ixstart = 3 * n;
          for (ix = 0; ix < ixstart; ix++) {
            Vw_bar->data[ix] = 0.0;
          }

          ix = Pc_new->size[0] * Pc_new->size[1];
          Pc_new->size[0] = 3;
          Pc_new->size[1] = n;
          emxEnsureCapacity((emxArray__common *)Pc_new, ix, (int)sizeof(double));
          ixstart = 3 * n;
          for (ix = 0; ix < ixstart; ix++) {
            Pc_new->data[ix] = 0.0;
          }

          ix = Pc2_new->size[0] * Pc2_new->size[1];
          Pc2_new->size[0] = 3;
          Pc2_new->size[1] = n;
          emxEnsureCapacity((emxArray__common *)Pc2_new, ix, (int)sizeof(double));
          ixstart = 3 * n;
          for (ix = 0; ix < ixstart; ix++) {
            Pc2_new->data[ix] = 0.0;
          }

          for (i = 0; i < n; i++) {
            // first point on line i
            for (ix = 0; ix < 3; ix++) {
              mtmp = 0.0;
              for (itmp = 0; itmp < 3; itmp++) {
                mtmp += RzRxRot[ix + 3 * itmp] * Pw->data[itmp + Pw->size[0] * i];
              }

              temp[ix] = mtmp + Xm[ix];
            }

            mtmp = 0.0;
            for (ix = 0; ix < 3; ix++) {
              mtmp += temp[ix] * n_c->data[ix + n_c->size[0] * i];
            }

            for (ix = 0; ix < 3; ix++) {
              Pc_new->data[ix + Pc_new->size[0] * i] = temp[ix] - mtmp *
                n_c->data[ix + n_c->size[0] * i];
            }

            // second point is the closest point on line i to camera center.
            mtmp = 0.0;
            for (ix = 0; ix < 3; ix++) {
              mtmp += Pw->data[ix + Pw->size[0] * i] * Vw->data[ix + Vw->size[0]
                * i];
            }

            for (ix = 0; ix < 3; ix++) {
              temp[ix] = Pw->data[ix + Pw->size[0] * i] - mtmp * Vw->data[ix +
                Vw->size[0] * i];
            }

            for (ix = 0; ix < 3; ix++) {
              Vw_bar->data[ix + Vw_bar->size[0] * i] = temp[ix];
            }

            for (ix = 0; ix < 3; ix++) {
              mtmp = 0.0;
              for (itmp = 0; itmp < 3; itmp++) {
                mtmp += RzRxRot[ix + 3 * itmp] * temp[itmp];
              }

              nc1[ix] = mtmp + Xm[ix];
            }

            mtmp = 0.0;
            for (ix = 0; ix < 3; ix++) {
              mtmp += nc1[ix] * n_c->data[ix + n_c->size[0] * i];
            }

            for (ix = 0; ix < 3; ix++) {
              Pc2_new->data[ix + Pc2_new->size[0] * i] = nc1[ix] - mtmp *
                n_c->data[ix + n_c->size[0] * i];
            }
          }

          ix = b_Pc_new->size[0] * b_Pc_new->size[1];
          b_Pc_new->size[0] = 3;
          b_Pc_new->size[1] = Pc_new->size[1] + Pc2_new->size[1];
          emxEnsureCapacity((emxArray__common *)b_Pc_new, ix, (int)sizeof(double));
          ixstart = Pc_new->size[1];
          for (ix = 0; ix < ixstart; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_Pc_new->data[itmp + b_Pc_new->size[0] * ix] = Pc_new->data[itmp
                + Pc_new->size[0] * ix];
            }
          }

          ixstart = Pc2_new->size[1];
          for (ix = 0; ix < ixstart; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_Pc_new->data[itmp + b_Pc_new->size[0] * (ix + Pc_new->size[1])] =
                Pc2_new->data[itmp + Pc2_new->size[0] * ix];
            }
          }

          ix = b_Pw->size[0] * b_Pw->size[1];
          b_Pw->size[0] = 3;
          b_Pw->size[1] = Pw->size[1] + Vw_bar->size[1];
          emxEnsureCapacity((emxArray__common *)b_Pw, ix, (int)sizeof(double));
          ixstart = Pw->size[1];
          for (ix = 0; ix < ixstart; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_Pw->data[itmp + b_Pw->size[0] * ix] = Pw->data[itmp + Pw->size[0]
                * ix];
            }
          }

          ixstart = Vw_bar->size[1];
          for (ix = 0; ix < ixstart; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_Pw->data[itmp + b_Pw->size[0] * (ix + Pw->size[1])] =
                Vw_bar->data[itmp + Vw_bar->size[0] * ix];
            }
          }

          b_calcampose(b_Pc_new, b_Pw, RzRxRot, Xm);
          for (ix = 0; ix < 3; ix++) {
            for (itmp = 0; itmp < 3; itmp++) {
              b_Rx[itmp + 3 * ix] = -RzRxRot[ix + 3 * itmp];
            }
          }

          for (ix = 0; ix < 3; ix++) {
            pos_cw[ix] = 0.0;
            for (itmp = 0; itmp < 3; itmp++) {
              pos_cw[ix] += b_Rx[ix + 3 * itmp] * Xm[itmp];
            }
          }

          // check the condition n_c^T * rot_wc * V_w = 0;
          conditionErr = 0.0;
          for (i = 0; i < n; i++) {
            for (ix = 0; ix < 3; ix++) {
              b_xs[ix] = 0.0;
              for (itmp = 0; itmp < 3; itmp++) {
                mtmp = b_xs[ix] + n_c->data[itmp + n_c->size[0] * i] *
                  RzRxRot[itmp + 3 * ix];
                b_xs[ix] = mtmp;
              }
            }

            mtmp = 0.0;
            for (ix = 0; ix < 3; ix++) {
              mtmp += b_xs[ix] * Vw->data[ix + Vw->size[0] * i];
            }

            conditionErr += mtmp * mtmp;
          }

          if ((conditionErr / (double)n < 0.001) || (HowToChooseFixedTwoLines ==
               3) ){ // || (!autoChooseLines)) {
            // check whether the world scene is in front of the camera.
            numLineInFrontofCamera = 0.0;
//            if ((HowToChooseFixedTwoLines < 3) || (!autoChooseLines)) {
              for (i = 0; i < n; i++) {
                for (ix = 0; ix < 3; ix++) {
                  b_xs[ix] = Pw->data[ix + Pw->size[0] * i] - pos_cw[ix];
                }

                for (ix = 0; ix < 3; ix++) {
                  b_RzRxRot[ix] = 0.0;
                  for (itmp = 0; itmp < 3; itmp++) {
                    b_RzRxRot[ix] += RzRxRot[ix + 3 * itmp] * b_xs[itmp];
                  }
                }

                if (b_RzRxRot[2] > 0.0) {
                  numLineInFrontofCamera++;
                }
              }
//            } else {
//              numLineInFrontofCamera = n;
//            }

            if (numLineInFrontofCamera > 0.5 * (double)n) {
              // most of the lines are in front of camera, then check the reprojection error. 
              reprojectionError = 0.0;
    //          std::cout << "n: " << n << std::endl;

              for (i = 0; i < n; i++) {
                for (ix = 0; ix < 3; ix++) {
                  temp[ix] = Pw->data[ix + Pw->size[0] * i] - pos_cw[ix];
                }

                c_temp[0] = temp[1] * Vw->data[2 + Vw->size[0] * i] - temp[2] *
                  Vw->data[1 + Vw->size[0] * i];
                c_temp[1] = temp[2] * Vw->data[Vw->size[0] * i] - temp[0] *
                  Vw->data[2 + Vw->size[0] * i];
                c_temp[2] = temp[0] * Vw->data[1 + Vw->size[0] * i] - temp[1] *
                  Vw->data[Vw->size[0] * i];
                for (ix = 0; ix < 3; ix++) {
                  nc1[ix] = 0.0;
                  for (itmp = 0; itmp < 3; itmp++) {
                    nc1[ix] += RzRxRot[ix + 3 * itmp] * c_temp[itmp];
                  }
                }

                // line projection function
                mtmp = 0.0;
                for (ix = 0; ix < 3; ix++) {
                  mtmp += nc1[ix] * xs->data[ix + xs->size[0] * i];
                }

                b_d3 = 0.0;
                for (ix = 0; ix < 3; ix++) {
                  b_d3 += nc1[ix] * xe->data[ix + xe->size[0] * i];
                }

                for (ix = 0; ix < 3; ix++) {
                  b_xs[ix] = xs->data[ix + xs->size[0] * i] - xe->data[ix +
                    xe->size[0] * i];
                }
                double addition =norm(b_xs) / 3.0 * ((mtmp * mtmp + mtmp *
                                                      b_d3) + b_d3 * b_d3) / (nc1[0] * nc1[0] + nc1[1] * nc1[1]);
                reprojectionError += addition;
          //      std::cout << "addition: " << addition << std::endl;

              }

              if (reprojectionError < *minimalReprojectionError) {
                optimumrot_cw_size_idx_0 = 3;
                optimumrot_cw_size_idx_1 = 3;
                for (ix = 0; ix < 3; ix++) {
                  for (itmp = 0; itmp < 3; itmp++) {
                    optimumrot_cw_data[itmp + 3 * ix] = RzRxRot[ix + 3 * itmp];
                  }
                }

                for (i = 0; i < 3; i++) {
                  optimumpos_cw[i] = pos_cw[i];
                }

                *minimalReprojectionError = reprojectionError;

              }
            }
          }
        }

        if ((optimumrot_cw_size_idx_0 > 0) || (!autoChooseLines)) {
          guard1 = true;
          exitg1 = 1;
        } else {
          HowToChooseFixedTwoLines++;
          guard1 = false;
        }
      }
    } else {
      guard1 = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (guard1) {
    rot_cw_size[0] = optimumrot_cw_size_idx_0;
    rot_cw_size[1] = optimumrot_cw_size_idx_1;
    ixstart = optimumrot_cw_size_idx_0 * optimumrot_cw_size_idx_1;
    for (ix = 0; ix < ixstart; ix++) {
      rot_cw_data[ix] = optimumrot_cw_data[ix];
    }

    for (i = 0; i < 3; i++) {
      pos_cw[i] = optimumpos_cw[i];
    }
  }


  emxFree_real_T(&b_Pw);
  emxFree_real_T(&b_Pc_new);
  emxFree_real_T(&SMat);
  emxFree_real_T(&UMat);
  emxFree_real_T(&Pc2_new);
  emxFree_real_T(&Pc_new);
  emxFree_real_T(&Mat);
  emxFree_real_T(&n_c);
  emxFree_real_T(&Vw_bar);
  emxFree_real_T(&nc_bar);
  emxFree_real_T(&lineLenVec);
}

//
// This function follows the framework of our RPnL algorithm:
//  Robust and Efficient Pose Estimation from Line Correspondences, ACCV2012.
// input: xs(:, i) = the start point of the ith image line [startpointx, startpointy, 1];
//        xe(:, i) = the end point of the ith image line   [endpointx,   endpointy, 1];
//        Vw(:, i) = the direction of ith line in the world frame
//        Pw(:, i) = a point of ith line in the world frame
// output: rot_cw = the orientation of camera in rotation matrix parametrization
//                  (V_w = rot_cw * V_c)
//         pos_cw = the position of camera in global frame;
//                  (P_w = rot_cw * P_c + pos_cw;
// Arguments    : double xs[36]
//                double xe[36]
//                double Vw[36]
//                double Pw[36]
//                double rot_cw_data[]
//                int rot_cw_size[2]
//                double pos_cw[3]
// Return Type  : void
//
void b_PnL(double xs[36], double xe[36], double Vw[36], double Pw[36], double
           rot_cw_data[], int rot_cw_size[2], double pos_cw[3])
{
  int optimumrot_cw_size_idx_0;
  int optimumrot_cw_size_idx_1;
  double optimumpos_cw[3];
  int i;
  double lineLenVec[12];
  int LineID;
  double l1[3];
  int i2;
  double B;
  int HowToChooseFixedTwoLines;
  boolean_T guard1 = false;
  int32_T exitg1;
  double Rx[3];
  double mtmp;
  int ixstart;
  int ix;
  double temp;
  boolean_T exitg3;
  double b_temp[3];
  double d0;
  static const double dv1[3] = { 1.1, 0.9659, 0.866 };

  double nc1[3];
  double Xm[3];
  double Rot[9];
  double n_c[36];
  double nc_bar[36];
  double Vw_bar[36];
  double b_nc1;
  double sinpsi;
  static const signed char iv0[3] = { 1, 0, 0 };

  double b_Rx[9];
  double c_Rx[9];
  double A2;
  double B2;
  double C2;
  double x2;
  double y2;
  double z2;
  double coef[9];
  double polyDF[16];
  double u11;
  double u12;
  double u13;
  double u14;
  double u15;
  double u21;
  double u22;
  double u23;
  double u24;
  double u25;
  double u31;
  double u32;
  double u33;
  double u34;
  double u35;
  double u36;
  double a4;
  double a3;
  double a2;
  double a1;
  double a0;
  double b3;
  double b2;
  double b1;
  double b0;
  double b_d0;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double b_a4[9];
  int rs_size[1];
  creal_T rs_data[15];
  double b_rs_data[15];
  int b_rs_size[1];
  int minRoots_size[1];
  double minRoots_data[15];
  int n;
  boolean_T exitg2;
  int c_rs_size[1];
  boolean_T b_minRoots_data[15];
  int b_minRoots_size[1];
  double PolyVal_data[15];
  int PolyVal_size[1];
  double minimalReprojectionError;
  double sinphi;
  double Rz[9];
  static const signed char iv1[3] = { 0, 0, 1 };

  double RzRxRot[9];
  double Mat[138];
  double VMat[36];
  double SMat[138];
  double UMat[529];
  double vec[6];
  double normalizeTheta;
  double costheta;
  double sintheta;
  static const signed char iv2[3] = { 0, 1, 0 };

  double b_costheta[9];
  double c_costheta[9];
  double Pc_new[36];
  double Pc2_new[36];
  double y;
  double b_Pc_new[72];
  double b_Pw[72];
  double conditionErr;
  double numLineInFrontofCamera;
  double c_Pw[3];
  double b_RzRxRot[3];
  double reprojectionError;
  double c_temp[3];
  double optimumrot_cw_data[9];
  rot_cw_size[0] = 3;
  rot_cw_size[1] = 3;
  memset(&rot_cw_data[0], 0, 9U * sizeof(double));
  optimumrot_cw_size_idx_0 = 0;
  optimumrot_cw_size_idx_1 = 0;
  for (i = 0; i < 3; i++) {
    pos_cw[i] = 0.0;
    optimumpos_cw[i] = 0.0;
  }

  memset(&lineLenVec[0], 0, 12U * sizeof(double));

  //  added to satisfy Matlab Coder:
  LineID = -2;
  for (i2 = 0; i2 < 3; i2++) {
    l1[i2] = xs[i2] - xe[i2];
  }

  B = norm(l1);
  for (i2 = 0; i2 < 3; i2++) {
    l1[i2] /= B;
  }

  HowToChooseFixedTwoLines = 1;
  guard1 = false;
  do {
    exitg1 = 0;
    if (HowToChooseFixedTwoLines - 1 < 3) {
      if (HowToChooseFixedTwoLines == 1) {
        for (i = 0; i < 12; i++) {
          for (i2 = 0; i2 < 3; i2++) {
            Rx[i2] = xs[i2 + 3 * i] - xe[i2 + 3 * i];
          }

          lineLenVec[i] = norm(Rx);
        }

        // choose the line with longest length in the image plane;
        mtmp = lineLenVec[0];
        ixstart = 0;
        for (ix = 0; ix < 11; ix++) {
          if (lineLenVec[ix + 1] > mtmp) {
            mtmp = lineLenVec[ix + 1];
            ixstart = ix + 1;
          }
        }

        LineID = ixstart;
        lineLenVec[ixstart] = lineLenVec[0];
        lineLenVec[0] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          temp = xs[i2];
          xs[i2] = xs[i2 + 3 * ixstart];
          xs[i2 + 3 * ixstart] = temp;
          temp = xe[i2];
          xe[i2] = xe[i2 + 3 * ixstart];
          xe[i2 + 3 * ixstart] = temp;
          temp = Vw[i2];
          Vw[i2] = Vw[i2 + 3 * ixstart];
          Vw[i2 + 3 * ixstart] = temp;
          temp = Pw[i2];
          Pw[i2] = Pw[i2 + 3 * ixstart];
          Pw[i2 + 3 * ixstart] = temp;
          l1[i2] = xs[i2] - xe[i2];
        }

        B = norm(l1);
        for (i2 = 0; i2 < 3; i2++) {
          l1[i2] /= B;
        }
      }

      // first line is fixed. Find the second line
      i = 0;
      exitg3 = false;
      while ((!exitg3) && (i < 11)) {
        mtmp = lineLenVec[0];
        ixstart = 0;
        for (ix = 0; ix < 11; ix++) {
          if (lineLenVec[ix + 1] > mtmp) {
            mtmp = lineLenVec[ix + 1];
            ixstart = ix + 1;
          }
        }

        LineID = ixstart;

        // the current lonest line
        for (i2 = 0; i2 < 3; i2++) {
          b_temp[i2] = xs[i2 + 3 * ixstart] - xe[i2 + 3 * ixstart];
        }

        B = norm(b_temp);
        lineLenVec[ixstart] = 0.0;
        d0 = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          d0 += l1[i2] * (b_temp[i2] / B);
        }

        if (fabs(d0) < dv1[HowToChooseFixedTwoLines - 1]) {
          //  0<angle<180, 15<angle<165,or 30<angle<150
          exitg3 = true;
        } else {
          i++;
        }
      }

      for (i2 = 0; i2 < 3; i2++) {
        temp = xs[3 + i2];
        xs[3 + i2] = xs[i2 + 3 * LineID];
        xs[i2 + 3 * LineID] = temp;
        temp = xe[3 + i2];
        xe[3 + i2] = xe[i2 + 3 * LineID];
        xe[i2 + 3 * LineID] = temp;
        temp = Vw[3 + i2];
        Vw[3 + i2] = Vw[i2 + 3 * LineID];
        Vw[i2 + 3 * LineID] = temp;
        temp = Pw[3 + i2];
        Pw[3 + i2] = Pw[i2 + 3 * LineID];
        Pw[i2 + 3 * LineID] = temp;
      }

      lineLenVec[LineID] = lineLenVec[1];
      lineLenVec[1] = 0.0;

      //  The rotation matrix R_wc is decomposed in way which is slightly different from the description in the paper, 
      //  but the framework is the same.
      //  R_wc = (Rot') * R * Rot =  (Rot') * (Ry(theta) * Rz(phi) * Rx(psi)) * Rot 
      nc1[0] = xs[1] * xe[2] - xs[2] * xe[1];
      nc1[1] = xs[2] * xe[0] - xs[0] * xe[2];
      nc1[2] = xs[0] * xe[1] - xs[1] * xe[0];
      B = norm(nc1);
      mtmp = norm(*(double (*)[3])&Vw[0]);
      for (i2 = 0; i2 < 3; i2++) {
        b_temp[i2] = Vw[i2] / mtmp;
        nc1[i2] /= B;
      }

      Xm[0] = nc1[1] * b_temp[2] - nc1[2] * b_temp[1];
      Xm[1] = nc1[2] * b_temp[0] - nc1[0] * b_temp[2];
      Xm[2] = nc1[0] * b_temp[1] - nc1[1] * b_temp[0];
      B = norm(Xm);
      for (i2 = 0; i2 < 3; i2++) {
        Xm[i2] /= B;
      }

      // the X axis of Model frame
      // the Y axis of Model frame
      b_temp[0] = Xm[1] * nc1[2] - Xm[2] * nc1[1];
      b_temp[1] = Xm[2] * nc1[0] - Xm[0] * nc1[2];
      b_temp[2] = Xm[0] * nc1[1] - Xm[1] * nc1[0];
      B = norm(b_temp);
      for (i2 = 0; i2 < 3; i2++) {
        temp = b_temp[i2] / B;

        // the Z axis of Model frame;
        Rot[3 * i2] = Xm[i2];
        Rot[1 + 3 * i2] = nc1[i2];
        Rot[2 + 3 * i2] = temp;
        b_temp[i2] = temp;
      }

      //  Rot * [Xm, Ym, Zm] = I.
      // rotate all the vector by Rot.
      //  nc_bar(:,i) = Rot * nc(:,i)
      //  Vw_bar(:,i) = Rot * Vw(:,i)
      for (i = 0; i < 12; i++) {
        nc1[0] = xs[1 + 3 * i] * xe[2 + 3 * i] - xs[2 + 3 * i] * xe[1 + 3 * i];
        nc1[1] = xs[2 + 3 * i] * xe[3 * i] - xs[3 * i] * xe[2 + 3 * i];
        nc1[2] = xs[3 * i] * xe[1 + 3 * i] - xs[1 + 3 * i] * xe[3 * i];
        B = norm(nc1);
        for (i2 = 0; i2 < 3; i2++) {
          b_nc1 = nc1[i2] / B;
          n_c[i2 + 3 * i] = b_nc1;
          nc1[i2] = b_nc1;
        }

        for (i2 = 0; i2 < 3; i2++) {
          nc_bar[i2 + 3 * i] = 0.0;
          for (ix = 0; ix < 3; ix++) {
            nc_bar[i2 + 3 * i] += Rot[i2 + 3 * ix] * nc1[ix];
          }

          Vw_bar[i2 + 3 * i] = 0.0;
          for (ix = 0; ix < 3; ix++) {
            Vw_bar[i2 + 3 * i] += Rot[i2 + 3 * ix] * Vw[ix + 3 * i];
          }
        }
      }

      //  Determine the angle psi, it is the angle between z axis and Vw_bar(:,1). 
      //  The rotation matrix Rx(psi) rotates Vw_bar(:,1) to z axis
      // the angle between z axis and Vw_bar(:,1); cospsi=[0,0,1] * Vw_bar(:,1);. 
      sinpsi = sqrt(1.0 - Vw_bar[2] * Vw_bar[2]);
      for (i2 = 0; i2 < 3; i2++) {
        b_Rx[3 * i2] = iv0[i2];
      }

      b_Rx[1] = 0.0;
      b_Rx[4] = Vw_bar[2];
      b_Rx[7] = -sinpsi;
      b_Rx[2] = 0.0;
      b_Rx[5] = sinpsi;
      b_Rx[8] = Vw_bar[2];

      //  should be the Z axis, i.e. [0, 0, 1]';
      for (i2 = 0; i2 < 3; i2++) {
        Rx[i2] = 0.0;
        for (ix = 0; ix < 3; ix++) {
          Rx[i2] += b_Rx[i2 + 3 * ix] * Vw_bar[ix];
        }
      }

      if (1.0 - fabs(Rx[2]) > 1.0E-5) {
        for (i2 = 0; i2 < 3; i2++) {
          for (ix = 0; ix < 3; ix++) {
            c_Rx[ix + 3 * i2] = b_Rx[i2 + 3 * ix];
          }
        }

        for (i2 = 0; i2 < 3; i2++) {
          for (ix = 0; ix < 3; ix++) {
            b_Rx[ix + 3 * i2] = c_Rx[ix + 3 * i2];
          }
        }
      }

      // estimate the rotation angle phi by least square residual.
      // i.e the rotation matrix Rz(phi)
      for (i2 = 0; i2 < 3; i2++) {
        b_temp[i2] = 0.0;
        for (ix = 0; ix < 3; ix++) {
          b_temp[i2] += b_Rx[i2 + 3 * ix] * Vw_bar[3 + ix];
        }
      }

      A2 = b_temp[0];
      B2 = b_temp[1];
      C2 = b_temp[2];
      x2 = nc_bar[3];
      y2 = nc_bar[4];
      z2 = nc_bar[5];
      memset(&coef[0], 0, 9U * sizeof(double));

      // coefficients of equation (7)
      memset(&polyDF[0], 0, sizeof(double) << 4);

      // dF = ployDF(1) * t^15 + ployDF(2) * t^14 + ... + ployDF(15) * t + ployDF(16); 
      //  construct the  polynomial F'
      for (i = 0; i < 10; i++) {
        // The first two lines are included in every triplet.
        for (i2 = 0; i2 < 3; i2++) {
          b_temp[i2] = 0.0;
          for (ix = 0; ix < 3; ix++) {
            b_temp[i2] += b_Rx[i2 + 3 * ix] * Vw_bar[ix + 3 * (i + 2)];
          }
        }

        u11 = -z2 * A2 * nc_bar[1 + 3 * (i + 2)] * b_temp[1] + y2 * B2 * nc_bar
          [2 + 3 * (i + 2)] * b_temp[0];
        u12 = -y2 * A2 * nc_bar[2 + 3 * (i + 2)] * b_temp[1] + z2 * B2 * nc_bar
          [1 + 3 * (i + 2)] * b_temp[0];
        u13 = ((-y2 * B2 * nc_bar[2 + 3 * (i + 2)] * b_temp[1] + z2 * B2 *
                nc_bar[1 + 3 * (i + 2)] * b_temp[1]) + y2 * A2 * nc_bar[2 + 3 *
               (i + 2)] * b_temp[0]) - z2 * A2 * nc_bar[1 + 3 * (i + 2)] *
          b_temp[0];
        u14 = -y2 * B2 * nc_bar[3 * (i + 2)] * b_temp[2] + x2 * C2 * nc_bar[1 +
          3 * (i + 2)] * b_temp[1];
        u15 = x2 * C2 * nc_bar[1 + 3 * (i + 2)] * b_temp[0] - y2 * A2 * nc_bar[3
          * (i + 2)] * b_temp[2];
        u21 = -x2 * A2 * nc_bar[1 + 3 * (i + 2)] * b_temp[1] + y2 * B2 * nc_bar
          [3 * (i + 2)] * b_temp[0];
        u22 = -y2 * A2 * nc_bar[3 * (i + 2)] * b_temp[1] + x2 * B2 * nc_bar[1 +
          3 * (i + 2)] * b_temp[0];
        u23 = ((x2 * B2 * nc_bar[1 + 3 * (i + 2)] * b_temp[1] - y2 * B2 *
                nc_bar[3 * (i + 2)] * b_temp[1]) - x2 * A2 * nc_bar[1 + 3 * (i +
                2)] * b_temp[0]) + y2 * A2 * nc_bar[3 * (i + 2)] * b_temp[0];
        u24 = y2 * B2 * nc_bar[2 + 3 * (i + 2)] * b_temp[2] - z2 * C2 * nc_bar[1
          + 3 * (i + 2)] * b_temp[1];
        u25 = y2 * A2 * nc_bar[2 + 3 * (i + 2)] * b_temp[2] - z2 * C2 * nc_bar[1
          + 3 * (i + 2)] * b_temp[0];
        u31 = -x2 * A2 * nc_bar[2 + 3 * (i + 2)] * b_temp[0] + z2 * A2 * nc_bar
          [3 * (i + 2)] * b_temp[0];
        u32 = -x2 * B2 * nc_bar[2 + 3 * (i + 2)] * b_temp[1] + z2 * B2 * nc_bar
          [3 * (i + 2)] * b_temp[1];
        u33 = ((x2 * A2 * nc_bar[2 + 3 * (i + 2)] * b_temp[1] - z2 * A2 *
                nc_bar[3 * (i + 2)] * b_temp[1]) + x2 * B2 * nc_bar[2 + 3 * (i +
                2)] * b_temp[0]) - z2 * B2 * nc_bar[3 * (i + 2)] * b_temp[0];
        u34 = ((z2 * A2 * nc_bar[2 + 3 * (i + 2)] * b_temp[2] + x2 * A2 *
                nc_bar[3 * (i + 2)] * b_temp[2]) - z2 * C2 * nc_bar[2 + 3 * (i +
                2)] * b_temp[0]) - x2 * C2 * nc_bar[3 * (i + 2)] * b_temp[0];
        u35 = ((-z2 * B2 * nc_bar[2 + 3 * (i + 2)] * b_temp[2] - x2 * B2 *
                nc_bar[3 * (i + 2)] * b_temp[2]) + z2 * C2 * nc_bar[2 + 3 * (i +
                2)] * b_temp[1]) + x2 * C2 * nc_bar[3 * (i + 2)] * b_temp[1];
        u36 = -x2 * C2 * nc_bar[2 + 3 * (i + 2)] * b_temp[2] + z2 * C2 * nc_bar
          [3 * (i + 2)] * b_temp[2];
        a4 = ((((((((((u11 * u11 + u12 * u12) - u13 * u13) - 2.0 * u11 * u12) +
                    u21 * u21) + u22 * u22) - u23 * u23) - 2.0 * u21 * u22) -
                u31 * u31) - u32 * u32) + u33 * u33) + 2.0 * u31 * u32;
        a3 = 2.0 * ((((((((u11 * u14 - u13 * u15) - u12 * u14) + u21 * u24) -
                        u23 * u25) - u22 * u24) - u31 * u34) + u33 * u35) + u32 *
                    u34);
        a2 = (((((((((((((((-2.0 * u12 * u12 + u13 * u13) + u14 * u14) - u15 *
                          u15) + 2.0 * u11 * u12) - 2.0 * u22 * u22) + u23 * u23)
                      + u24 * u24) - u25 * u25) + 2.0 * u21 * u22) + 2.0 * u32 *
                   u32) - u33 * u33) - u34 * u34) + u35 * u35) - 2.0 * u31 * u32)
              - 2.0 * u31 * u36) + 2.0 * u32 * u36;
        a1 = 2.0 * ((((((u12 * u14 + u13 * u15) + u22 * u24) + u23 * u25) - u32 *
                      u34) - u33 * u35) - u34 * u36);
        a0 = ((((((u12 * u12 + u15 * u15) + u22 * u22) + u25 * u25) - u32 * u32)
               - u35 * u35) - u36 * u36) - 2.0 * u32 * u36;
        b3 = 2.0 * (((((u11 * u13 - u12 * u13) + u21 * u23) - u22 * u23) - u31 *
                     u33) + u32 * u33);
        b2 = 2.0 * ((((((((u11 * u15 - u12 * u15) + u13 * u14) + u21 * u25) -
                        u22 * u25) + u23 * u24) - u31 * u35) + u32 * u35) - u33 *
                    u34);
        b1 = 2.0 * ((((((u12 * u13 + u14 * u15) + u22 * u23) + u24 * u25) - u32 *
                      u33) - u34 * u35) - u33 * u36);
        b0 = 2.0 * (((u12 * u15 + u22 * u25) - u32 * u35) - u35 * u36);
        b_d0 = a0 * a0 - b0 * b0;
        d1 = 2.0 * (a0 * a1 - b0 * b1);
        d2 = (((a1 * a1 + 2.0 * a0 * a2) + b0 * b0) - b1 * b1) - 2.0 * b0 * b2;
        d3 = 2.0 * ((((a0 * a3 + a1 * a2) + b0 * b1) - b1 * b2) - b0 * b3);
        d4 = (((((a2 * a2 + 2.0 * a0 * a4) + 2.0 * a1 * a3) + b1 * b1) + 2.0 *
               b0 * b2) - b2 * b2) - 2.0 * b1 * b3;
        d5 = 2.0 * ((((a1 * a4 + a2 * a3) + b1 * b2) + b0 * b3) - b2 * b3);
        d6 = (((a3 * a3 + 2.0 * a2 * a4) + b2 * b2) - b3 * b3) + 2.0 * b1 * b3;
        d7 = 2.0 * (a3 * a4 + b2 * b3);
        d8 = a4 * a4 + b3 * b3;
        b_a4[0] = a4;
        b_a4[1] = a3;
        b_a4[2] = a2;
        b_a4[3] = a1;
        b_a4[4] = a0;
        b_a4[5] = b3;
        b_a4[6] = b2;
        b_a4[7] = b1;
        b_a4[8] = b0;
        for (i2 = 0; i2 < 9; i2++) {
          coef[i2] += b_a4[i2];
        }

        polyDF[0] += 8.0 * d8 * d8;
        polyDF[1] += 15.0 * d7 * d8;
        polyDF[2] = (polyDF[2] + 14.0 * d6 * d8) + 7.0 * d7 * d7;
        polyDF[3] += 13.0 * (d5 * d8 + d6 * d7);
        polyDF[4] = (polyDF[4] + 12.0 * (d4 * d8 + d5 * d7)) + 6.0 * d6 * d6;
        polyDF[5] += 11.0 * ((d3 * d8 + d4 * d7) + d5 * d6);
        polyDF[6] = (polyDF[6] + 10.0 * ((d2 * d8 + d3 * d7) + d4 * d6)) + 5.0 *
          d5 * d5;
        polyDF[7] += 9.0 * (((d1 * d8 + d2 * d7) + d3 * d6) + d4 * d5);
        polyDF[8] = ((polyDF[8] + 8.0 * ((d1 * d7 + d2 * d6) + d3 * d5)) + 4.0 *
                     d4 * d4) + 8.0 * b_d0 * d8;
        polyDF[9] = (polyDF[9] + 7.0 * ((d1 * d6 + d2 * d5) + d3 * d4)) + 7.0 *
          b_d0 * d7;
        polyDF[10] = ((polyDF[10] + 6.0 * (d1 * d5 + d2 * d4)) + 3.0 * d3 * d3)
          + 6.0 * b_d0 * d6;
        polyDF[11] = (polyDF[11] + 5.0 * (d1 * d4 + d2 * d3)) + 5.0 * b_d0 * d5;
        polyDF[12] = ((polyDF[12] + 4.0 * d1 * d3) + 2.0 * d2 * d2) + 4.0 * b_d0
          * d4;
        polyDF[13] = (polyDF[13] + 3.0 * d1 * d2) + 3.0 * b_d0 * d3;
        polyDF[14] = (polyDF[14] + d1 * d1) + 2.0 * b_d0 * d2;
        polyDF[15] += b_d0 * d1;
      }

      // solve polyDF
      roots(polyDF, rs_data, rs_size);

      //  retriving the local minima of the cost function.
      b_rs_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (i2 = 0; i2 < ixstart; i2++) {
        b_rs_data[i2] = rs_data[i2].re;
      }

      b_abs(b_rs_data, b_rs_size, minRoots_data, minRoots_size);
      ixstart = 1;
      n = minRoots_size[0];
      mtmp = minRoots_data[0];
      if (minRoots_size[0] > 1) {
        if (rtIsNaN(minRoots_data[0])) {
          ix = 2;
          exitg2 = false;
          while ((!exitg2) && (ix <= n)) {
            ixstart = ix;
            if (!rtIsNaN(minRoots_data[ix - 1])) {
              mtmp = minRoots_data[ix - 1];
              exitg2 = true;
            } else {
              ix++;
            }
          }
        }

        if (ixstart < minRoots_size[0]) {
          while (ixstart + 1 <= n) {
            if (minRoots_data[ixstart] > mtmp) {
              mtmp = minRoots_data[ixstart];
            }

            ixstart++;
          }
        }
      }

      c_rs_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (i2 = 0; i2 < ixstart; i2++) {
        b_rs_data[i2] = rs_data[i2].im;
      }

      b_abs(b_rs_data, c_rs_size, minRoots_data, minRoots_size);
      b_minRoots_size[0] = minRoots_size[0];
      ixstart = minRoots_size[0];
      for (i2 = 0; i2 < ixstart; i2++) {
        b_minRoots_data[i2] = (minRoots_data[i2] / mtmp > 0.001);
      }

      eml_null_assignment(rs_data, rs_size, b_minRoots_data, b_minRoots_size);
      minRoots_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (i2 = 0; i2 < ixstart; i2++) {
        minRoots_data[i2] = rs_data[i2].re;
      }

      for (i2 = 0; i2 < 15; i2++) {
        b_rs_data[i2] = (15.0 - (double)i2) * polyDF[i2];
      }

      polyval(b_rs_data, minRoots_data, minRoots_size, PolyVal_data, rs_size);
      PolyVal_size[0] = rs_size[0];
      ixstart = rs_size[0];
      for (i2 = 0; i2 < ixstart; i2++) {
        b_minRoots_data[i2] = (PolyVal_data[i2] <= 0.0);
      }

      b_eml_null_assignment(minRoots_data, minRoots_size, b_minRoots_data,
                            PolyVal_size);
      if (minRoots_size[0] == 0) {
        exitg1 = 1;
      } else {
        // for each minimum, we try to find a solution of the camera pose, then
        // choose the one with the least reprojection residual as the optimum of the solution. 
        minimalReprojectionError = 100.0;

        //  In general, there are two solutions which yields small re-projection error 
        //  or condition error:"n_c * R_wc * V_w=0". One of the solution transforms the 
        //  world scene behind the camera center, the other solution transforms the world 
        //  scene in front of camera center. While only the latter one is correct. 
        //  This can easily be checked by verifying their Z coordinates in the camera frame. 
        //  P_c(Z) must be larger than 0 if it's in front of the camera.
        for (n = 0; n < minRoots_size[0]; n++) {
          d0 = (((coef[0] * rt_powd_snf(minRoots_data[n], 4.0) + coef[1] *
                  rt_powd_snf(minRoots_data[n], 3.0)) + coef[2] *
                 (minRoots_data[n] * minRoots_data[n])) + coef[3] *
                minRoots_data[n]) + coef[4];
          b_sign(&d0);
          mtmp = ((coef[5] * rt_powd_snf(minRoots_data[n], 3.0) + coef[6] *
                   (minRoots_data[n] * minRoots_data[n])) + coef[7] *
                  minRoots_data[n]) + coef[8];
          b_sign(&mtmp);
          sinphi = -d0 * mtmp * sqrt(fabs(1.0 - minRoots_data[n] *
            minRoots_data[n]));
          Rz[0] = minRoots_data[n];
          Rz[3] = -sinphi;
          Rz[6] = 0.0;
          Rz[1] = sinphi;
          Rz[4] = minRoots_data[n];
          Rz[7] = 0.0;
          for (i2 = 0; i2 < 3; i2++) {
            Rz[2 + 3 * i2] = iv1[i2];
          }

          // now, according to Sec4.3, we estimate the rotation angle theta
          // and the translation vector at a time.
          for (i2 = 0; i2 < 3; i2++) {
            for (ix = 0; ix < 3; ix++) {
              c_Rx[i2 + 3 * ix] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                c_Rx[i2 + 3 * ix] += Rz[i2 + 3 * ixstart] * b_Rx[ixstart + 3 *
                  ix];
              }
            }

            for (ix = 0; ix < 3; ix++) {
              RzRxRot[i2 + 3 * ix] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                RzRxRot[i2 + 3 * ix] += c_Rx[i2 + 3 * ixstart] * Rot[ixstart + 3
                  * ix];
              }
            }
          }

          // According to the fact that n_i^C should be orthogonal to Pi^c and Vi^c, we  
          // have: scalarproduct(Vi^c, ni^c) = 0  and scalarproduct(Pi^c, ni^c) = 0. 
          // where Vi^c = Rwc * Vi^w,  Pi^c = Rwc *(Pi^w - pos_cw) = Rwc * Pi^w - pos; 
          // Using the above two constraints to construct linear equation system Mat about
          // [costheta, sintheta, tx, ty, tz, 1].
          memset(&Mat[0], 0, 138U * sizeof(double));
          for (i = 0; i < 12; i++) {
            for (i2 = 0; i2 < 3; i2++) {
              b_temp[i2] = 0.0;
              for (ix = 0; ix < 3; ix++) {
                b_temp[i2] += RzRxRot[i2 + 3 * ix] * Vw[ix + 3 * i];
              }

              nc1[i2] = 0.0;
              for (ix = 0; ix < 3; ix++) {
                nc1[i2] += RzRxRot[i2 + 3 * ix] * Pw[ix + 3 * i];
              }
            }

            //  apply the constraint scalarproduct(Vi^c, ni^c) = 0
            if (1 + i > 1) {
              // if i=1, then scalarproduct(Vi^c, ni^c) always be 0
              Mat[((1 + i) << 1) - 3] = nc_bar[3 * i] * b_temp[0] + nc_bar[2 + 3
                * i] * b_temp[2];
              Mat[((1 + i) << 1) + 20] = nc_bar[3 * i] * b_temp[2] - nc_bar[2 +
                3 * i] * b_temp[0];
              Mat[((1 + i) << 1) + 112] = nc_bar[1 + 3 * i] * b_temp[1];
            }

            //  apply the constraint scalarproduct(Pi^c, ni^c) = 0
            Mat[((1 + i) << 1) - 2] = nc_bar[3 * i] * nc1[0] + nc_bar[2 + 3 * i]
              * nc1[2];
            Mat[((1 + i) << 1) + 21] = nc_bar[3 * i] * nc1[2] - nc_bar[2 + 3 * i]
              * nc1[0];
            Mat[((1 + i) << 1) + 44] = -nc_bar[3 * i];
            Mat[((1 + i) << 1) + 67] = -nc_bar[1 + 3 * i];
            Mat[((1 + i) << 1) + 90] = -nc_bar[2 + 3 * i];
            Mat[((1 + i) << 1) + 113] = nc_bar[1 + 3 * i] * nc1[1];
          }

          // solve the linear system Mat * [costheta, sintheta, tx, ty, tz, 1]' = 0  using SVD,
          svd(Mat, UMat, SMat, VMat);

          //  the last column of Vmat;
          for (i2 = 0; i2 < 6; i2++) {
            vec[i2] = VMat[30 + i2] / VMat[35];
          }

          // the condition that the last element of vec should be 1.
          normalizeTheta = 1.0 / sqrt(vec[0] * vec[0] + vec[1] * vec[1]);

          // the condition costheta^2+sintheta^2 = 1;
          costheta = vec[0] * normalizeTheta;
          sintheta = vec[1] * normalizeTheta;

          // now, we get the rotation matrix rot_wc and translation pos_wc
          c_Rx[0] = costheta;
          c_Rx[3] = 0.0;
          c_Rx[6] = sintheta;
          for (i2 = 0; i2 < 3; i2++) {
            c_Rx[1 + 3 * i2] = iv2[i2];
          }

          c_Rx[2] = -sintheta;
          c_Rx[5] = 0.0;
          c_Rx[8] = costheta;
          for (i2 = 0; i2 < 3; i2++) {
            for (ix = 0; ix < 3; ix++) {
              b_costheta[i2 + 3 * ix] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                b_costheta[i2 + 3 * ix] += c_Rx[i2 + 3 * ixstart] * Rz[ixstart +
                  3 * ix];
              }
            }

            for (ix = 0; ix < 3; ix++) {
              c_costheta[i2 + 3 * ix] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                c_costheta[i2 + 3 * ix] += b_costheta[i2 + 3 * ixstart] *
                  b_Rx[ixstart + 3 * ix];
              }
            }
          }

          for (i2 = 0; i2 < 3; i2++) {
            for (ix = 0; ix < 3; ix++) {
              c_Rx[i2 + 3 * ix] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                c_Rx[i2 + 3 * ix] += Rot[ixstart + 3 * i2] * c_costheta[ixstart
                  + 3 * ix];
              }
            }

            for (ix = 0; ix < 3; ix++) {
              RzRxRot[i2 + 3 * ix] = 0.0;
              for (ixstart = 0; ixstart < 3; ixstart++) {
                RzRxRot[i2 + 3 * ix] += c_Rx[i2 + 3 * ixstart] * Rot[ixstart + 3
                  * ix];
              }

              b_costheta[ix + 3 * i2] = -Rot[i2 + 3 * ix];
            }
          }

          for (i2 = 0; i2 < 3; i2++) {
            Xm[i2] = 0.0;
            for (ix = 0; ix < 3; ix++) {
              Xm[i2] += b_costheta[i2 + 3 * ix] * vec[2 + ix];
            }
          }

          // now normalize the camera pose by 3D alignment. We first translate the points 
          // on line in the world frame Pw to points in the camera frame Pc. Then we project 
          // Pc onto the line interpretation plane as Pc_new. So we could call the point 
          // alignment algorithm to normalize the camera by aligning Pc_new and Pw. 
          // In order to improve the accuracy of the aligment step, we choose two points for each 
          // lines. The first point is Pwi, the second point is  the closest point on line i to camera center.  
          // (Pw2i = Pwi - (Pwi'*Vwi)*Vwi.)
          for (i = 0; i < 12; i++) {
            // first point on line i
            for (i2 = 0; i2 < 3; i2++) {
              d0 = 0.0;
              for (ix = 0; ix < 3; ix++) {
                d0 += RzRxRot[i2 + 3 * ix] * Pw[ix + 3 * i];
              }

              b_temp[i2] = d0 + Xm[i2];
            }

            mtmp = 0.0;
            for (i2 = 0; i2 < 3; i2++) {
              mtmp += b_temp[i2] * n_c[i2 + 3 * i];
            }

            // second point is the closest point on line i to camera center.
            y = 0.0;
            for (i2 = 0; i2 < 3; i2++) {
              Pc_new[i2 + 3 * i] = b_temp[i2] - mtmp * n_c[i2 + 3 * i];
              y += Pw[i2 + 3 * i] * Vw[i2 + 3 * i];
            }

            for (i2 = 0; i2 < 3; i2++) {
              temp = Pw[i2 + 3 * i] - y * Vw[i2 + 3 * i];
              Vw_bar[i2 + 3 * i] = temp;
              b_temp[i2] = temp;
            }

            for (i2 = 0; i2 < 3; i2++) {
              d0 = 0.0;
              for (ix = 0; ix < 3; ix++) {
                d0 += RzRxRot[i2 + 3 * ix] * b_temp[ix];
              }

              nc1[i2] = d0 + Xm[i2];
            }

            mtmp = 0.0;
            for (i2 = 0; i2 < 3; i2++) {
              mtmp += nc1[i2] * n_c[i2 + 3 * i];
            }

            for (i2 = 0; i2 < 3; i2++) {
              Pc2_new[i2 + 3 * i] = nc1[i2] - mtmp * n_c[i2 + 3 * i];
            }
          }

          for (i2 = 0; i2 < 12; i2++) {
            for (ix = 0; ix < 3; ix++) {
              b_Pc_new[ix + 3 * i2] = Pc_new[ix + 3 * i2];
            }
          }

          for (i2 = 0; i2 < 12; i2++) {
            for (ix = 0; ix < 3; ix++) {
              b_Pc_new[ix + 3 * (i2 + 12)] = Pc2_new[ix + 3 * i2];
              b_Pw[ix + 3 * i2] = Pw[ix + 3 * i2];
            }
          }

          for (i2 = 0; i2 < 12; i2++) {
            for (ix = 0; ix < 3; ix++) {
              b_Pw[ix + 3 * (i2 + 12)] = Vw_bar[ix + 3 * i2];
            }
          }

          calcampose(b_Pc_new, b_Pw, RzRxRot, Xm);
          for (i2 = 0; i2 < 3; i2++) {
            for (ix = 0; ix < 3; ix++) {
              c_Rx[ix + 3 * i2] = -RzRxRot[i2 + 3 * ix];
            }
          }

          for (i2 = 0; i2 < 3; i2++) {
            pos_cw[i2] = 0.0;
            for (ix = 0; ix < 3; ix++) {
              pos_cw[i2] += c_Rx[i2 + 3 * ix] * Xm[ix];
            }
          }

          // check the condition n_c^T * rot_wc * V_w = 0;
          conditionErr = 0.0;
          for (i = 0; i < 12; i++) {
            mtmp = 0.0;
            for (i2 = 0; i2 < 3; i2++) {
              Rx[i2] = 0.0;
              for (ix = 0; ix < 3; ix++) {
                Rx[i2] += n_c[ix + 3 * i] * RzRxRot[ix + 3 * i2];
              }

              mtmp += Rx[i2] * Vw[i2 + 3 * i];
            }

            conditionErr += mtmp * mtmp;
          }

          if ((conditionErr / 12.0 < 0.001) || (HowToChooseFixedTwoLines == 3))
          {
            // check whether the world scene is in front of the camera.
            numLineInFrontofCamera = 0.0;
            if (HowToChooseFixedTwoLines < 3) {
              for (i = 0; i < 12; i++) {
                for (i2 = 0; i2 < 3; i2++) {
                  c_Pw[i2] = Pw[i2 + 3 * i] - pos_cw[i2];
                }

                for (i2 = 0; i2 < 3; i2++) {
                  b_RzRxRot[i2] = 0.0;
                  for (ix = 0; ix < 3; ix++) {
                    b_RzRxRot[i2] += RzRxRot[i2 + 3 * ix] * c_Pw[ix];
                  }
                }

                if (b_RzRxRot[2] > 0.0) {
                  numLineInFrontofCamera++;
                }
              }
            } else {
              numLineInFrontofCamera = 12.0;
            }

            if (numLineInFrontofCamera > 6.0) {
              // most of the lines are in front of camera, then check the reprojection error. 
              reprojectionError = 0.0;
              for (i = 0; i < 12; i++) {
                for (i2 = 0; i2 < 3; i2++) {
                  b_temp[i2] = Pw[i2 + 3 * i] - pos_cw[i2];
                }

                c_temp[0] = b_temp[1] * Vw[2 + 3 * i] - b_temp[2] * Vw[1 + 3 * i];
                c_temp[1] = b_temp[2] * Vw[3 * i] - b_temp[0] * Vw[2 + 3 * i];
                c_temp[2] = b_temp[0] * Vw[1 + 3 * i] - b_temp[1] * Vw[3 * i];

                // line projection function
                d0 = 0.0;
                for (i2 = 0; i2 < 3; i2++) {
                  nc1[i2] = 0.0;
                  for (ix = 0; ix < 3; ix++) {
                    nc1[i2] += RzRxRot[i2 + 3 * ix] * c_temp[ix];
                  }

                  d0 += nc1[i2] * xs[i2 + 3 * i];
                }

                mtmp = 0.0;
                for (i2 = 0; i2 < 3; i2++) {
                  mtmp += nc1[i2] * xe[i2 + 3 * i];
                  Rx[i2] = xs[i2 + 3 * i] - xe[i2 + 3 * i];
                }

                reprojectionError += norm(Rx) / 3.0 * ((d0 * d0 + d0 * mtmp) +
                  mtmp * mtmp) / (nc1[0] * nc1[0] + nc1[1] * nc1[1]);
              }

              if (reprojectionError < minimalReprojectionError) {
                optimumrot_cw_size_idx_0 = 3;
                optimumrot_cw_size_idx_1 = 3;
                for (i2 = 0; i2 < 3; i2++) {
                  for (ix = 0; ix < 3; ix++) {
                    optimumrot_cw_data[ix + 3 * i2] = RzRxRot[i2 + 3 * ix];
                  }
                }

                for (i = 0; i < 3; i++) {
                  optimumpos_cw[i] = pos_cw[i];
                }

                minimalReprojectionError = reprojectionError;
              }
            }
          }
        }

        if (optimumrot_cw_size_idx_0 > 0) {
          guard1 = true;
          exitg1 = 1;
        } else {
          HowToChooseFixedTwoLines++;
          guard1 = false;
        }
      }
    } else {
      guard1 = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (guard1) {
    rot_cw_size[0] = optimumrot_cw_size_idx_0;
    rot_cw_size[1] = optimumrot_cw_size_idx_1;
    ixstart = optimumrot_cw_size_idx_0 * optimumrot_cw_size_idx_1;
    for (i2 = 0; i2 < ixstart; i2++) {
      rot_cw_data[i2] = optimumrot_cw_data[i2];
    }

    for (i = 0; i < 3; i++) {
      pos_cw[i] = optimumpos_cw[i];
    }
  }
}

//
// File trailer for PnL.cpp
//
// [EOF]
//
