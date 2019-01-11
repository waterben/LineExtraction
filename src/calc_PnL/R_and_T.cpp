//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: R_and_T.cpp
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
#include "cross.h"
#include "camParams.h"
#include "calc_PnL_emxutil.h"

// Function Declarations
static void c_eml_xgesvd(const double A[42], double U[36], double S[6], double
  V[49]);
static void d_eml_xrot(double x[49], int ix0, int iy0, double c, double s);
static void d_eml_xscal(double a, double x[49], int ix0);
static void d_eml_xswap(double x[49], int ix0, int iy0);
static double e_eml_xdotc(int n, const double x[42], int ix0, const double y[42],
  int iy0);
static double e_eml_xnrm2(int n, const double x[42], int ix0);
static double f_eml_xdotc(int n, const double x[49], int ix0, const double y[49],
  int iy0);
static double f_eml_xnrm2(int n, const double x[7], int ix0);
static void i_eml_xaxpy(int n, double a, int ix0, double y[42], int iy0);
static void j_eml_xaxpy(int n, double a, const double x[42], int ix0, double y[6],
  int iy0);
static void k_eml_xaxpy(int n, double a, const double x[6], int ix0, double y[42],
  int iy0);
static void l_eml_xaxpy(int n, double a, int ix0, double y[49], int iy0);

// Function Definitions

//
// Arguments    : const double A[42]
//                double U[36]
//                double S[6]
//                double V[49]
// Return Type  : void
//
static void c_eml_xgesvd(const double A[42], double U[36], double S[6], double
  V[49])
{
  double b_A[42];
  double s[7];
  double e[7];
  int kase;
  double work[6];
  int q;
  int iter;
  boolean_T apply_transform;
  double ztest0;
  int qp1jj;
  int qs;
  int m;
  double rt;
  double ztest;
  double snorm;
  int32_T exitg3;
  boolean_T exitg2;
  double f;
  double varargin_1[5];
  double mtmp;
  boolean_T exitg1;
  double sqds;
  memcpy(&b_A[0], &A[0], 42U * sizeof(double));
  for (kase = 0; kase < 7; kase++) {
    s[kase] = 0.0;
    e[kase] = 0.0;
  }

  for (kase = 0; kase < 6; kase++) {
    work[kase] = 0.0;
  }

  memset(&U[0], 0, 36U * sizeof(double));
  memset(&V[0], 0, 49U * sizeof(double));
  for (q = 0; q < 5; q++) {
    iter = q + 6 * q;
    apply_transform = false;
    ztest0 = e_eml_xnrm2(6 - q, b_A, iter + 1);
    if (ztest0 > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabs(s[q]) >= 1.0020841800044864E-292) {
        ztest0 = 1.0 / s[q];
        kase = (iter - q) + 6;
        for (qp1jj = iter; qp1jj + 1 <= kase; qp1jj++) {
          b_A[qp1jj] *= ztest0;
        }
      } else {
        kase = (iter - q) + 6;
        for (qp1jj = iter; qp1jj + 1 <= kase; qp1jj++) {
          b_A[qp1jj] /= s[q];
        }
      }

      b_A[iter]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }

    for (qs = q + 1; qs + 1 < 8; qs++) {
      kase = q + 6 * qs;
      if (apply_transform) {
        i_eml_xaxpy(6 - q, -(e_eml_xdotc(6 - q, b_A, iter + 1, b_A, kase + 1) /
                             b_A[q + 6 * q]), iter + 1, b_A, kase + 1);
      }

      e[qs] = b_A[kase];
    }

    for (qp1jj = q; qp1jj + 1 < 7; qp1jj++) {
      U[qp1jj + 6 * q] = b_A[qp1jj + 6 * q];
    }

    ztest0 = f_eml_xnrm2(6 - q, e, q + 2);
    if (ztest0 == 0.0) {
      e[q] = 0.0;
    } else {
      if (e[q + 1] < 0.0) {
        e[q] = -ztest0;
      } else {
        e[q] = ztest0;
      }

      ztest0 = e[q];
      if (fabs(e[q]) >= 1.0020841800044864E-292) {
        ztest0 = 1.0 / e[q];
        for (qp1jj = q + 1; qp1jj + 1 < 8; qp1jj++) {
          e[qp1jj] *= ztest0;
        }
      } else {
        for (qp1jj = q + 1; qp1jj + 1 < 8; qp1jj++) {
          e[qp1jj] /= ztest0;
        }
      }

      e[q + 1]++;
      e[q] = -e[q];
      for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
        work[qp1jj] = 0.0;
      }

      for (qs = q + 1; qs + 1 < 8; qs++) {
        j_eml_xaxpy(5 - q, e[qs], b_A, (q + 6 * qs) + 2, work, q + 2);
      }

      for (qs = q + 1; qs + 1 < 8; qs++) {
        k_eml_xaxpy(5 - q, -e[qs] / e[q + 1], work, q + 2, b_A, (q + 6 * qs) + 2);
      }
    }

    for (qp1jj = q + 1; qp1jj + 1 < 8; qp1jj++) {
      V[qp1jj + 7 * q] = e[qp1jj];
    }
  }

  m = 5;
  s[5] = b_A[35];
  s[6] = 0.0;
  e[5] = b_A[41];
  e[6] = 0.0;
  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    U[30 + qp1jj] = 0.0;
  }

  U[35] = 1.0;
  for (q = 4; q > -1; q += -1) {
    iter = q + 6 * q;
    if (s[q] != 0.0) {
      for (qs = q + 1; qs + 1 < 7; qs++) {
        kase = (q + 6 * qs) + 1;
        e_eml_xaxpy(6 - q, -(c_eml_xdotc(6 - q, U, iter + 1, U, kase) / U[iter]),
                    iter + 1, U, kase);
      }

      for (qp1jj = q; qp1jj + 1 < 7; qp1jj++) {
        U[qp1jj + 6 * q] = -U[qp1jj + 6 * q];
      }

      U[iter]++;
      for (qp1jj = 1; qp1jj <= q; qp1jj++) {
        U[(qp1jj + 6 * q) - 1] = 0.0;
      }
    } else {
      for (qp1jj = 0; qp1jj < 6; qp1jj++) {
        U[qp1jj + 6 * q] = 0.0;
      }

      U[iter] = 1.0;
    }
  }

  for (q = 6; q > -1; q += -1) {
    if ((q + 1 <= 5) && (e[q] != 0.0)) {
      kase = (q + 7 * q) + 2;
      for (qs = q + 1; qs + 1 < 8; qs++) {
        qp1jj = (q + 7 * qs) + 2;
        l_eml_xaxpy(6 - q, -(f_eml_xdotc(6 - q, V, kase, V, qp1jj) / V[kase - 1]),
                    kase, V, qp1jj);
      }
    }

    for (qp1jj = 0; qp1jj < 7; qp1jj++) {
      V[qp1jj + 7 * q] = 0.0;
    }

    V[q + 7 * q] = 1.0;
  }

  for (q = 0; q < 7; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0) {
      rt = fabs(s[q]);
      ztest = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 7) {
        ztest0 = e[q] / ztest;
      }

      if (q + 1 <= 6) {
        b_eml_xscal(ztest, U, 1 + 6 * q);
      }
    }

    if ((q + 1 < 7) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      s[q + 1] *= ztest;
      d_eml_xscal(ztest, V, 1 + 7 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0;
  for (qp1jj = 0; qp1jj < 7; qp1jj++) {
    ztest0 = fabs(s[qp1jj]);
    ztest = fabs(e[qp1jj]);
    if ((ztest0 >= ztest) || rtIsNaN(ztest)) {
    } else {
      ztest0 = ztest;
    }

    if ((snorm >= ztest0) || rtIsNaN(ztest0)) {
    } else {
      snorm = ztest0;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    qp1jj = m;
    do {
      exitg3 = 0;
      q = qp1jj + 1;
      if (qp1jj + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabs(e[qp1jj]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s[qp1jj]) + fabs(s[qp1jj +
               1]))) || (ztest0 <= 1.0020841800044864E-292) || ((iter > 20) &&
             (ztest0 <= 2.2204460492503131E-16 * snorm))) {
          e[qp1jj] = 0.0;
          exitg3 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg3 == 0);

    if (qp1jj + 1 == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      kase = m + 2;
      exitg2 = false;
      while ((!exitg2) && (kase >= qp1jj + 1)) {
        qs = kase;
        if (kase == qp1jj + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0;
          if (kase < m + 2) {
            ztest0 = fabs(e[kase - 1]);
          }

          if (kase > qp1jj + 2) {
            ztest0 += fabs(e[kase - 2]);
          }

          ztest = fabs(s[kase - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s[kase - 1] = 0.0;
            exitg2 = true;
          } else {
            kase--;
          }
        }
      }

      if (qs == qp1jj + 1) {
        kase = 3;
      } else if (qs == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0;
      for (qp1jj = m; qp1jj + 1 >= q + 1; qp1jj--) {
        ztest0 = s[qp1jj];
        eml_xrotg(&ztest0, &f, &ztest, &rt);
        s[qp1jj] = ztest0;
        if (qp1jj + 1 > q + 1) {
          f = -rt * e[qp1jj - 1];
          e[qp1jj - 1] *= ztest;
        }

        d_eml_xrot(V, 1 + 7 * qp1jj, 1 + 7 * (m + 1), ztest, rt);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (qp1jj = q; qp1jj + 1 <= m + 2; qp1jj++) {
        eml_xrotg(&s[qp1jj], &f, &ztest, &rt);
        f = -rt * e[qp1jj];
        e[qp1jj] *= ztest;
        eml_xrot(U, 1 + 6 * qp1jj, 1 + 6 * (q - 1), ztest, rt);
      }
      break;

     case 3:
      varargin_1[0] = fabs(s[m + 1]);
      varargin_1[1] = fabs(s[m]);
      varargin_1[2] = fabs(e[m]);
      varargin_1[3] = fabs(s[q]);
      varargin_1[4] = fabs(e[q]);
      kase = 1;
      mtmp = varargin_1[0];
      if (rtIsNaN(varargin_1[0])) {
        qp1jj = 2;
        exitg1 = false;
        while ((!exitg1) && (qp1jj < 6)) {
          kase = qp1jj;
          if (!rtIsNaN(varargin_1[qp1jj - 1])) {
            mtmp = varargin_1[qp1jj - 1];
            exitg1 = true;
          } else {
            qp1jj++;
          }
        }
      }

      if (kase < 5) {
        while (kase + 1 < 6) {
          if (varargin_1[kase] > mtmp) {
            mtmp = varargin_1[kase];
          }

          kase++;
        }
      }

      f = s[m + 1] / mtmp;
      ztest0 = s[m] / mtmp;
      ztest = e[m] / mtmp;
      sqds = s[q] / mtmp;
      rt = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0;
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      if ((rt != 0.0) || (ztest0 != 0.0)) {
        ztest = sqrt(rt * rt + ztest0);
        if (rt < 0.0) {
          ztest = -ztest;
        }

        ztest = ztest0 / (rt + ztest);
      } else {
        ztest = 0.0;
      }

      f = (sqds + f) * (sqds - f) + ztest;
      ztest0 = sqds * (e[q] / mtmp);
      for (qp1jj = q + 1; qp1jj <= m + 1; qp1jj++) {
        eml_xrotg(&f, &ztest0, &ztest, &rt);
        if (qp1jj > q + 1) {
          e[qp1jj - 2] = f;
        }

        f = ztest * s[qp1jj - 1] + rt * e[qp1jj - 1];
        e[qp1jj - 1] = ztest * e[qp1jj - 1] - rt * s[qp1jj - 1];
        ztest0 = rt * s[qp1jj];
        s[qp1jj] *= ztest;
        d_eml_xrot(V, 1 + 7 * (qp1jj - 1), 1 + 7 * qp1jj, ztest, rt);
        s[qp1jj - 1] = f;
        eml_xrotg(&s[qp1jj - 1], &ztest0, &ztest, &rt);
        f = ztest * e[qp1jj - 1] + rt * s[qp1jj];
        s[qp1jj] = -rt * e[qp1jj - 1] + ztest * s[qp1jj];
        ztest0 = rt * e[qp1jj];
        e[qp1jj] *= ztest;
        if (qp1jj < 6) {
          eml_xrot(U, 1 + 6 * (qp1jj - 1), 1 + 6 * qp1jj, ztest, rt);
        }
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
        d_eml_xscal(-1.0, V, 1 + 7 * q);
      }

      kase = q + 1;
      while ((q + 1 < 7) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        d_eml_xswap(V, 1 + 7 * q, 1 + 7 * (q + 1));
        if (q + 1 < 6) {
          eml_xswap(U, 1 + 6 * q, 1 + 6 * (q + 1));
        }

        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
    S[qp1jj] = s[qp1jj];
  }
}

//
// Arguments    : double x[49]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
static void d_eml_xrot(double x[49], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 7; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

//
// Arguments    : double a
//                double x[49]
//                int ix0
// Return Type  : void
//
static void d_eml_xscal(double a, double x[49], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 6; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : double x[49]
//                int ix0
//                int iy0
// Return Type  : void
//
static void d_eml_xswap(double x[49], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 7; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int n
//                const double x[42]
//                int ix0
//                const double y[42]
//                int iy0
// Return Type  : double
//
static double e_eml_xdotc(int n, const double x[42], int ix0, const double y[42],
  int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  ix = ix0;
  iy = iy0;
  for (k = 1; k <= n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

//
// Arguments    : int n
//                const double x[42]
//                int ix0
// Return Type  : double
//
static double e_eml_xnrm2(int n, const double x[42], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// Arguments    : int n
//                const double x[49]
//                int ix0
//                const double y[49]
//                int iy0
// Return Type  : double
//
static double f_eml_xdotc(int n, const double x[49], int ix0, const double y[49],
  int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

//
// Arguments    : int n
//                const double x[7]
//                int ix0
// Return Type  : double
//
static double f_eml_xnrm2(int n, const double x[7], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[42]
//                int iy0
// Return Type  : void
//
static void i_eml_xaxpy(int n, double a, int ix0, double y[42], int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                const double x[42]
//                int ix0
//                double y[6]
//                int iy0
// Return Type  : void
//
static void j_eml_xaxpy(int n, double a, const double x[42], int ix0, double y[6],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                const double x[6]
//                int ix0
//                double y[42]
//                int iy0
// Return Type  : void
//
static void k_eml_xaxpy(int n, double a, const double x[6], int ix0, double y[42],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[49]
//                int iy0
// Return Type  : void
//
static void l_eml_xaxpy(int n, double a, int ix0, double y[49], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// This function follows the iterative algorithm (R_and_T) proposed by Kumar and Hanson:
//  Robust methods for estimating pose and a sensitivity analysis.
// input: xs(:, i)  = the start point of the ith image line [startpointx, startpointy, 1];
//        xe(:, i)  = the end point of the ith image line   [endpointx,   endpointy, 1];
//        P1w(:, i) = one point of ith line in the world frame, not nessary to be the endpoints.
//        P2w(:, i) = another point of ith line in the world frame, not nessary to be the endpoints.
//        initRot_cw: the iterative algorithm need initial value of the roation matrix
//        initPos_cw: the iterative algorithm need initial value of the translation vector
//        maxIterNum: the maximum number of iterations allowed by the  iterative algorithm
//        TerminateTh:the threshold to accept the estimated results and termiate the algorithm.
// output: Rot_cw = the orientation of camera in rotation matrix parametrization
//                  (V_w = Rot_cw * V_c)
//         Pos_cw = the position of camera in global frame;
//                  (P_w = Rot_cw * P_c + pos_cw;
//                   P_c = Rot_wc * P_w + pos_wc;
//                   Pos_wc = -Rot_cw' * Pos_cw and Pos_cw = -Rot_wc' * Pos_wc)
// Arguments    : const emxArray_real_T *xs
//                const emxArray_real_T *xe
//                const emxArray_real_T *P1w
//                const emxArray_real_T *P2w
//                const double initRot_cw[9]
//                const double initPos_cw[3]
//                double maxIterNum
//                double TerminateTh
//                double Rot_cw[9]
//                double Pos_cw[3]
// Return Type  : void
//
void R_and_T(const emxArray_real_T *xs, const emxArray_real_T *xe, const
             emxArray_real_T *P1w, const emxArray_real_T *P2w, const double
             initRot_cw[9], const double initPos_cw[3], double maxIterNum,
             double TerminateTh, double Rot_cw[9], double Pos_cw[3])
{
  emxArray_real_T *w;
  int n;
  int i16;
  int loop_ub;
  emxArray_real_T *nc;
  double vec[3];
  double c_bar[3];
  double B;
  double b_initRot_cw[9];
  double Rot_wc[9];
  int i17;
  double Pos_wc[3];
  int iter;
  boolean_T exitg1;
  double A[42];
  double C[9];
  double D[9];
  double F[9];
  double d_bar[3];
  double Pi[3];
  double b_Pi;
  double bi[3];
  double b_w;
  double d4;
  double scale;
  double b_c_bar;
  double VMat[49];
  double s[6];
  double UMat[36];
  double b_vec[7];
  double dv3[9];
  static const signed char iv8[3] = { 0, 1, 2 };

  boolean_T guard1 = false;
  static const signed char iv9[3] = { 3, 4, 5 };

  emxInit_real_T(&w, 1);
  n = xs->size[1] - 1;

  // first compute the weight of each line and the normal of the interpretation plane passing through to camera center and the line  
  i16 = w->size[0];
  w->size[0] = xs->size[1];
  emxEnsureCapacity((emxArray__common *)w, i16, (int)sizeof(double));
  loop_ub = xs->size[1];
  for (i16 = 0; i16 < loop_ub; i16++) {
    w->data[i16] = 0.0;
  }

  b_emxInit_real_T(&nc, 2);
  i16 = nc->size[0] * nc->size[1];
  nc->size[0] = 3;
  emxEnsureCapacity((emxArray__common *)nc, i16, (int)sizeof(double));
  loop_ub = xs->size[1];
  i16 = nc->size[0] * nc->size[1];
  nc->size[1] = loop_ub;
  emxEnsureCapacity((emxArray__common *)nc, i16, (int)sizeof(double));
  loop_ub = 3 * xs->size[1];
  for (i16 = 0; i16 < loop_ub; i16++) {
    nc->data[i16] = 0.0;
  }

  for (loop_ub = 0; loop_ub <= n; loop_ub++) {
    for (i16 = 0; i16 < 3; i16++) {
      vec[i16] = xs->data[i16 + xs->size[0] * loop_ub] - xe->data[i16 + xe->
        size[0] * loop_ub];
    }

    w->data[loop_ub] = 1.0 / norm(vec);

    //  the weight of a line is the inverse of its image length
    c_bar[0] = xs->data[1 + xs->size[0] * loop_ub] * xe->data[2 + xe->size[0] *
      loop_ub] - xs->data[2 + xs->size[0] * loop_ub] * xe->data[1 + xe->size[0] *
      loop_ub];
    c_bar[1] = xs->data[2 + xs->size[0] * loop_ub] * xe->data[xe->size[0] *
      loop_ub] - xs->data[xs->size[0] * loop_ub] * xe->data[2 + xe->size[0] *
      loop_ub];
    c_bar[2] = xs->data[xs->size[0] * loop_ub] * xe->data[1 + xe->size[0] *
      loop_ub] - xs->data[1 + xs->size[0] * loop_ub] * xe->data[xe->size[0] *
      loop_ub];
    B = norm(c_bar);
    for (i16 = 0; i16 < 3; i16++) {
      nc->data[i16 + nc->size[0] * loop_ub] = c_bar[i16] / B;
    }
  }

  for (i16 = 0; i16 < 3; i16++) {
    for (i17 = 0; i17 < 3; i17++) {
      Rot_wc[i17 + 3 * i16] = initRot_cw[i16 + 3 * i17];
      b_initRot_cw[i17 + 3 * i16] = -initRot_cw[i16 + 3 * i17];
    }
  }

  for (i16 = 0; i16 < 3; i16++) {
    Pos_wc[i16] = 0.0;
    for (i17 = 0; i17 < 3; i17++) {
      Pos_wc[i16] += b_initRot_cw[i16 + 3 * i17] * initPos_cw[i17];
    }
  }

  iter = 0;
  exitg1 = false;
  while ((!exitg1) && (iter <= (int)maxIterNum - 1)) {
    // construct the equation (31)
    memset(&A[0], 0, 42U * sizeof(double));

    //  A*[dT, dOmiga, 1] = 0
    for (i16 = 0; i16 < 9; i16++) {
      C[i16] = 0.0;
      D[i16] = 0.0;
      F[i16] = 0.0;
    }

    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      c_bar[loop_ub] = 0.0;
      d_bar[loop_ub] = 0.0;
    }

    for (loop_ub = 0; loop_ub <= n; loop_ub++) {
      for (i16 = 0; i16 < 3; i16++) {
        Pi[i16] = 0.0;
        for (i17 = 0; i17 < 3; i17++) {
          b_Pi = Pi[i16] + Rot_wc[i16 + 3 * i17] * P1w->data[i17 + P1w->size[0] *
            loop_ub];
          Pi[i16] = b_Pi;
        }
      }

      //  for first point on line
      bi[0] = Pi[1] * nc->data[2 + nc->size[0] * loop_ub] - Pi[2] * nc->data[1 +
        nc->size[0] * loop_ub];
      bi[1] = Pi[2] * nc->data[nc->size[0] * loop_ub] - Pi[0] * nc->data[2 +
        nc->size[0] * loop_ub];
      bi[2] = Pi[0] * nc->data[1 + nc->size[0] * loop_ub] - Pi[1] * nc->data
        [nc->size[0] * loop_ub];
      b_w = w->data[loop_ub];
      for (i16 = 0; i16 < 3; i16++) {
        for (i17 = 0; i17 < 3; i17++) {
          d4 = b_w * nc->data[i16 + nc->size[0] * loop_ub] * nc->data[i17 +
            nc->size[0] * loop_ub];
          C[i16 + 3 * i17] += d4;
        }
      }

      b_w = w->data[loop_ub];
      for (i16 = 0; i16 < 3; i16++) {
        for (i17 = 0; i17 < 3; i17++) {
          D[i16 + 3 * i17] += b_w * bi[i16] * bi[i17];
        }
      }

      b_w = w->data[loop_ub];
      for (i16 = 0; i16 < 3; i16++) {
        for (i17 = 0; i17 < 3; i17++) {
          d4 = b_w * nc->data[i16 + nc->size[0] * loop_ub] * bi[i17];
          F[i16 + 3 * i17] += d4;
        }
      }

      d4 = 0.0;
      for (i16 = 0; i16 < 3; i16++) {
        d4 += nc->data[i16 + nc->size[0] * loop_ub] * (Pi[i16] + Pos_wc[i16]);
      }

      scale = w->data[loop_ub] * d4;
      for (i16 = 0; i16 < 3; i16++) {
        b_c_bar = c_bar[i16] + scale * nc->data[i16 + nc->size[0] * loop_ub];
        c_bar[i16] = b_c_bar;
      }

      for (i16 = 0; i16 < 3; i16++) {
        d_bar[i16] += scale * bi[i16];
      }

      for (i16 = 0; i16 < 3; i16++) {
        Pi[i16] = 0.0;
        for (i17 = 0; i17 < 3; i17++) {
          b_Pi = Pi[i16] + Rot_wc[i16 + 3 * i17] * P2w->data[i17 + P2w->size[0] *
            loop_ub];
          Pi[i16] = b_Pi;
        }
      }

      //  for second point on line
      bi[0] = Pi[1] * nc->data[2 + nc->size[0] * loop_ub] - Pi[2] * nc->data[1 +
        nc->size[0] * loop_ub];
      bi[1] = Pi[2] * nc->data[nc->size[0] * loop_ub] - Pi[0] * nc->data[2 +
        nc->size[0] * loop_ub];
      bi[2] = Pi[0] * nc->data[1 + nc->size[0] * loop_ub] - Pi[1] * nc->data
        [nc->size[0] * loop_ub];
      b_w = w->data[loop_ub];
      for (i16 = 0; i16 < 3; i16++) {
        for (i17 = 0; i17 < 3; i17++) {
          d4 = b_w * nc->data[i16 + nc->size[0] * loop_ub] * nc->data[i17 +
            nc->size[0] * loop_ub];
          C[i16 + 3 * i17] += d4;
        }
      }

      b_w = w->data[loop_ub];
      for (i16 = 0; i16 < 3; i16++) {
        for (i17 = 0; i17 < 3; i17++) {
          D[i16 + 3 * i17] += b_w * bi[i16] * bi[i17];
        }
      }

      b_w = w->data[loop_ub];
      for (i16 = 0; i16 < 3; i16++) {
        for (i17 = 0; i17 < 3; i17++) {
          d4 = b_w * nc->data[i16 + nc->size[0] * loop_ub] * bi[i17];
          F[i16 + 3 * i17] += d4;
        }
      }

      d4 = 0.0;
      for (i16 = 0; i16 < 3; i16++) {
        d4 += nc->data[i16 + nc->size[0] * loop_ub] * (Pi[i16] + Pos_wc[i16]);
      }

      scale = w->data[loop_ub] * d4;
      for (i16 = 0; i16 < 3; i16++) {
        b_c_bar = c_bar[i16] + scale * nc->data[i16 + nc->size[0] * loop_ub];
        c_bar[i16] = b_c_bar;
      }

      for (i16 = 0; i16 < 3; i16++) {
        d_bar[i16] += scale * bi[i16];
      }
    }

    for (i16 = 0; i16 < 3; i16++) {
      for (i17 = 0; i17 < 3; i17++) {
        A[i17 + 6 * i16] = C[i17 + 3 * i16];
      }
    }

    for (i16 = 0; i16 < 3; i16++) {
      for (i17 = 0; i17 < 3; i17++) {
        A[i17 + 6 * (3 + i16)] = F[i17 + 3 * i16];
      }
    }

    for (i16 = 0; i16 < 3; i16++) {
      A[36 + i16] = c_bar[i16];
    }

    for (i16 = 0; i16 < 3; i16++) {
      for (i17 = 0; i17 < 3; i17++) {
        A[(i17 + 6 * i16) + 3] = F[i16 + 3 * i17];
      }
    }

    for (i16 = 0; i16 < 3; i16++) {
      for (i17 = 0; i17 < 3; i17++) {
        A[(i17 + 6 * (3 + i16)) + 3] = D[i17 + 3 * i16];
      }
    }

    for (i16 = 0; i16 < 3; i16++) {
      A[i16 + 39] = d_bar[i16];
    }

    // sovle the system by using SVD;
    c_eml_xgesvd(A, UMat, s, VMat);

    //  the last column of Vmat;
    for (i16 = 0; i16 < 7; i16++) {
      b_vec[i16] = VMat[42 + i16] / VMat[48];
    }

    // the condition that the last element of vec should be 1.
    // update the rotation and translation parameters;
    dv3[0] = 1.0;
    dv3[3] = -b_vec[5];
    dv3[6] = b_vec[4];
    dv3[1] = b_vec[5];
    dv3[4] = 1.0;
    dv3[7] = -b_vec[3];
    dv3[2] = -b_vec[4];
    dv3[5] = b_vec[3];
    dv3[8] = 1.0;
    for (i16 = 0; i16 < 3; i16++) {
      for (i17 = 0; i17 < 3; i17++) {
        b_initRot_cw[i16 + 3 * i17] = 0.0;
        for (loop_ub = 0; loop_ub < 3; loop_ub++) {
          b_initRot_cw[i16 + 3 * i17] += dv3[i16 + 3 * loop_ub] * Rot_wc[loop_ub
            + 3 * i17];
        }
      }
    }

    for (i16 = 0; i16 < 3; i16++) {
      for (i17 = 0; i17 < 3; i17++) {
        Rot_wc[i17 + 3 * i16] = b_initRot_cw[i17 + 3 * i16];
      }

      //  newRot_wc = ( I + [dOmiga]x ) oldRot_wc
      //  !!!!!!!! may be we can compute new R using rodrigues(r+dr)
      Pos_wc[i16] += b_vec[i16];
      vec[i16] = b_vec[iv8[i16]];
    }

    guard1 = false;
    if (norm(vec) < TerminateTh) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        vec[loop_ub] = b_vec[iv9[loop_ub]];
      }

      if (norm(vec) < 0.1 * TerminateTh) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      iter++;
    }
  }

  emxFree_real_T(&nc);
  emxFree_real_T(&w);
  for (i16 = 0; i16 < 3; i16++) {
    for (i17 = 0; i17 < 3; i17++) {
      Rot_cw[i17 + 3 * i16] = Rot_wc[i16 + 3 * i17];
      b_initRot_cw[i17 + 3 * i16] = -Rot_cw[i17 + 3 * i16];
    }
  }

  for (i16 = 0; i16 < 3; i16++) {
    Pos_cw[i16] = 0.0;
    for (i17 = 0; i17 < 3; i17++) {
      Pos_cw[i16] += b_initRot_cw[i16 + 3 * i17] * Pos_wc[i17];
    }
  }
}

//
// This function follows the iterative algorithm (R_and_T) proposed by Kumar and Hanson:
//  Robust methods for estimating pose and a sensitivity analysis.
// input: xs(:, i)  = the start point of the ith image line [startpointx, startpointy, 1];
//        xe(:, i)  = the end point of the ith image line   [endpointx,   endpointy, 1];
//        P1w(:, i) = one point of ith line in the world frame, not nessary to be the endpoints.
//        P2w(:, i) = another point of ith line in the world frame, not nessary to be the endpoints.
//        initRot_cw: the iterative algorithm need initial value of the roation matrix
//        initPos_cw: the iterative algorithm need initial value of the translation vector
//        maxIterNum: the maximum number of iterations allowed by the  iterative algorithm
//        TerminateTh:the threshold to accept the estimated results and termiate the algorithm.
// output: Rot_cw = the orientation of camera in rotation matrix parametrization
//                  (V_w = Rot_cw * V_c)
//         Pos_cw = the position of camera in global frame;
//                  (P_w = Rot_cw * P_c + pos_cw;
//                   P_c = Rot_wc * P_w + pos_wc;
//                   Pos_wc = -Rot_cw' * Pos_cw and Pos_cw = -Rot_wc' * Pos_wc)
// Arguments    : const double xs[36]
//                const double xe[36]
//                const double P1w[36]
//                const double P2w[36]
//                const double initRot_cw_data[]
//                const int initRot_cw_size[2]
//                const double initPos_cw[3]
//                double Rot_cw_data[]
//                int Rot_cw_size[2]
//                double Pos_cw_data[]
//                int Pos_cw_size[1]
// Return Type  : void
//
void b_R_and_T(const double xs[36], const double xe[36], const double P1w[36],
               const double P2w[36], const double initRot_cw_data[], const int
               initRot_cw_size[2], const double initPos_cw[3], double
               Rot_cw_data[], int Rot_cw_size[2], double Pos_cw_data[], int
               Pos_cw_size[1])
{
  double w[12];
  double nc[36];
  int m;
  double vec[3];
  int i8;
  double c_bar[3];
  double B;
  int Rot_wc_size_idx_0;
  int Rot_wc_size_idx_1;
  int br;
  int cr;
  double Rot_wc_data[9];
  int k;
  double a_data[9];
  int Pos_wc_size_idx_0;
  double Pos_wc_data[3];
  int ic;
  int ar;
  int ib;
  int ia;
  int iter;
  boolean_T exitg1;
  double A[42];
  double C[9];
  double D[9];
  double F[9];
  double d_bar[3];
  double Pi_data[3];
  double bi[3];
  double scale;
  double VMat[49];
  double s[6];
  double UMat[36];
  double b_vec[7];
  double y_data[9];
  static const signed char iv3[3] = { 0, 1, 2 };

  boolean_T guard1 = false;
  static const signed char iv4[3] = { 3, 4, 5 };

  // first compute the weight of each line and the normal of the interpretation plane passing through to camera center and the line  
  for (m = 0; m < 12; m++) {
    for (i8 = 0; i8 < 3; i8++) {
      vec[i8] = xs[i8 + 3 * m] - xe[i8 + 3 * m];
    }

    w[m] = 1.0 / norm(vec);

    //  the weight of a line is the inverse of its image length
    cross(*(double (*)[3])&xs[3 * m], *(double (*)[3])&xe[3 * m], c_bar);
    B = norm(c_bar);
    for (i8 = 0; i8 < 3; i8++) {
      nc[i8 + 3 * m] = c_bar[i8] / B;
    }
  }

  Rot_wc_size_idx_0 = initRot_cw_size[1];
  Rot_wc_size_idx_1 = initRot_cw_size[0];
  br = initRot_cw_size[0];
  for (i8 = 0; i8 < br; i8++) {
    m = initRot_cw_size[1];
    for (cr = 0; cr < m; cr++) {
      Rot_wc_data[cr + Rot_wc_size_idx_0 * i8] = initRot_cw_data[i8 +
        initRot_cw_size[0] * cr];
    }
  }

  k = initRot_cw_size[1];
  br = initRot_cw_size[0];
  for (i8 = 0; i8 < br; i8++) {
    m = initRot_cw_size[1];
    for (cr = 0; cr < m; cr++) {
      a_data[cr + k * i8] = -initRot_cw_data[i8 + initRot_cw_size[0] * cr];
    }
  }

  if (initRot_cw_size[0] == 1) {
    Pos_wc_size_idx_0 = initRot_cw_size[1];
    br = initRot_cw_size[1];
    for (i8 = 0; i8 < br; i8++) {
      Pos_wc_data[i8] = 0.0;
      cr = 0;
      while (cr <= 0) {
        Pos_wc_data[i8] += a_data[i8] * initPos_cw[0];
        cr = 1;
      }
    }
  } else {
    k = initRot_cw_size[0];
    m = initRot_cw_size[1];
    Pos_wc_size_idx_0 = (signed char)initRot_cw_size[1];
    br = (signed char)initRot_cw_size[1];
    for (i8 = 0; i8 < br; i8++) {
      Pos_wc_data[i8] = 0.0;
    }

    if (initRot_cw_size[1] == 0) {
    } else {
      cr = 0;
      while ((m > 0) && (cr <= 0)) {
        for (ic = 1; ic <= m; ic++) {
          Pos_wc_data[ic - 1] = 0.0;
        }

        cr = m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= 0)) {
        ar = 0;
        i8 = br + k;
        for (ib = br; ib + 1 <= i8; ib++) {
          if (initPos_cw[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= m; ic++) {
              ia++;
              Pos_wc_data[ic] += initPos_cw[ib] * a_data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr = m;
      }
    }
  }

  iter = 0;
  exitg1 = false;
  while ((!exitg1) && (iter < 20)) {
    // construct the equation (31)
    memset(&A[0], 0, 42U * sizeof(double));

    //  A*[dT, dOmiga, 1] = 0
    for (i8 = 0; i8 < 9; i8++) {
      C[i8] = 0.0;
      D[i8] = 0.0;
      F[i8] = 0.0;
    }

    for (m = 0; m < 3; m++) {
      c_bar[m] = 0.0;
      d_bar[m] = 0.0;
    }

    for (m = 0; m < 12; m++) {
      if (Rot_wc_size_idx_1 == 1) {
        k = Rot_wc_size_idx_0;
        for (i8 = 0; i8 < Rot_wc_size_idx_0; i8++) {
          Pi_data[i8] = 0.0;
          cr = 0;
          while (cr <= 0) {
            Pi_data[i8] += Rot_wc_data[i8] * P1w[3 * m];
            cr = 1;
          }
        }
      } else {
        k = (signed char)Rot_wc_size_idx_0;
        br = (signed char)Rot_wc_size_idx_0;
        for (i8 = 0; i8 < br; i8++) {
          Pi_data[i8] = 0.0;
        }

        if (Rot_wc_size_idx_0 == 0) {
        } else {
          cr = 0;
          while ((Rot_wc_size_idx_0 > 0) && (cr <= 0)) {
            for (ic = 1; ic <= Rot_wc_size_idx_0; ic++) {
              Pi_data[ic - 1] = 0.0;
            }

            cr = Rot_wc_size_idx_0;
          }

          br = 0;
          cr = 0;
          while ((Rot_wc_size_idx_0 > 0) && (cr <= 0)) {
            ar = 0;
            i8 = br + Rot_wc_size_idx_1;
            for (ib = br; ib + 1 <= i8; ib++) {
              if (P1w[ib + 3 * m] != 0.0) {
                ia = ar;
                for (ic = 0; ic + 1 <= Rot_wc_size_idx_0; ic++) {
                  ia++;
                  Pi_data[ic] += P1w[ib + 3 * m] * Rot_wc_data[ia - 1];
                }
              }

              ar += Rot_wc_size_idx_0;
            }

            br += Rot_wc_size_idx_1;
            cr = Rot_wc_size_idx_0;
          }
        }
      }

      //  for first point on line
      b_cross(Pi_data, *(double (*)[3])&nc[3 * m], bi);
      for (i8 = 0; i8 < 3; i8++) {
        for (cr = 0; cr < 3; cr++) {
          C[i8 + 3 * cr] += w[m] * nc[i8 + 3 * m] * nc[cr + 3 * m];
        }
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (cr = 0; cr < 3; cr++) {
          D[i8 + 3 * cr] += w[m] * bi[i8] * bi[cr];
        }
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (cr = 0; cr < 3; cr++) {
          F[i8 + 3 * cr] += w[m] * nc[i8 + 3 * m] * bi[cr];
        }
      }

      for (i8 = 0; i8 < k; i8++) {
        vec[i8] = Pi_data[i8] + Pos_wc_data[i8];
      }

      B = 0.0;
      for (i8 = 0; i8 < 3; i8++) {
        B += nc[i8 + 3 * m] * vec[i8];
      }

      scale = w[m] * B;
      for (i8 = 0; i8 < 3; i8++) {
        c_bar[i8] += scale * nc[i8 + 3 * m];
        d_bar[i8] += scale * bi[i8];
      }

      if (Rot_wc_size_idx_1 == 1) {
        k = Rot_wc_size_idx_0;
        for (i8 = 0; i8 < Rot_wc_size_idx_0; i8++) {
          Pi_data[i8] = 0.0;
          cr = 0;
          while (cr <= 0) {
            Pi_data[i8] += Rot_wc_data[i8] * P2w[3 * m];
            cr = 1;
          }
        }
      } else {
        k = (signed char)Rot_wc_size_idx_0;
        br = (signed char)Rot_wc_size_idx_0;
        for (i8 = 0; i8 < br; i8++) {
          Pi_data[i8] = 0.0;
        }

        if (Rot_wc_size_idx_0 == 0) {
        } else {
          cr = 0;
          while ((Rot_wc_size_idx_0 > 0) && (cr <= 0)) {
            for (ic = 1; ic <= Rot_wc_size_idx_0; ic++) {
              Pi_data[ic - 1] = 0.0;
            }

            cr = Rot_wc_size_idx_0;
          }

          br = 0;
          cr = 0;
          while ((Rot_wc_size_idx_0 > 0) && (cr <= 0)) {
            ar = 0;
            i8 = br + Rot_wc_size_idx_1;
            for (ib = br; ib + 1 <= i8; ib++) {
              if (P2w[ib + 3 * m] != 0.0) {
                ia = ar;
                for (ic = 0; ic + 1 <= Rot_wc_size_idx_0; ic++) {
                  ia++;
                  Pi_data[ic] += P2w[ib + 3 * m] * Rot_wc_data[ia - 1];
                }
              }

              ar += Rot_wc_size_idx_0;
            }

            br += Rot_wc_size_idx_1;
            cr = Rot_wc_size_idx_0;
          }
        }
      }

      //  for second point on line
      b_cross(Pi_data, *(double (*)[3])&nc[3 * m], bi);
      for (i8 = 0; i8 < 3; i8++) {
        for (cr = 0; cr < 3; cr++) {
          C[i8 + 3 * cr] += w[m] * nc[i8 + 3 * m] * nc[cr + 3 * m];
        }
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (cr = 0; cr < 3; cr++) {
          D[i8 + 3 * cr] += w[m] * bi[i8] * bi[cr];
        }
      }

      for (i8 = 0; i8 < 3; i8++) {
        for (cr = 0; cr < 3; cr++) {
          F[i8 + 3 * cr] += w[m] * nc[i8 + 3 * m] * bi[cr];
        }
      }

      for (i8 = 0; i8 < k; i8++) {
        vec[i8] = Pi_data[i8] + Pos_wc_data[i8];
      }

      B = 0.0;
      for (i8 = 0; i8 < 3; i8++) {
        B += nc[i8 + 3 * m] * vec[i8];
      }

      scale = w[m] * B;
      for (i8 = 0; i8 < 3; i8++) {
        c_bar[i8] += scale * nc[i8 + 3 * m];
        d_bar[i8] += scale * bi[i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (cr = 0; cr < 3; cr++) {
        A[cr + 6 * i8] = C[cr + 3 * i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (cr = 0; cr < 3; cr++) {
        A[cr + 6 * (3 + i8)] = F[cr + 3 * i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      A[36 + i8] = c_bar[i8];
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (cr = 0; cr < 3; cr++) {
        A[(cr + 6 * i8) + 3] = F[i8 + 3 * cr];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (cr = 0; cr < 3; cr++) {
        A[(cr + 6 * (3 + i8)) + 3] = D[cr + 3 * i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      A[i8 + 39] = d_bar[i8];
    }

    // sovle the system by using SVD;
    c_eml_xgesvd(A, UMat, s, VMat);

    //  the last column of Vmat;
    for (i8 = 0; i8 < 7; i8++) {
      b_vec[i8] = VMat[42 + i8] / VMat[48];
    }

    // the condition that the last element of vec should be 1.
    // update the rotation and translation parameters;
    C[0] = 1.0;
    C[3] = -b_vec[5];
    C[6] = b_vec[4];
    C[1] = b_vec[5];
    C[4] = 1.0;
    C[7] = -b_vec[3];
    C[2] = -b_vec[4];
    C[5] = b_vec[3];
    C[8] = 1.0;
    if (Rot_wc_size_idx_0 == 1) {
      m = Rot_wc_size_idx_1;
      for (i8 = 0; i8 < 3; i8++) {
        for (cr = 0; cr < Rot_wc_size_idx_1; cr++) {
          y_data[i8 + 3 * cr] = 0.0;
          for (k = 0; k < 3; k++) {
            y_data[i8 + 3 * cr] += C[i8 + 3 * k] * Rot_wc_data[k + cr];
          }
        }
      }
    } else {
      m = (signed char)Rot_wc_size_idx_1;
      br = 3 * (signed char)Rot_wc_size_idx_1;
      for (i8 = 0; i8 < br; i8++) {
        y_data[i8] = 0.0;
      }

      if (Rot_wc_size_idx_1 == 0) {
      } else {
        k = 3 * (Rot_wc_size_idx_1 - 1);
        for (cr = 0; cr <= k; cr += 3) {
          for (ic = cr; ic + 1 <= cr + 3; ic++) {
            y_data[ic] = 0.0;
          }
        }

        br = 0;
        for (cr = 0; cr <= k; cr += 3) {
          ar = 0;
          for (ib = br; ib + 1 <= br + 3; ib++) {
            if (Rot_wc_data[ib] != 0.0) {
              ia = ar;
              for (ic = cr; ic + 1 <= cr + 3; ic++) {
                ia++;
                y_data[ic] += Rot_wc_data[ib] * C[ia - 1];
              }
            }

            ar += 3;
          }

          br += 3;
        }
      }
    }

    Rot_wc_size_idx_0 = 3;
    Rot_wc_size_idx_1 = m;
    br = 3 * m;
    for (i8 = 0; i8 < br; i8++) {
      Rot_wc_data[i8] = y_data[i8];
    }

    //  newRot_wc = ( I + [dOmiga]x ) oldRot_wc
    //  !!!!!!!! may be we can compute new R using rodrigues(r+dr)
    Pos_wc_size_idx_0 = 3;
    for (i8 = 0; i8 < 3; i8++) {
      vec[i8] = b_vec[iv3[i8]];
      Pos_wc_data[i8] += b_vec[i8];
    }

    guard1 = false;
    if (norm(vec) < 1.0E-5) {
      for (m = 0; m < 3; m++) {
        vec[m] = b_vec[iv4[m]];
      }

      if (norm(vec) < 1.0000000000000002E-6) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      iter++;
    }
  }

  Rot_cw_size[0] = Rot_wc_size_idx_1;
  Rot_cw_size[1] = Rot_wc_size_idx_0;
  for (i8 = 0; i8 < Rot_wc_size_idx_0; i8++) {
    for (cr = 0; cr < Rot_wc_size_idx_1; cr++) {
      Rot_cw_data[cr + Rot_wc_size_idx_1 * i8] = Rot_wc_data[i8 +
        Rot_wc_size_idx_0 * cr];
    }
  }

  br = Rot_wc_size_idx_1 * Rot_wc_size_idx_0;
  for (i8 = 0; i8 < br; i8++) {
    a_data[i8] = -Rot_cw_data[i8];
  }

  if ((Rot_wc_size_idx_0 == 1) || (Pos_wc_size_idx_0 == 1)) {
    Pos_cw_size[0] = Rot_wc_size_idx_1;
    for (i8 = 0; i8 < Rot_wc_size_idx_1; i8++) {
      Pos_cw_data[i8] = 0.0;
      for (cr = 0; cr < Rot_wc_size_idx_0; cr++) {
        Pos_cw_data[i8] += a_data[i8 + Rot_wc_size_idx_1 * cr] * Pos_wc_data[cr];
      }
    }
  } else {
    Pos_cw_size[0] = (signed char)Rot_wc_size_idx_1;
    br = (signed char)Rot_wc_size_idx_1;
    for (i8 = 0; i8 < br; i8++) {
      Pos_cw_data[i8] = 0.0;
    }

    if (Rot_wc_size_idx_1 == 0) {
    } else {
      cr = 0;
      while ((Rot_wc_size_idx_1 > 0) && (cr <= 0)) {
        for (ic = 1; ic <= Rot_wc_size_idx_1; ic++) {
          Pos_cw_data[ic - 1] = 0.0;
        }

        cr = Rot_wc_size_idx_1;
      }

      br = 0;
      cr = 0;
      while ((Rot_wc_size_idx_1 > 0) && (cr <= 0)) {
        ar = 0;
        i8 = br + Rot_wc_size_idx_0;
        for (ib = br; ib + 1 <= i8; ib++) {
          if (Pos_wc_data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= Rot_wc_size_idx_1; ic++) {
              ia++;
              Pos_cw_data[ic] += Pos_wc_data[ib] * a_data[ia - 1];
            }
          }

          ar += Rot_wc_size_idx_1;
        }

        br += Rot_wc_size_idx_0;
        cr = Rot_wc_size_idx_1;
      }
    }
  }
}

//
// Arguments    : double a
//                double x[36]
//                int ix0
// Return Type  : void
//
void b_eml_xscal(double a, double x[36], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 5; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : int n
//                const double x[36]
//                int ix0
//                const double y[36]
//                int iy0
// Return Type  : double
//
double c_eml_xdotc(int n, const double x[36], int ix0, const double y[36], int
                   iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[36]
//                int iy0
// Return Type  : void
//
void e_eml_xaxpy(int n, double a, int ix0, double y[36], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : double x[36]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void eml_xrot(double x[36], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 6; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

//
// Arguments    : double x[36]
//                int ix0
//                int iy0
// Return Type  : void
//
void eml_xswap(double x[36], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 6; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

//
// File trailer for R_and_T.cpp
//
// [EOF]
//
