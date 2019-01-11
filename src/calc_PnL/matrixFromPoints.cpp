//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: matrixFromPoints.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "matrixFromPoints.h"
#include "camParams.h"

// Function Declarations
static void d_eml_xgesvd(const double A[24], double U[4], double S[2], double V
  [144]);
static void e_eml_xrot(double x[144], int ix0, int iy0, double c, double s);
static void e_eml_xscal(double a, double x[4], int ix0);
static void e_eml_xswap(double x[144], int ix0, int iy0);
static void f_eml_xrot(double x[4], int ix0, int iy0, double c, double s);
static void f_eml_xscal(double a, double x[144], int ix0);
static void f_eml_xswap(double x[4]);
static double g_eml_xdotc(int n, const double x[24], int ix0, const double y[24],
  int iy0);
static double g_eml_xnrm2(int n, const double x[24], int ix0);
static double h_eml_xdotc(const double x[4], const double y[4]);
static double h_eml_xnrm2(int n, const double x[12], int ix0);
static double i_eml_xdotc(int n, const double x[144], int ix0, const double y
  [144], int iy0);
static void m_eml_xaxpy(int n, double a, int ix0, double y[24], int iy0);
static void n_eml_xaxpy(int n, double a, const double x[24], int ix0, double y[2]);
static void o_eml_xaxpy(int n, double a, const double x[2], double y[24], int
  iy0);
static void p_eml_xaxpy(double a, double y[4]);
static void q_eml_xaxpy(int n, double a, int ix0, double y[144], int iy0);

// Function Definitions

//
// Arguments    : const double A[24]
//                double U[4]
//                double S[2]
//                double V[144]
// Return Type  : void
//
static void d_eml_xgesvd(const double A[24], double U[4], double S[2], double V
  [144])
{
  double b_A[24];
  double s[3];
  int kase;
  double e[12];
  double work[2];
  int q;
  int qp1jj;
  boolean_T apply_transform;
  double ztest0;
  double ztest;
  int qs;
  int m;
  double rt;
  int iter;
  double snorm;
  int32_T exitg3;
  boolean_T exitg2;
  double f;
  double varargin_1[5];
  double mtmp;
  boolean_T exitg1;
  double sqds;
  memcpy(&b_A[0], &A[0], 24U * sizeof(double));
  for (kase = 0; kase < 3; kase++) {
    s[kase] = 0.0;
  }

  memset(&e[0], 0, 12U * sizeof(double));
  for (kase = 0; kase < 2; kase++) {
    work[kase] = 0.0;
  }

  for (kase = 0; kase < 4; kase++) {
    U[kase] = 0.0;
  }

  memset(&V[0], 0, 144U * sizeof(double));
  for (q = 0; q < 2; q++) {
    qp1jj = q + (q << 1);
    apply_transform = false;
    if (q + 1 <= 1) {
      ztest0 = g_eml_xnrm2(2, b_A, qp1jj + 1);
      if (ztest0 > 0.0) {
        apply_transform = true;
        if (b_A[qp1jj] < 0.0) {
          ztest0 = -ztest0;
        }

        if (fabs(ztest0) >= 1.0020841800044864E-292) {
          ztest = 1.0 / ztest0;
          for (kase = qp1jj; kase + 1 <= qp1jj + 2; kase++) {
            b_A[kase] *= ztest;
          }
        } else {
          for (kase = qp1jj; kase + 1 <= qp1jj + 2; kase++) {
            b_A[kase] /= ztest0;
          }
        }

        b_A[qp1jj]++;
        s[0] = -ztest0;
      } else {
        s[0] = 0.0;
      }
    }

    for (qs = q + 1; qs + 1 < 13; qs++) {
      kase = q + (qs << 1);
      if (apply_transform) {
        m_eml_xaxpy(2 - q, -(g_eml_xdotc(2 - q, b_A, qp1jj + 1, b_A, kase + 1) /
                             b_A[q + (q << 1)]), qp1jj + 1, b_A, kase + 1);
      }

      e[qs] = b_A[kase];
    }

    if (q + 1 <= 1) {
      for (kase = 0; kase < 2; kase++) {
        U[kase] = b_A[kase];
      }
    }

    ztest0 = h_eml_xnrm2(11 - q, e, q + 2);
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
        for (kase = q + 1; kase + 1 < 13; kase++) {
          e[kase] *= ztest0;
        }
      } else {
        for (kase = q + 1; kase + 1 < 13; kase++) {
          e[kase] /= ztest0;
        }
      }

      e[q + 1]++;
      e[q] = -e[q];
      if (q + 2 <= 2) {
        work[1] = 0.0;
        for (qs = 0; qs < 11; qs++) {
          n_eml_xaxpy(1 - q, e[qs + 1], b_A, 2 + ((qs + 1) << 1), work);
        }

        for (qs = 0; qs < 11; qs++) {
          o_eml_xaxpy(1 - q, -e[qs + 1] / e[1], work, b_A, 2 + ((qs + 1) << 1));
        }
      }
    }

    for (kase = q + 1; kase + 1 < 13; kase++) {
      V[kase + 12 * q] = e[kase];
    }
  }

  m = 1;
  s[1] = b_A[3];
  s[2] = 0.0;
  e[2] = 0.0;
  for (kase = 0; kase < 2; kase++) {
    U[2 + kase] = 0.0;
  }

  U[3] = 1.0;
  if (s[0] != 0.0) {
    p_eml_xaxpy(-(h_eml_xdotc(U, U) / U[0]), U);
    for (kase = 0; kase < 2; kase++) {
      U[kase] = -U[kase];
    }

    U[0]++;
  } else {
    for (kase = 0; kase < 2; kase++) {
      U[kase] = 0.0;
    }

    U[0] = 1.0;
  }

  for (q = 11; q > -1; q += -1) {
    if ((q + 1 <= 2) && (e[q] != 0.0)) {
      kase = (q + 12 * q) + 2;
      for (qs = q + 1; qs + 1 < 13; qs++) {
        qp1jj = (q + 12 * qs) + 2;
        q_eml_xaxpy(11 - q, -(i_eml_xdotc(11 - q, V, kase, V, qp1jj) / V[kase -
                              1]), kase, V, qp1jj);
      }
    }

    memset(&V[12 * q], 0, 12U * sizeof(double));
    V[q + 12 * q] = 1.0;
  }

  for (q = 0; q < 3; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0) {
      rt = fabs(s[q]);
      ztest = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 3) {
        ztest0 = e[q] / ztest;
      }

      if (q + 1 <= 2) {
        e_eml_xscal(ztest, U, 1 + (q << 1));
      }
    }

    if ((q + 1 < 3) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      s[q + 1] *= ztest;
      f_eml_xscal(ztest, V, 1 + 12 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0;
  for (kase = 0; kase < 3; kase++) {
    ztest0 = fabs(s[kase]);
    ztest = fabs(e[kase]);
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
    kase = m;
    do {
      exitg3 = 0;
      q = kase + 1;
      if (kase + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = fabs(e[kase]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s[kase]) + fabs(s[kase + 1])))
            || (ztest0 <= 1.0020841800044864E-292) || ((iter > 20) && (ztest0 <=
              2.2204460492503131E-16 * snorm))) {
          e[kase] = 0.0;
          exitg3 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg3 == 0);

    if (kase + 1 == m + 1) {
      kase = 4;
    } else {
      qs = m + 2;
      qp1jj = m + 2;
      exitg2 = false;
      while ((!exitg2) && (qp1jj >= kase + 1)) {
        qs = qp1jj;
        if (qp1jj == kase + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0;
          if (qp1jj < m + 2) {
            ztest0 = fabs(e[qp1jj - 1]);
          }

          if (qp1jj > kase + 2) {
            ztest0 += fabs(e[qp1jj - 2]);
          }

          ztest = fabs(s[qp1jj - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s[qp1jj - 1] = 0.0;
            exitg2 = true;
          } else {
            qp1jj--;
          }
        }
      }

      if (qs == kase + 1) {
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
      for (kase = m; kase + 1 >= q + 1; kase--) {
        ztest0 = s[kase];
        eml_xrotg(&ztest0, &f, &ztest, &rt);
        s[kase] = ztest0;
        if (kase + 1 > q + 1) {
          f = -rt * e[0];
          e[0] *= ztest;
        }

        e_eml_xrot(V, 1 + 12 * kase, 1 + 12 * (m + 1), ztest, rt);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (kase = q; kase + 1 <= m + 2; kase++) {
        eml_xrotg(&s[kase], &f, &ztest, &rt);
        f = -rt * e[kase];
        e[kase] *= ztest;
        f_eml_xrot(U, 1 + (kase << 1), 1 + ((q - 1) << 1), ztest, rt);
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
      for (kase = q + 1; kase <= m + 1; kase++) {
        eml_xrotg(&f, &ztest0, &ztest, &rt);
        if (kase > q + 1) {
          e[0] = f;
        }

        f = ztest * s[kase - 1] + rt * e[kase - 1];
        e[kase - 1] = ztest * e[kase - 1] - rt * s[kase - 1];
        ztest0 = rt * s[kase];
        s[kase] *= ztest;
        e_eml_xrot(V, 1 + 12 * (kase - 1), 1 + 12 * kase, ztest, rt);
        s[kase - 1] = f;
        eml_xrotg(&s[kase - 1], &ztest0, &ztest, &rt);
        f = ztest * e[kase - 1] + rt * s[kase];
        s[kase] = -rt * e[kase - 1] + ztest * s[kase];
        ztest0 = rt * e[kase];
        e[kase] *= ztest;
        if (kase < 2) {
          f_eml_xrot(U, 1, 3, ztest, rt);
        }
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
        f_eml_xscal(-1.0, V, 1 + 12 * q);
      }

      kase = q + 1;
      while ((q + 1 < 3) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        e_eml_xswap(V, 1 + 12 * q, 1 + 12 * (q + 1));
        if (q + 1 < 2) {
          f_eml_xswap(U);
        }

        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (kase = 0; kase < 2; kase++) {
    S[kase] = s[kase];
  }
}

//
// Arguments    : double x[144]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
static void e_eml_xrot(double x[144], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 12; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

//
// Arguments    : double a
//                double x[4]
//                int ix0
// Return Type  : void
//
static void e_eml_xscal(double a, double x[4], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 1; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : double x[144]
//                int ix0
//                int iy0
// Return Type  : void
//
static void e_eml_xswap(double x[144], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 12; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : double x[4]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
static void f_eml_xrot(double x[4], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 2; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

//
// Arguments    : double a
//                double x[144]
//                int ix0
// Return Type  : void
//
static void f_eml_xscal(double a, double x[144], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 11; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : double x[4]
// Return Type  : void
//
static void f_eml_xswap(double x[4])
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = 0;
  iy = 2;
  for (k = 0; k < 2; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int n
//                const double x[24]
//                int ix0
//                const double y[24]
//                int iy0
// Return Type  : double
//
static double g_eml_xdotc(int n, const double x[24], int ix0, const double y[24],
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
//                const double x[24]
//                int ix0
// Return Type  : double
//
static double g_eml_xnrm2(int n, const double x[24], int ix0)
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (n == 1) {
    y = fabs(x[ix0 - 1]);
  } else {
    scale = 2.2250738585072014E-308;
    for (k = ix0; k <= ix0 + 1; k++) {
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

    y = scale * sqrt(y);
  }

  return y;
}

//
// Arguments    : const double x[4]
//                const double y[4]
// Return Type  : double
//
static double h_eml_xdotc(const double x[4], const double y[4])
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  ix = 0;
  iy = 2;
  for (k = 0; k < 2; k++) {
    d += x[ix] * y[iy];
    ix++;
    iy++;
  }

  return d;
}

//
// Arguments    : int n
//                const double x[12]
//                int ix0
// Return Type  : double
//
static double h_eml_xnrm2(int n, const double x[12], int ix0)
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
//                const double x[144]
//                int ix0
//                const double y[144]
//                int iy0
// Return Type  : double
//
static double i_eml_xdotc(int n, const double x[144], int ix0, const double y
  [144], int iy0)
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
//                double y[24]
//                int iy0
// Return Type  : void
//
static void m_eml_xaxpy(int n, double a, int ix0, double y[24], int iy0)
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
//                const double x[24]
//                int ix0
//                double y[2]
// Return Type  : void
//
static void n_eml_xaxpy(int n, double a, const double x[24], int ix0, double y[2])
{
  if ((n < 1) || (a == 0.0)) {
  } else {
    y[1] += a * x[ix0 - 1];
  }
}

//
// Arguments    : int n
//                double a
//                const double x[2]
//                double y[24]
//                int iy0
// Return Type  : void
//
static void o_eml_xaxpy(int n, double a, const double x[2], double y[24], int
  iy0)
{
  if ((n < 1) || (a == 0.0)) {
  } else {
    y[iy0 - 1] += a * x[1];
  }
}

//
// Arguments    : double a
//                double y[4]
// Return Type  : void
//
static void p_eml_xaxpy(double a, double y[4])
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = 0;
    iy = 2;
    for (k = 0; k < 2; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[144]
//                int iy0
// Return Type  : void
//
static void q_eml_xaxpy(int n, double a, int ix0, double y[144], int iy0)
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
// compute camera matrix from a set of related 3d and 2d points
// Arguments    : const double p3d[96]
//                const double p2d[48]
//                double M[12]
// Return Type  : void
//
void matrixFromPoints(const double p3d[96], const double p2d[48], double M[12])
{
  double A[24];
  int i;
  int i9;
  int i10;
  double V[144];
  double s[2];
  double U[4];
  double x;
  memset(&A[0], 0, 24U * sizeof(double));

  //  create A
  for (i = 0; i < 24; i++) {
    // extract single coord
    i9 = ((1 + i) << 1) - 2;
    for (i10 = 0; i10 < 3; i10++) {
      A[i9 + (i10 << 1)] = p3d[i10 + (i << 2)];
    }

    A[6 + i9] = 1.0;
    A[8 + i9] = 0.0;
    A[10 + i9] = 0.0;
    A[12 + i9] = 0.0;
    A[14 + i9] = 0.0;
    A[16 + i9] = -p2d[i << 1] * p3d[i << 2];
    A[18 + i9] = -p2d[i << 1] * p3d[1 + (i << 2)];
    A[20 + i9] = -p2d[i << 1] * p3d[2 + (i << 2)];
    A[22 + i9] = -p2d[i << 1];
    A[1] = 0.0;
    A[3] = 0.0;
    A[5] = 0.0;
    A[7] = 0.0;
    for (i9 = 0; i9 < 3; i9++) {
      A[1 + ((i9 + 4) << 1)] = p3d[i9 + (i << 2)];
    }

    A[15] = 1.0;
    A[17] = -p2d[1 + (i << 1)] * p3d[i << 2];
    A[19] = -p2d[1 + (i << 1)] * p3d[1 + (i << 2)];
    A[21] = -p2d[1 + (i << 1)] * p3d[2 + (i << 2)];
    A[23] = -p2d[1 + (i << 1)];
  }

  //  compute SVD
  d_eml_xgesvd(A, U, s, V);

  // V
  //  get M from V (wi = 0 or smallest -> V already sorted -> last column)
  for (i9 = 0; i9 < 4; i9++) {
    M[3 * i9] = V[132 + i9];
    M[1 + 3 * i9] = V[i9 + 136];
    M[2 + 3 * i9] = V[i9 + 140];
  }

  // M
  //  scale M
  x = 0.0;
  for (i9 = 0; i9 < 3; i9++) {
    x += M[2 + 3 * i9] * M[2 + 3 * i9];
  }

  x = sqrt(x);
  for (i9 = 0; i9 < 12; i9++) {
    M[i9] /= x;
  }
}

//
// File trailer for matrixFromPoints.cpp
//
// [EOF]
//
