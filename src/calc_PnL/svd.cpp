//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "svd.h"
#include "camParams.h"
#include "calc_PnL_emxutil.h"

// Function Declarations
static void b_eml_xaxpy(int n, double a, const double x[138], int ix0, double y
  [23], int iy0);
static double b_eml_xdotc(int n, const double x[529], int ix0, const double y
  [529], int iy0);
static double b_eml_xnrm2(int n, const double x[6], int ix0);
static void b_eml_xrot(double x[529], int ix0, int iy0, double c, double s);
static void b_eml_xswap(double x[529], int ix0, int iy0);
static void c_eml_xaxpy(int n, double a, const double x[23], int ix0, double y
  [138], int iy0);
static void d_eml_xaxpy(int n, double a, int ix0, double y[529], int iy0);
static void e_eml_xgesvd(const emxArray_real_T *A, emxArray_real_T *U, double
  S_data[], int S_size[1], double V[36]);
static void eml_xaxpy(int n, double a, int ix0, double y[138], int iy0);
static double eml_xdotc(int n, const double x[138], int ix0, const double y[138],
  int iy0);
static void eml_xgesvd(const double A[138], double U[529], double S[6], double
  V[36]);
static double eml_xnrm2(int n, const double x[138], int ix0);
static void eml_xscal(double a, double x[529], int ix0);
static void g_eml_xrot(int n, emxArray_real_T *x, int ix0, int iy0, double c,
  double s);
static void g_eml_xscal(int n, double a, emxArray_real_T *x, int ix0);
static void g_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0);
static double i_eml_xnrm2(int n, const emxArray_real_T *x, int ix0);
static double j_eml_xdotc(int n, const emxArray_real_T *x, int ix0, const
  emxArray_real_T *y, int iy0);
static double k_eml_xdotc(int n, const emxArray_real_T *x, int ix0, const
  emxArray_real_T *y, int iy0);
static void r_eml_xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0);
static void s_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0);
static void t_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0);
static void u_eml_xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0);

// Function Definitions

//
// Arguments    : int n
//                double a
//                const double x[138]
//                int ix0
//                double y[23]
//                int iy0
// Return Type  : void
//
static void b_eml_xaxpy(int n, double a, const double x[138], int ix0, double y
  [23], int iy0)
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
//                const double x[529]
//                int ix0
//                const double y[529]
//                int iy0
// Return Type  : double
//
static double b_eml_xdotc(int n, const double x[529], int ix0, const double y
  [529], int iy0)
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
//                const double x[6]
//                int ix0
// Return Type  : double
//
static double b_eml_xnrm2(int n, const double x[6], int ix0)
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
// Arguments    : double x[529]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
static void b_eml_xrot(double x[529], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 23; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

//
// Arguments    : double x[529]
//                int ix0
//                int iy0
// Return Type  : void
//
static void b_eml_xswap(double x[529], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 23; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int n
//                double a
//                const double x[23]
//                int ix0
//                double y[138]
//                int iy0
// Return Type  : void
//
static void c_eml_xaxpy(int n, double a, const double x[23], int ix0, double y
  [138], int iy0)
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
//                double y[529]
//                int iy0
// Return Type  : void
//
static void d_eml_xaxpy(int n, double a, int ix0, double y[529], int iy0)
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
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                double S_data[]
//                int S_size[1]
//                double V[36]
// Return Type  : void
//
static void e_eml_xgesvd(const emxArray_real_T *A, emxArray_real_T *U, double
  S_data[], int S_size[1], double V[36])
{
  emxArray_real_T *b_A;
  int m;
  int qp1;
  int n;
  double s_data[6];
  double e[6];
  emxArray_real_T *work;
  int kase;
  int nct;
  int q;
  int qq;
  int nmq;
  boolean_T apply_transform;
  double ztest0;
  int iter;
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
  b_emxInit_real_T(&b_A, 2);
  m = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = 6;
  emxEnsureCapacity((emxArray__common *)b_A, m, (int)sizeof(double));
  qp1 = A->size[0] * A->size[1];
  for (m = 0; m < qp1; m++) {
    b_A->data[m] = A->data[m];
  }

  n = A->size[0];
  for (m = 0; m < 6; m++) {
    s_data[m] = 0.0;
    e[m] = 0.0;
  }

  emxInit_real_T(&work, 1);
  qp1 = A->size[0];
  m = work->size[0];
  work->size[0] = qp1;
  emxEnsureCapacity((emxArray__common *)work, m, (int)sizeof(double));
  for (m = 0; m < qp1; m++) {
    work->data[m] = 0.0;
  }

  qp1 = A->size[0];
  kase = A->size[0];
  m = U->size[0] * U->size[1];
  U->size[0] = qp1;
  emxEnsureCapacity((emxArray__common *)U, m, (int)sizeof(double));
  m = U->size[0] * U->size[1];
  U->size[1] = kase;
  emxEnsureCapacity((emxArray__common *)U, m, (int)sizeof(double));
  qp1 *= kase;
  for (m = 0; m < qp1; m++) {
    U->data[m] = 0.0;
  }

  for (m = 0; m < 36; m++) {
    V[m] = 0.0;
  }

  if (A->size[0] - 1 <= 6) {
    nct = A->size[0] - 1;
  } else {
    nct = 6;
  }

  if (nct >= 4) {
    m = nct;
  } else {
    m = 4;
  }

  for (q = 0; q + 1 <= m; q++) {
    qq = q + n * q;
    nmq = n - q;
    apply_transform = false;
    if (q + 1 <= nct) {
      ztest0 = i_eml_xnrm2(nmq, b_A, qq + 1);
      if (ztest0 > 0.0) {
        apply_transform = true;
        if (b_A->data[qq] < 0.0) {
          ztest0 = -ztest0;
        }

        s_data[q] = ztest0;
        if (fabs(s_data[q]) >= 1.0020841800044864E-292) {
          ztest0 = 1.0 / s_data[q];
          qp1 = qq + nmq;
          for (kase = qq; kase + 1 <= qp1; kase++) {
            b_A->data[kase] *= ztest0;
          }
        } else {
          qp1 = qq + nmq;
          for (kase = qq; kase + 1 <= qp1; kase++) {
            b_A->data[kase] /= s_data[q];
          }
        }

        b_A->data[qq]++;
        s_data[q] = -s_data[q];
      } else {
        s_data[q] = 0.0;
      }
    }

    for (iter = q + 1; iter + 1 < 7; iter++) {
      qp1 = q + n * iter;
      if (apply_transform) {
        ztest0 = j_eml_xdotc(nmq, b_A, qq + 1, b_A, qp1 + 1);
        ztest0 = -(ztest0 / b_A->data[q + b_A->size[0] * q]);
        r_eml_xaxpy(nmq, ztest0, qq + 1, b_A, qp1 + 1);
      }

      e[iter] = b_A->data[qp1];
    }

    if (q + 1 <= nct) {
      for (kase = q; kase + 1 <= n; kase++) {
        U->data[kase + U->size[0] * q] = b_A->data[kase + b_A->size[0] * q];
      }
    }

    if (q + 1 <= 4) {
      ztest0 = b_eml_xnrm2(5 - q, e, q + 2);
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
          for (kase = q + 1; kase + 1 < 7; kase++) {
            e[kase] *= ztest0;
          }
        } else {
          for (kase = q + 1; kase + 1 < 7; kase++) {
            e[kase] /= ztest0;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (kase = q + 1; kase + 1 <= n; kase++) {
          work->data[kase] = 0.0;
        }

        for (iter = q + 1; iter + 1 < 7; iter++) {
          s_eml_xaxpy(nmq - 1, e[iter], b_A, (q + n * iter) + 2, work, q + 2);
        }

        for (iter = q + 1; iter + 1 < 7; iter++) {
          t_eml_xaxpy(nmq - 1, -e[iter] / e[q + 1], work, q + 2, b_A, (q + n *
            iter) + 2);
        }
      }

      for (kase = q + 1; kase + 1 < 7; kase++) {
        V[kase + 6 * q] = e[kase];
      }
    }
  }

  emxFree_real_T(&work);
  m = 4;
  if (nct < 6) {
    s_data[nct] = b_A->data[nct + b_A->size[0] * nct];
  }

  e[4] = b_A->data[4 + b_A->size[0] * 5];
  e[5] = 0.0;
  iter = nct;
  emxFree_real_T(&b_A);
  while (iter + 1 <= n) {
    for (kase = 1; kase <= n; kase++) {
      U->data[(kase + U->size[0] * iter) - 1] = 0.0;
    }

    U->data[iter + U->size[0] * iter] = 1.0;
    iter++;
  }

  for (q = nct - 1; q + 1 > 0; q--) {
    nmq = n - q;
    qq = q + n * q;
    if (s_data[q] != 0.0) {
      for (iter = q + 1; iter + 1 <= n; iter++) {
        qp1 = (q + n * iter) + 1;
        ztest0 = k_eml_xdotc(nmq, U, qq + 1, U, qp1);
        ztest0 = -(ztest0 / U->data[qq]);
        u_eml_xaxpy(nmq, ztest0, qq + 1, U, qp1);
      }

      for (kase = q; kase + 1 <= n; kase++) {
        U->data[kase + U->size[0] * q] = -U->data[kase + U->size[0] * q];
      }

      U->data[qq]++;
      for (kase = 1; kase <= q; kase++) {
        U->data[(kase + U->size[0] * q) - 1] = 0.0;
      }
    } else {
      for (kase = 1; kase <= n; kase++) {
        U->data[(kase + U->size[0] * q) - 1] = 0.0;
      }

      U->data[qq] = 1.0;
    }
  }

  for (q = 5; q > -1; q += -1) {
    if ((q + 1 <= 4) && (e[q] != 0.0)) {
      qp1 = (q + 6 * q) + 2;
      for (iter = q + 1; iter + 1 < 7; iter++) {
        kase = (q + 6 * iter) + 2;
        ztest0 = c_eml_xdotc(5 - q, V, qp1, V, kase);
        ztest0 = -(ztest0 / V[qp1 - 1]);
        e_eml_xaxpy(5 - q, ztest0, qp1, V, kase);
      }
    }

    for (kase = 0; kase < 6; kase++) {
      V[kase + 6 * q] = 0.0;
    }

    V[q + 6 * q] = 1.0;
  }

  for (q = 0; q < 6; q++) {
    ztest0 = e[q];
    if (s_data[q] != 0.0) {
      rt = fabs(s_data[q]);
      ztest = s_data[q] / rt;
      s_data[q] = rt;
      if (q + 1 < 6) {
        ztest0 = e[q] / ztest;
      }

      g_eml_xscal(n, ztest, U, 1 + n * q);
    }

    if ((q + 1 < 6) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      s_data[q + 1] *= ztest;
      b_eml_xscal(ztest, V, 1 + 6 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0;
  for (kase = 0; kase < 6; kase++) {
    ztest0 = fabs(s_data[kase]);
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
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s_data[kase]) + fabs
              (s_data[kase + 1]))) || (ztest0 <= 1.0020841800044864E-292) ||
            ((iter > 20) && (ztest0 <= 2.2204460492503131E-16 * snorm))) {
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
      nct = m + 2;
      qp1 = m + 2;
      exitg2 = false;
      while ((!exitg2) && (qp1 >= kase + 1)) {
        nct = qp1;
        if (qp1 == kase + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0;
          if (qp1 < m + 2) {
            ztest0 = fabs(e[qp1 - 1]);
          }

          if (qp1 > kase + 2) {
            ztest0 += fabs(e[qp1 - 2]);
          }

          ztest = fabs(s_data[qp1 - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s_data[qp1 - 1] = 0.0;
            exitg2 = true;
          } else {
            qp1--;
          }
        }
      }

      if (nct == kase + 1) {
        kase = 3;
      } else if (nct == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = nct;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0;
      for (kase = m; kase + 1 >= q + 1; kase--) {
        ztest0 = s_data[kase];
        eml_xrotg(&ztest0, &f, &ztest, &rt);
        s_data[kase] = ztest0;
        if (kase + 1 > q + 1) {
          f = -rt * e[kase - 1];
          e[kase - 1] *= ztest;
        }

        eml_xrot(V, 1 + 6 * kase, 1 + 6 * (m + 1), ztest, rt);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (kase = q; kase + 1 <= m + 2; kase++) {
        eml_xrotg(&s_data[kase], &f, &ztest, &rt);
        f = -rt * e[kase];
        e[kase] *= ztest;
        g_eml_xrot(n, U, 1 + n * kase, 1 + n * (q - 1), ztest, rt);
      }
      break;

     case 3:
      varargin_1[0] = fabs(s_data[m + 1]);
      varargin_1[1] = fabs(s_data[m]);
      varargin_1[2] = fabs(e[m]);
      varargin_1[3] = fabs(s_data[q]);
      varargin_1[4] = fabs(e[q]);
      qp1 = 1;
      mtmp = varargin_1[0];
      if (rtIsNaN(varargin_1[0])) {
        kase = 2;
        exitg1 = false;
        while ((!exitg1) && (kase < 6)) {
          qp1 = kase;
          if (!rtIsNaN(varargin_1[kase - 1])) {
            mtmp = varargin_1[kase - 1];
            exitg1 = true;
          } else {
            kase++;
          }
        }
      }

      if (qp1 < 5) {
        while (qp1 + 1 < 6) {
          if (varargin_1[qp1] > mtmp) {
            mtmp = varargin_1[qp1];
          }

          qp1++;
        }
      }

      f = s_data[m + 1] / mtmp;
      ztest0 = s_data[m] / mtmp;
      ztest = e[m] / mtmp;
      sqds = s_data[q] / mtmp;
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
          e[kase - 2] = f;
        }

        f = ztest * s_data[kase - 1] + rt * e[kase - 1];
        e[kase - 1] = ztest * e[kase - 1] - rt * s_data[kase - 1];
        ztest0 = rt * s_data[kase];
        s_data[kase] *= ztest;
        eml_xrot(V, 1 + 6 * (kase - 1), 1 + 6 * kase, ztest, rt);
        s_data[kase - 1] = f;
        eml_xrotg(&s_data[kase - 1], &ztest0, &ztest, &rt);
        f = ztest * e[kase - 1] + rt * s_data[kase];
        s_data[kase] = -rt * e[kase - 1] + ztest * s_data[kase];
        ztest0 = rt * e[kase];
        e[kase] *= ztest;
        g_eml_xrot(n, U, 1 + n * (kase - 1), 1 + n * kase, ztest, rt);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s_data[q] < 0.0) {
        s_data[q] = -s_data[q];
        b_eml_xscal(-1.0, V, 1 + 6 * q);
      }

      qp1 = q + 1;
      while ((q + 1 < 6) && (s_data[q] < s_data[qp1])) {
        rt = s_data[q];
        s_data[q] = s_data[qp1];
        s_data[qp1] = rt;
        eml_xswap(V, 1 + 6 * q, 1 + 6 * (q + 1));
        g_eml_xswap(n, U, 1 + n * q, 1 + n * (q + 1));
        q = qp1;
        qp1++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  S_size[0] = 6;
  for (kase = 0; kase < 6; kase++) {
    S_data[kase] = s_data[kase];
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[138]
//                int iy0
// Return Type  : void
//
static void eml_xaxpy(int n, double a, int ix0, double y[138], int iy0)
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
//                const double x[138]
//                int ix0
//                const double y[138]
//                int iy0
// Return Type  : double
//
static double eml_xdotc(int n, const double x[138], int ix0, const double y[138],
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
// Arguments    : const double A[138]
//                double U[529]
//                double S[6]
//                double V[36]
// Return Type  : void
//
static void eml_xgesvd(const double A[138], double U[529], double S[6], double
  V[36])
{
  double b_A[138];
  double s[6];
  double e[6];
  int kase;
  double work[23];
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
  memcpy(&b_A[0], &A[0], 138U * sizeof(double));
  for (kase = 0; kase < 6; kase++) {
    s[kase] = 0.0;
    e[kase] = 0.0;
  }

  memset(&work[0], 0, 23U * sizeof(double));
  memset(&U[0], 0, 529U * sizeof(double));
  memset(&V[0], 0, 36U * sizeof(double));
  for (q = 0; q < 6; q++) {
    iter = q + 23 * q;
    apply_transform = false;
    ztest0 = eml_xnrm2(23 - q, b_A, iter + 1);
    if (ztest0 > 0.0) {
      apply_transform = true;
      if (b_A[iter] < 0.0) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      if (fabs(s[q]) >= 1.0020841800044864E-292) {
        ztest0 = 1.0 / s[q];
        kase = (iter - q) + 23;
        for (qp1jj = iter; qp1jj + 1 <= kase; qp1jj++) {
          b_A[qp1jj] *= ztest0;
        }
      } else {
        kase = (iter - q) + 23;
        for (qp1jj = iter; qp1jj + 1 <= kase; qp1jj++) {
          b_A[qp1jj] /= s[q];
        }
      }

      b_A[iter]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }

    for (qs = q + 1; qs + 1 < 7; qs++) {
      kase = q + 23 * qs;
      if (apply_transform) {
        eml_xaxpy(23 - q, -(eml_xdotc(23 - q, b_A, iter + 1, b_A, kase + 1) /
                            b_A[q + 23 * q]), iter + 1, b_A, kase + 1);
      }

      e[qs] = b_A[kase];
    }

    for (qp1jj = q; qp1jj + 1 < 24; qp1jj++) {
      U[qp1jj + 23 * q] = b_A[qp1jj + 23 * q];
    }

    if (q + 1 <= 4) {
      ztest0 = b_eml_xnrm2(5 - q, e, q + 2);
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
          for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
            e[qp1jj] *= ztest0;
          }
        } else {
          for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
            e[qp1jj] /= ztest0;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (qp1jj = q + 1; qp1jj + 1 < 24; qp1jj++) {
          work[qp1jj] = 0.0;
        }

        for (qs = q + 1; qs + 1 < 7; qs++) {
          b_eml_xaxpy(22 - q, e[qs], b_A, (q + 23 * qs) + 2, work, q + 2);
        }

        for (qs = q + 1; qs + 1 < 7; qs++) {
          c_eml_xaxpy(22 - q, -e[qs] / e[q + 1], work, q + 2, b_A, (q + 23 * qs)
                      + 2);
        }
      }

      for (qp1jj = q + 1; qp1jj + 1 < 7; qp1jj++) {
        V[qp1jj + 6 * q] = e[qp1jj];
      }
    }
  }

  m = 4;
  e[4] = b_A[119];
  e[5] = 0.0;
  for (qs = 0; qs < 17; qs++) {
    memset(&U[23 * (qs + 6)], 0, 23U * sizeof(double));
    U[(qs + 23 * (qs + 6)) + 6] = 1.0;
  }

  for (q = 5; q > -1; q += -1) {
    iter = q + 23 * q;
    if (s[q] != 0.0) {
      for (qs = q + 1; qs + 1 < 24; qs++) {
        kase = (q + 23 * qs) + 1;
        d_eml_xaxpy(23 - q, -(b_eml_xdotc(23 - q, U, iter + 1, U, kase) / U[iter]),
                    iter + 1, U, kase);
      }

      for (qp1jj = q; qp1jj + 1 < 24; qp1jj++) {
        U[qp1jj + 23 * q] = -U[qp1jj + 23 * q];
      }

      U[iter]++;
      for (qp1jj = 1; qp1jj <= q; qp1jj++) {
        U[(qp1jj + 23 * q) - 1] = 0.0;
      }
    } else {
      memset(&U[23 * q], 0, 23U * sizeof(double));
      U[iter] = 1.0;
    }
  }

  for (q = 5; q > -1; q += -1) {
    if ((q + 1 <= 4) && (e[q] != 0.0)) {
      kase = (q + 6 * q) + 2;
      for (qs = q + 1; qs + 1 < 7; qs++) {
        qp1jj = (q + 6 * qs) + 2;
        e_eml_xaxpy(5 - q, -(c_eml_xdotc(5 - q, V, kase, V, qp1jj) / V[kase - 1]),
                    kase, V, qp1jj);
      }
    }

    for (qp1jj = 0; qp1jj < 6; qp1jj++) {
      V[qp1jj + 6 * q] = 0.0;
    }

    V[q + 6 * q] = 1.0;
  }

  for (q = 0; q < 6; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0) {
      rt = fabs(s[q]);
      ztest = s[q] / rt;
      s[q] = rt;
      if (q + 1 < 6) {
        ztest0 = e[q] / ztest;
      }

      eml_xscal(ztest, U, 1 + 23 * q);
    }

    if ((q + 1 < 6) && (ztest0 != 0.0)) {
      rt = fabs(ztest0);
      ztest = rt / ztest0;
      ztest0 = rt;
      s[q + 1] *= ztest;
      b_eml_xscal(ztest, V, 1 + 6 * (q + 1));
    }

    e[q] = ztest0;
  }

  iter = 0;
  snorm = 0.0;
  for (qp1jj = 0; qp1jj < 6; qp1jj++) {
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

        eml_xrot(V, 1 + 6 * qp1jj, 1 + 6 * (m + 1), ztest, rt);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (qp1jj = q; qp1jj + 1 <= m + 2; qp1jj++) {
        eml_xrotg(&s[qp1jj], &f, &ztest, &rt);
        f = -rt * e[qp1jj];
        e[qp1jj] *= ztest;
        b_eml_xrot(U, 1 + 23 * qp1jj, 1 + 23 * (q - 1), ztest, rt);
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
        eml_xrot(V, 1 + 6 * (qp1jj - 1), 1 + 6 * qp1jj, ztest, rt);
        s[qp1jj - 1] = f;
        eml_xrotg(&s[qp1jj - 1], &ztest0, &ztest, &rt);
        f = ztest * e[qp1jj - 1] + rt * s[qp1jj];
        s[qp1jj] = -rt * e[qp1jj - 1] + ztest * s[qp1jj];
        ztest0 = rt * e[qp1jj];
        e[qp1jj] *= ztest;
        b_eml_xrot(U, 1 + 23 * (qp1jj - 1), 1 + 23 * qp1jj, ztest, rt);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0) {
        s[q] = -s[q];
        b_eml_xscal(-1.0, V, 1 + 6 * q);
      }

      kase = q + 1;
      while ((q + 1 < 6) && (s[q] < s[kase])) {
        rt = s[q];
        s[q] = s[kase];
        s[kase] = rt;
        eml_xswap(V, 1 + 6 * q, 1 + 6 * (q + 1));
        b_eml_xswap(U, 1 + 23 * q, 1 + 23 * (q + 1));
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
// Arguments    : int n
//                const double x[138]
//                int ix0
// Return Type  : double
//
static double eml_xnrm2(int n, const double x[138], int ix0)
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
// Arguments    : double a
//                double x[529]
//                int ix0
// Return Type  : void
//
static void eml_xscal(double a, double x[529], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 22; k++) {
    x[k - 1] *= a;
  }
}

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
static void g_eml_xrot(int n, emxArray_real_T *x, int ix0, int iy0, double c,
  double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = c * x->data[ix] + s * x->data[iy];
    x->data[iy] = c * x->data[iy] - s * x->data[ix];
    x->data[ix] = temp;
    iy++;
    ix++;
  }
}

//
// Arguments    : int n
//                double a
//                emxArray_real_T *x
//                int ix0
// Return Type  : void
//
static void g_eml_xscal(int n, double a, emxArray_real_T *x, int ix0)
{
  int i21;
  int k;
  i21 = (ix0 + n) - 1;
  for (k = ix0; k <= i21; k++) {
    x->data[k - 1] *= a;
  }
}

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int iy0
// Return Type  : void
//
static void g_eml_xswap(int n, emxArray_real_T *x, int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 1; k <= n; k++) {
    temp = x->data[ix];
    x->data[ix] = x->data[iy];
    x->data[iy] = temp;
    ix++;
    iy++;
  }
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
// Return Type  : double
//
static double i_eml_xnrm2(int n, const emxArray_real_T *x, int ix0)
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
    absxk = fabs(x->data[k - 1]);
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
//                const emxArray_real_T *x
//                int ix0
//                const emxArray_real_T *y
//                int iy0
// Return Type  : double
//
static double j_eml_xdotc(int n, const emxArray_real_T *x, int ix0, const
  emxArray_real_T *y, int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  ix = ix0;
  iy = iy0;
  for (k = 1; k <= n; k++) {
    d += x->data[ix - 1] * y->data[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

//
// Arguments    : int n
//                const emxArray_real_T *x
//                int ix0
//                const emxArray_real_T *y
//                int iy0
// Return Type  : double
//
static double k_eml_xdotc(int n, const emxArray_real_T *x, int ix0, const
  emxArray_real_T *y, int iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  ix = ix0;
  iy = iy0;
  for (k = 1; k <= n; k++) {
    d += x->data[ix - 1] * y->data[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

//
// Arguments    : int n
//                double a
//                int ix0
//                emxArray_real_T *y
//                int iy0
// Return Type  : void
//
static void r_eml_xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y->data[iy] += a * y->data[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                const emxArray_real_T *x
//                int ix0
//                emxArray_real_T *y
//                int iy0
// Return Type  : void
//
static void s_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y->data[iy] += a * x->data[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                const emxArray_real_T *x
//                int ix0
//                emxArray_real_T *y
//                int iy0
// Return Type  : void
//
static void t_eml_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
  emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y->data[iy] += a * x->data[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                emxArray_real_T *y
//                int iy0
// Return Type  : void
//
static void u_eml_xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0)
{
  int ix;
  int iy;
  int k;
  if (a == 0.0) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y->data[iy] += a * y->data[ix];
      ix++;
      iy++;
    }
  }
}

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                emxArray_real_T *S
//                double V[36]
// Return Type  : void
//
void b_svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *S,
           double V[36])
{
  int s_size[1];
  double s_data[6];
  unsigned int uv0[2];
  int k;
  int loop_ub;
  e_eml_xgesvd(A, U, s_data, s_size, V);
  for (k = 0; k < 2; k++) {
    uv0[k] = (unsigned int)A->size[k];
  }

  k = S->size[0] * S->size[1];
  S->size[0] = (int)uv0[0];
  S->size[1] = 6;
  emxEnsureCapacity((emxArray__common *)S, k, (int)sizeof(double));
  loop_ub = (int)uv0[0] * 6;
  for (k = 0; k < loop_ub; k++) {
    S->data[k] = 0.0;
  }

  for (k = 0; k < 6; k++) {
    S->data[k + S->size[0] * k] = s_data[k];
  }
}

//
// Arguments    : const double A[138]
//                double U[529]
//                double S[138]
//                double V[36]
// Return Type  : void
//
void svd(const double A[138], double U[529], double S[138], double V[36])
{
  double s[6];
  int k;
  eml_xgesvd(A, U, s, V);
  memset(&S[0], 0, 138U * sizeof(double));
  for (k = 0; k < 6; k++) {
    S[k + 23 * k] = s[k];
  }
}

//
// File trailer for svd.cpp
//
// [EOF]
//
