
#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ifft2.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "ifft2.h"

#include "ipermute.h"
#include "logGaborFilter.h"
#include "logGaborFilter_emxutil.h"
#include "logGaborFilter_rtwutil.h"
#include "phasecong.h"
#include "rt_nonfinite.h"

// Function Declarations
static void b_eml_fft(const emxArray_creal_T* x, int n, emxArray_creal_T* y);

// Function Definitions

//
// Arguments    : const emxArray_creal_T *x
//                int n
//                emxArray_creal_T *y
// Return Type  : void
//
static void b_eml_fft(const emxArray_creal_T* x, int n, emxArray_creal_T* y) {
  int ihi;
  int n2;
  int nd2;
  int i2;
  int minval;
  int ixDelta;
  emxArray_real_T* costab1q;
  int nRowsD2;
  int nRowsD4;
  int lastChan;
  double e;
  int k;
  emxArray_real_T* costab;
  emxArray_real_T* sintab;
  int ix;
  int chanStart;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  int istart;
  int j;
  double twid_im;
  ihi = y->size[0] * y->size[1];
  y->size[0] = n;
  y->size[1] = x->size[1];
  emxEnsureCapacity((emxArray__common*)y, ihi, (int)sizeof(creal_T));
  if (n > x->size[0]) {
    ihi = y->size[0] * y->size[1];
    emxEnsureCapacity((emxArray__common*)y, ihi, (int)sizeof(creal_T));
    n2 = y->size[1];
    for (ihi = 0; ihi < n2; ihi++) {
      nd2 = y->size[0];
      for (i2 = 0; i2 < nd2; i2++) {
        y->data[i2 + y->size[0] * ihi].re = 0.0;
        y->data[i2 + y->size[0] * ihi].im = 0.0;
      }
    }
  }

  if ((x->size[0] == 0) || (x->size[1] == 0)) {
  } else {
    if (x->size[0] <= n) {
      minval = x->size[0];
    } else {
      minval = n;
    }

    n2 = (x->size[0] - minval) + 1;
    if (1 >= n2) {
      ixDelta = 1;
    } else {
      ixDelta = n2;
    }

    emxInit_real_T(&costab1q, 2);
    nRowsD2 = n / 2;
    nRowsD4 = nRowsD2 / 2;
    lastChan = n * (div_s32(x->size[0] * x->size[1], x->size[0]) - 1);
    e = 6.2831853071795862 / (double)n;
    ihi = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = nRowsD4 + 1;
    emxEnsureCapacity((emxArray__common*)costab1q, ihi, (int)sizeof(double));
    costab1q->data[0] = 1.0;
    nd2 = nRowsD4 / 2;
    for (k = 1; k <= nd2; k++) {
      costab1q->data[k] = cos(e * (double)k);
    }

    for (k = nd2 + 1; k < nRowsD4; k++) {
      costab1q->data[k] = sin(e * (double)(nRowsD4 - k));
    }

    emxInit_real_T(&costab, 2);
    emxInit_real_T(&sintab, 2);
    costab1q->data[nRowsD4] = 0.0;
    nd2 = costab1q->size[1] - 1;
    n2 = (costab1q->size[1] - 1) << 1;
    ihi = costab->size[0] * costab->size[1];
    costab->size[0] = 1;
    costab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common*)costab, ihi, (int)sizeof(double));
    ihi = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = n2 + 1;
    emxEnsureCapacity((emxArray__common*)sintab, ihi, (int)sizeof(double));
    costab->data[0] = 1.0;
    sintab->data[0] = 0.0;
    for (k = 1; k <= nd2; k++) {
      costab->data[k] = costab1q->data[k];
      sintab->data[k] = costab1q->data[nd2 - k];
    }

    for (k = costab1q->size[1]; k <= n2; k++) {
      costab->data[k] = -costab1q->data[n2 - k];
      sintab->data[k] = costab1q->data[k - nd2];
    }

    emxFree_real_T(&costab1q);
    ix = 0;
    chanStart = 0;
    while ((n > 0) && (chanStart <= lastChan)) {
      n2 = 0;
      nd2 = chanStart;
      for (i = 1; i < minval; i++) {
        y->data[nd2] = x->data[ix];
        nd2 = n;
        tst = true;
        while (tst) {
          nd2 >>= 1;
          n2 ^= nd2;
          tst = ((n2 & nd2) == 0);
        }

        nd2 = chanStart + n2;
        ix++;
      }

      y->data[nd2] = x->data[ix];
      ix += ixDelta;
      i2 = (chanStart + n) - 2;
      if (n > 1) {
        for (i = chanStart; i <= i2; i += 2) {
          temp_re = y->data[i + 1].re;
          temp_im = y->data[i + 1].im;
          y->data[i + 1].re = y->data[i].re - y->data[i + 1].re;
          y->data[i + 1].im = y->data[i].im - y->data[i + 1].im;
          y->data[i].re += temp_re;
          y->data[i].im += temp_im;
        }
      }

      nd2 = 2;
      n2 = 4;
      k = nRowsD4;
      i2 = 1 + ((nRowsD4 - 1) << 2);
      while (k > 0) {
        i = chanStart;
        ihi = chanStart + i2;
        while (i < ihi) {
          temp_re = y->data[i + nd2].re;
          temp_im = y->data[i + nd2].im;
          y->data[i + nd2].re = y->data[i].re - temp_re;
          y->data[i + nd2].im = y->data[i].im - temp_im;
          y->data[i].re += temp_re;
          y->data[i].im += temp_im;
          i += n2;
        }

        istart = chanStart + 1;
        for (j = k; j < nRowsD2; j += k) {
          e = costab->data[j];
          twid_im = sintab->data[j];
          i = istart;
          ihi = istart + i2;
          while (i < ihi) {
            temp_re = e * y->data[i + nd2].re - twid_im * y->data[i + nd2].im;
            temp_im = e * y->data[i + nd2].im + twid_im * y->data[i + nd2].re;
            y->data[i + nd2].re = y->data[i].re - temp_re;
            y->data[i + nd2].im = y->data[i].im - temp_im;
            y->data[i].re += temp_re;
            y->data[i].im += temp_im;
            i += n2;
          }

          istart++;
        }

        k /= 2;
        nd2 = n2;
        n2 <<= 1;
        i2 -= nd2;
      }

      chanStart += n;
    }

    emxFree_real_T(&sintab);
    emxFree_real_T(&costab);
    if (y->size[0] > 1) {
      e = 1.0 / (double)y->size[0];
      ihi = y->size[0] * y->size[1];
      emxEnsureCapacity((emxArray__common*)y, ihi, (int)sizeof(creal_T));
      n2 = y->size[0];
      nd2 = y->size[1];
      n2 *= nd2;
      for (ihi = 0; ihi < n2; ihi++) {
        y->data[ihi].re *= e;
        y->data[ihi].im *= e;
      }
    }
  }
}

//
// Arguments    : const emxArray_creal_T *x
//                emxArray_creal_T *f
// Return Type  : void
//
void ifft2(const emxArray_creal_T* x, emxArray_creal_T* f) {
  emxArray_creal_T* b_x;
  int iheight;
  int sz_idx_0;
  int ju;
  int n;
  emxArray_creal_T* y;
  emxArray_creal_T* b_y1;
  int n1;
  int minval;
  int ixDelta;
  emxArray_real_T* costab1q;
  int nRowsD2;
  int nRowsD4;
  int lastChan;
  double e;
  int k;
  emxArray_real_T* costab;
  emxArray_real_T* sintab;
  int ix;
  int chanStart;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  int istart;
  int j;
  double twid_im;
  emxInit_creal_T(&b_x, 2);
  iheight = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[1];
  b_x->size[1] = x->size[0];
  emxEnsureCapacity((emxArray__common*)b_x, iheight, (int)sizeof(creal_T));
  sz_idx_0 = x->size[0];
  for (iheight = 0; iheight < sz_idx_0; iheight++) {
    ju = x->size[1];
    for (n = 0; n < ju; n++) {
      b_x->data[n + b_x->size[0] * iheight] = x->data[iheight + x->size[0] * n];
    }
  }

  emxInit_creal_T(&y, 2);
  emxInit_creal_T(&b_y1, 2);
  b_eml_fft(b_x, x->size[1], b_y1);
  ipermute(b_y1, y);
  n1 = y->size[0];
  sz_idx_0 = y->size[0];
  iheight = f->size[0] * f->size[1];
  f->size[0] = sz_idx_0;
  f->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common*)f, iheight, (int)sizeof(creal_T));
  emxFree_creal_T(&b_x);
  emxFree_creal_T(&b_y1);
  if ((y->size[0] == 0) || (y->size[1] == 0)) {
  } else {
    minval = y->size[0] - 1;
    ju = y->size[0] - minval;
    if (1 >= ju) {
      ixDelta = 1;
    } else {
      ixDelta = ju;
    }

    emxInit_real_T(&costab1q, 2);
    iheight = y->size[0];
    nRowsD2 = iheight / 2;
    nRowsD4 = nRowsD2 / 2;
    lastChan = y->size[0] * (div_s32(y->size[0] * y->size[1], y->size[0]) - 1);
    e = 6.2831853071795862 / (double)y->size[0];
    iheight = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = nRowsD4 + 1;
    emxEnsureCapacity((emxArray__common*)costab1q, iheight, (int)sizeof(double));
    costab1q->data[0] = 1.0;
    sz_idx_0 = nRowsD4 / 2;
    for (k = 1; k <= sz_idx_0; k++) {
      costab1q->data[k] = cos(e * (double)k);
    }

    for (k = sz_idx_0 + 1; k < nRowsD4; k++) {
      costab1q->data[k] = sin(e * (double)(nRowsD4 - k));
    }

    emxInit_real_T(&costab, 2);
    emxInit_real_T(&sintab, 2);
    costab1q->data[nRowsD4] = 0.0;
    n = costab1q->size[1] - 1;
    sz_idx_0 = (costab1q->size[1] - 1) << 1;
    iheight = costab->size[0] * costab->size[1];
    costab->size[0] = 1;
    costab->size[1] = sz_idx_0 + 1;
    emxEnsureCapacity((emxArray__common*)costab, iheight, (int)sizeof(double));
    iheight = sintab->size[0] * sintab->size[1];
    sintab->size[0] = 1;
    sintab->size[1] = sz_idx_0 + 1;
    emxEnsureCapacity((emxArray__common*)sintab, iheight, (int)sizeof(double));
    costab->data[0] = 1.0;
    sintab->data[0] = 0.0;
    for (k = 1; k <= n; k++) {
      costab->data[k] = costab1q->data[k];
      sintab->data[k] = costab1q->data[n - k];
    }

    for (k = costab1q->size[1]; k <= sz_idx_0; k++) {
      costab->data[k] = -costab1q->data[sz_idx_0 - k];
      sintab->data[k] = costab1q->data[k - n];
    }

    emxFree_real_T(&costab1q);
    ix = 0;
    chanStart = 0;
    while ((n1 > 0) && (chanStart <= lastChan)) {
      ju = 0;
      sz_idx_0 = chanStart;
      for (i = 1; i <= minval; i++) {
        f->data[sz_idx_0] = y->data[ix];
        n = n1;
        tst = true;
        while (tst) {
          n >>= 1;
          ju ^= n;
          tst = ((ju & n) == 0);
        }

        sz_idx_0 = chanStart + ju;
        ix++;
      }

      f->data[sz_idx_0] = y->data[ix];
      ix += ixDelta;
      sz_idx_0 = (chanStart + n1) - 2;
      if (n1 > 1) {
        for (i = chanStart; i <= sz_idx_0; i += 2) {
          temp_re = f->data[i + 1].re;
          temp_im = f->data[i + 1].im;
          f->data[i + 1].re = f->data[i].re - f->data[i + 1].re;
          f->data[i + 1].im = f->data[i].im - f->data[i + 1].im;
          f->data[i].re += temp_re;
          f->data[i].im += temp_im;
        }
      }

      sz_idx_0 = 2;
      ju = 4;
      k = nRowsD4;
      iheight = 1 + ((nRowsD4 - 1) << 2);
      while (k > 0) {
        i = chanStart;
        n = chanStart + iheight;
        while (i < n) {
          temp_re = f->data[i + sz_idx_0].re;
          temp_im = f->data[i + sz_idx_0].im;
          f->data[i + sz_idx_0].re = f->data[i].re - temp_re;
          f->data[i + sz_idx_0].im = f->data[i].im - temp_im;
          f->data[i].re += temp_re;
          f->data[i].im += temp_im;
          i += ju;
        }

        istart = chanStart + 1;
        for (j = k; j < nRowsD2; j += k) {
          e = costab->data[j];
          twid_im = sintab->data[j];
          i = istart;
          n = istart + iheight;
          while (i < n) {
            temp_re = e * f->data[i + sz_idx_0].re - twid_im * f->data[i + sz_idx_0].im;
            temp_im = e * f->data[i + sz_idx_0].im + twid_im * f->data[i + sz_idx_0].re;
            f->data[i + sz_idx_0].re = f->data[i].re - temp_re;
            f->data[i + sz_idx_0].im = f->data[i].im - temp_im;
            f->data[i].re += temp_re;
            f->data[i].im += temp_im;
            i += ju;
          }

          istart++;
        }

        k /= 2;
        sz_idx_0 = ju;
        ju <<= 1;
        iheight -= sz_idx_0;
      }

      chanStart += n1;
    }

    emxFree_real_T(&sintab);
    emxFree_real_T(&costab);
    if (f->size[0] > 1) {
      e = 1.0 / (double)f->size[0];
      iheight = f->size[0] * f->size[1];
      emxEnsureCapacity((emxArray__common*)f, iheight, (int)sizeof(creal_T));
      sz_idx_0 = f->size[0];
      ju = f->size[1];
      sz_idx_0 *= ju;
      for (iheight = 0; iheight < sz_idx_0; iheight++) {
        f->data[iheight].re *= e;
        f->data[iheight].im *= e;
      }
    }
  }

  emxFree_creal_T(&y);
}

//
// File trailer for ifft2.cpp
//
// [EOF]
//

#if defined(__clang__)
#  pragma clang diagnostic pop
#elif defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif
