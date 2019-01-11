//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: lowpassfilter.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rt_nonfinite.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "lowpassfilter.h"
#include "logGaborFilter_emxutil.h"
#include "rdivide.h"
#include "sqrt.h"
#include "power.h"
#include "meshgrid.h"
#include "mod.h"
#include "logGaborFilter_rtwutil.h"

// Function Definitions

//
// Arguments    : emxArray_real_T *x
//                int dim
// Return Type  : void
//
void eml_ifftshift(emxArray_real_T *x, int dim)
{
  int vlend2;
  int vstride;
  int k;
  int npages;
  int vspread;
  int midoffset;
  int i2;
  int i;
  int i1;
  int j;
  int ia;
  int ib;
  double xtmp;
  int ic;
  if (x->size[dim - 1] <= 1) {
  } else {
    vlend2 = x->size[dim - 1];
    vlend2 /= 2;
    if (vlend2 << 1 == x->size[dim - 1]) {
      if (x->size[dim - 1] <= 1) {
      } else {
        vlend2 = x->size[dim - 1];
        vlend2 /= 2;
        vstride = 1;
        k = 1;
        while (k <= dim - 1) {
          vstride *= x->size[0];
          k = 2;
        }

        npages = 1;
        k = dim + 1;
        while (k < 3) {
          npages *= x->size[1];
          k = 3;
        }

        vspread = (x->size[dim - 1] - 1) * vstride;
        midoffset = vlend2 * vstride;
        if (vlend2 << 1 == x->size[dim - 1]) {
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2 - 1;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              ib = i1 + midoffset;
              for (k = 1; k <= vlend2; k++) {
                xtmp = x->data[ia];
                x->data[ia] = x->data[ib];
                x->data[ib] = xtmp;
                ia += vstride;
                ib += vstride;
              }
            }
          }
        } else {
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2 - 1;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              ib = i1 + midoffset;
              xtmp = x->data[ib];
              for (k = 1; k <= vlend2; k++) {
                ic = ib + vstride;
                x->data[ib] = x->data[ia];
                x->data[ia] = x->data[ic];
                ia += vstride;
                ib = ic;
              }

              x->data[ib] = xtmp;
            }
          }
        }
      }
    } else {
      vstride = 1;
      k = 1;
      while (k <= dim - 1) {
        vstride *= x->size[0];
        k = 2;
      }

      npages = 1;
      k = dim + 1;
      while (k < 3) {
        npages *= x->size[1];
        k = 3;
      }

      vspread = (x->size[dim - 1] - 1) * vstride;
      midoffset = vlend2 * vstride;
      i2 = -1;
      for (i = 1; i <= npages; i++) {
        i1 = i2;
        i2 += vspread;
        for (j = 1; j <= vstride; j++) {
          i1++;
          i2++;
          ia = i1 + midoffset;
          ib = i2;
          xtmp = x->data[i2];
          for (k = 1; k <= vlend2; k++) {
            ia -= vstride;
            ic = ib;
            ib -= vstride;
            x->data[ic] = x->data[ia];
            x->data[ia] = x->data[ib];
          }

          x->data[ib] = xtmp;
        }
      }
    }
  }
}

//
// Arguments    : const double sze[2]
//                emxArray_real_T *f
// Return Type  : void
//
void lowpassfilter(const double sze[2], emxArray_real_T *f)
{
  emxArray_real_T *xrange;
  double anew;
  double y;
  int n;
  double apnd;
  double ndbl;
  double cdiff;
  double absa;
  double absb;
  int k;
  int nm1d2;
  emxArray_real_T *yrange;
  emxArray_real_T *radius;
  emxArray_real_T *x;
  emxArray_real_T *b_y;
  unsigned int uv1[2];
  emxArray_real_T *r2;

  //  LOWPASSFILTER - Constructs a low-pass butterworth filter.
  //
  //  usage: f = lowpassfilter(sze, cutoff, n)
  //
  //  where: sze    is a two element vector specifying the size of filter
  //                to construct [rows cols].
  //         cutoff is the cutoff frequency of the filter 0 - 0.5
  //         n      is the order of the filter, the higher n is the sharper
  //                the transition is. (n must be an integer >= 1).
  //                Note that n is doubled so that it is always an even integer. 
  //
  //                       1
  //       f =    --------------------
  //                               2n
  //               1.0 + (w/cutoff)
  //
  //  The frequency origin of the returned filter is at the corners.
  //
  //  See also: HIGHPASSFILTER, HIGHBOOSTFILTER, BANDPASSFILTER
  //
  //  Copyright (c) 1999 Peter Kovesi
  //  School of Computer Science & Software Engineering
  //  The University of Western Australia
  //  http://www.csse.uwa.edu.au/
  //
  //  Permission is hereby granted, free of charge, to any person obtaining a copy 
  //  of this software and associated documentation files (the "Software"), to deal 
  //  in the Software without restriction, subject to the following conditions:
  //
  //  The above copyright notice and this permission notice shall be included in  
  //  all copies or substantial portions of the Software.
  //
  //  The Software is provided "as is", without warranty of any kind.
  //  October 1999
  //  August  2005 - Fixed up frequency ranges for odd and even sized filters
  //                 (previous code was a bit approximate)
  //  Set up X and Y matrices with ranges normalised to +/- 0.5
  //  The following code adjusts things appropriately for odd and even values
  //  of rows and columns.
  emxInit_real_T(&xrange, 2);
  if (b_mod(sze[1]) != 0.0) {
    anew = -(sze[1] - 1.0) / 2.0;
    y = (sze[1] - 1.0) / 2.0;
    if (rtIsNaN(anew) || rtIsNaN(y)) {
      n = 1;
      anew = rtNaN;
      apnd = y;
    } else if (y < anew) {
      n = 0;
      apnd = y;
    } else if (rtIsInf(anew) || rtIsInf(y)) {
      n = 1;
      anew = rtNaN;
      apnd = y;
    } else {
      ndbl = floor((y - anew) + 0.5);
      apnd = anew + ndbl;
      cdiff = apnd - y;
      absa = fabs(anew);
      absb = fabs(y);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = y;
      } else if (cdiff > 0.0) {
        apnd = anew + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl;
      } else {
        n = 0;
      }
    }

    k = xrange->size[0] * xrange->size[1];
    xrange->size[0] = 1;
    xrange->size[1] = n;
    emxEnsureCapacity((emxArray__common *)xrange, k, (int)sizeof(double));
    if (n > 0) {
      xrange->data[0] = anew;
      if (n > 1) {
        xrange->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          xrange->data[k] = anew + (double)k;
          xrange->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          xrange->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          xrange->data[nm1d2] = anew + (double)nm1d2;
          xrange->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = xrange->size[0] * xrange->size[1];
    xrange->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)xrange, k, (int)sizeof(double));
    nm1d2 = xrange->size[0];
    k = xrange->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      xrange->data[k] /= sze[1] - 1.0;
    }
  } else {
    anew = -sze[1] / 2.0;
    y = sze[1] / 2.0;
    ndbl = sze[1] / 2.0 - 1.0;
    if (rtIsNaN(anew) || rtIsNaN(ndbl)) {
      n = 1;
      anew = rtNaN;
      apnd = y - 1.0;
    } else if (y - 1.0 < anew) {
      n = 0;
      apnd = y - 1.0;
    } else if (rtIsInf(anew) || rtIsInf(ndbl)) {
      n = 1;
      anew = rtNaN;
      apnd = y - 1.0;
    } else {
      ndbl = floor(((y - 1.0) - anew) + 0.5);
      apnd = anew + ndbl;
      cdiff = apnd - (y - 1.0);
      absa = fabs(anew);
      absb = fabs(y - 1.0);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = y - 1.0;
      } else if (cdiff > 0.0) {
        apnd = anew + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl;
      } else {
        n = 0;
      }
    }

    k = xrange->size[0] * xrange->size[1];
    xrange->size[0] = 1;
    xrange->size[1] = n;
    emxEnsureCapacity((emxArray__common *)xrange, k, (int)sizeof(double));
    if (n > 0) {
      xrange->data[0] = anew;
      if (n > 1) {
        xrange->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          xrange->data[k] = anew + (double)k;
          xrange->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          xrange->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          xrange->data[nm1d2] = anew + (double)nm1d2;
          xrange->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = xrange->size[0] * xrange->size[1];
    xrange->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)xrange, k, (int)sizeof(double));
    nm1d2 = xrange->size[0];
    k = xrange->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      xrange->data[k] /= sze[1];
    }
  }

  emxInit_real_T(&yrange, 2);
  if (sze[0] - floor(sze[0] / 2.0) * 2.0 != 0.0) {
    anew = -(sze[0] - 1.0) / 2.0;
    y = (sze[0] - 1.0) / 2.0;
    if (rtIsNaN(anew) || rtIsNaN(y)) {
      n = 1;
      anew = rtNaN;
      apnd = y;
    } else if (y < anew) {
      n = 0;
      apnd = y;
    } else if (rtIsInf(anew) || rtIsInf(y)) {
      n = 1;
      anew = rtNaN;
      apnd = y;
    } else {
      ndbl = floor((y - anew) + 0.5);
      apnd = anew + ndbl;
      cdiff = apnd - y;
      absa = fabs(anew);
      absb = fabs(y);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = y;
      } else if (cdiff > 0.0) {
        apnd = anew + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl;
      } else {
        n = 0;
      }
    }

    k = yrange->size[0] * yrange->size[1];
    yrange->size[0] = 1;
    yrange->size[1] = n;
    emxEnsureCapacity((emxArray__common *)yrange, k, (int)sizeof(double));
    if (n > 0) {
      yrange->data[0] = anew;
      if (n > 1) {
        yrange->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          yrange->data[k] = anew + (double)k;
          yrange->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          yrange->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          yrange->data[nm1d2] = anew + (double)nm1d2;
          yrange->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = yrange->size[0] * yrange->size[1];
    yrange->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)yrange, k, (int)sizeof(double));
    nm1d2 = yrange->size[0];
    k = yrange->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      yrange->data[k] /= sze[0] - 1.0;
    }
  } else {
    anew = -sze[0] / 2.0;
    y = sze[0] / 2.0;
    ndbl = sze[0] / 2.0 - 1.0;
    if (rtIsNaN(anew) || rtIsNaN(ndbl)) {
      n = 1;
      anew = rtNaN;
      apnd = y - 1.0;
    } else if (y - 1.0 < anew) {
      n = 0;
      apnd = y - 1.0;
    } else if (rtIsInf(anew) || rtIsInf(ndbl)) {
      n = 1;
      anew = rtNaN;
      apnd = y - 1.0;
    } else {
      ndbl = floor(((y - 1.0) - anew) + 0.5);
      apnd = anew + ndbl;
      cdiff = apnd - (y - 1.0);
      absa = fabs(anew);
      absb = fabs(y - 1.0);
      if ((absa >= absb) || rtIsNaN(absb)) {
        absb = absa;
      }

      if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
        ndbl++;
        apnd = y - 1.0;
      } else if (cdiff > 0.0) {
        apnd = anew + (ndbl - 1.0);
      } else {
        ndbl++;
      }

      if (ndbl >= 0.0) {
        n = (int)ndbl;
      } else {
        n = 0;
      }
    }

    k = yrange->size[0] * yrange->size[1];
    yrange->size[0] = 1;
    yrange->size[1] = n;
    emxEnsureCapacity((emxArray__common *)yrange, k, (int)sizeof(double));
    if (n > 0) {
      yrange->data[0] = anew;
      if (n > 1) {
        yrange->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          yrange->data[k] = anew + (double)k;
          yrange->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          yrange->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          yrange->data[nm1d2] = anew + (double)nm1d2;
          yrange->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = yrange->size[0] * yrange->size[1];
    yrange->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)yrange, k, (int)sizeof(double));
    nm1d2 = yrange->size[0];
    k = yrange->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      yrange->data[k] /= sze[0];
    }
  }

  emxInit_real_T(&radius, 2);
  emxInit_real_T(&x, 2);
  emxInit_real_T(&b_y, 2);
  meshgrid(xrange, yrange, x, b_y);
  power(x, radius);
  power(b_y, x);
  k = radius->size[0] * radius->size[1];
  emxEnsureCapacity((emxArray__common *)radius, k, (int)sizeof(double));
  nm1d2 = radius->size[0];
  k = radius->size[1];
  nm1d2 *= k;
  emxFree_real_T(&yrange);
  emxFree_real_T(&xrange);
  for (k = 0; k < nm1d2; k++) {
    radius->data[k] += x->data[k];
  }

  b_sqrt(radius);

  //  A matrix with every pixel = radius relative to centre.
  b_rdivide(radius, 0.45, x);
  emxFree_real_T(&radius);
  for (k = 0; k < 2; k++) {
    uv1[k] = (unsigned int)x->size[k];
  }

  k = b_y->size[0] * b_y->size[1];
  b_y->size[0] = (int)uv1[0];
  b_y->size[1] = (int)uv1[1];
  emxEnsureCapacity((emxArray__common *)b_y, k, (int)sizeof(double));
  n = x->size[0] * x->size[1];
  for (k = 0; k + 1 <= n; k++) {
    b_y->data[k] = rt_powd_snf(x->data[k], 30.0);
  }

  emxFree_real_T(&x);
  emxInit_real_T(&r2, 2);
  k = r2->size[0] * r2->size[1];
  r2->size[0] = b_y->size[0];
  r2->size[1] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)r2, k, (int)sizeof(double));
  nm1d2 = b_y->size[0] * b_y->size[1];
  for (k = 0; k < nm1d2; k++) {
    r2->data[k] = 1.0 + b_y->data[k];
  }

  emxFree_real_T(&b_y);
  c_rdivide(r2, f);
  emxFree_real_T(&r2);
  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    eml_ifftshift(f, nm1d2 + 1);
  }

  //  The filter
}

//
// File trailer for lowpassfilter.cpp
//
// [EOF]
//
