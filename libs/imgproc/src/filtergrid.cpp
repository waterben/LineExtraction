//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: filtergrid.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "filtergrid.h"

#include "logGaborFilter.h"
#include "logGaborFilter_emxutil.h"
#include "lowpassfilter.h"
#include "meshgrid.h"
#include "mod.h"
#include "phasecong.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"

// Function Definitions

//
// Handle case where rows, cols has been supplied as a 2-vector
// Arguments    : double rows
//                double cols
//                emxArray_real_T *radius
//                emxArray_real_T *u1
//                emxArray_real_T *u2
// Return Type  : void
//
void filtergrid(double rows, double cols, emxArray_real_T* radius, emxArray_real_T* u1, emxArray_real_T* u2) {
  emxArray_real_T* u1range;
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
  emxArray_real_T* u2range;
  emxArray_real_T* r1;

  //  FILTERGRID Generates grid for constructing frequency domain filters
  //
  //  Usage:  [radius, u1, u2] = filtergrid(rows, cols)
  //          [radius, u1, u2] = filtergrid([rows, cols])
  //
  //  Arguments:  rows, cols - Size of image/filter
  //
  //  Returns:        radius - Grid of size [rows cols] containing normalised
  //                           radius values from 0 to 0.5.  Grid is quadrant
  //                           shifted so that 0 frequency is at radius(1,1)
  //                  u1, u2 - Grids containing normalised frequency values
  //                           ranging from -0.5 to 0.5 in x and y directions
  //                           respectively. u1 and u2 are quadrant shifted.
  //
  //  Used by PHASECONGMONO, PHASECONG3 etc etc
  //
  //  Copyright (c) 1996-2013 Peter Kovesi
  //  Centre for Exploration Targeting
  //  The University of Western Australia
  //  peter.kovesi at uwa edu au
  //
  //  Permission is hereby granted, free of charge, to any person obtaining a copy
  //  of this software and associated documentation files (the "Software"), to deal
  //  in the Software without restriction, subject to the following conditions:
  //
  //  The above copyright notice and this permission notice shall be included in
  //  all copies or substantial portions of the Software.
  //
  //  The Software is provided "as is", without warranty of any kind.
  //
  //  May 2013
  //  Set up X and Y spatial frequency matrices, u1 and u2, with ranges
  //  normalised to +/- 0.5 The following code adjusts things appropriately for
  //  odd and even values of rows and columns so that the 0 frequency point is
  //  placed appropriately.
  emxInit_real_T(&u1range, 2);
  if (b_mod(cols) != 0.0) {
    anew = -(cols - 1.0) / 2.0;
    y = (cols - 1.0) / 2.0;
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

    k = u1range->size[0] * u1range->size[1];
    u1range->size[0] = 1;
    u1range->size[1] = n;
    emxEnsureCapacity((emxArray__common*)u1range, k, (int)sizeof(double));
    if (n > 0) {
      u1range->data[0] = anew;
      if (n > 1) {
        u1range->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          u1range->data[k] = anew + (double)k;
          u1range->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          u1range->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          u1range->data[nm1d2] = anew + (double)nm1d2;
          u1range->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = u1range->size[0] * u1range->size[1];
    u1range->size[0] = 1;
    emxEnsureCapacity((emxArray__common*)u1range, k, (int)sizeof(double));
    nm1d2 = u1range->size[0];
    k = u1range->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      u1range->data[k] /= cols - 1.0;
    }
  } else {
    anew = -cols / 2.0;
    y = cols / 2.0;
    ndbl = cols / 2.0 - 1.0;
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

    k = u1range->size[0] * u1range->size[1];
    u1range->size[0] = 1;
    u1range->size[1] = n;
    emxEnsureCapacity((emxArray__common*)u1range, k, (int)sizeof(double));
    if (n > 0) {
      u1range->data[0] = anew;
      if (n > 1) {
        u1range->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          u1range->data[k] = anew + (double)k;
          u1range->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          u1range->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          u1range->data[nm1d2] = anew + (double)nm1d2;
          u1range->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = u1range->size[0] * u1range->size[1];
    u1range->size[0] = 1;
    emxEnsureCapacity((emxArray__common*)u1range, k, (int)sizeof(double));
    nm1d2 = u1range->size[0];
    k = u1range->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      u1range->data[k] /= cols;
    }
  }

  emxInit_real_T(&u2range, 2);
  if (rows - floor(rows / 2.0) * 2.0 != 0.0) {
    anew = -(rows - 1.0) / 2.0;
    y = (rows - 1.0) / 2.0;
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

    k = u2range->size[0] * u2range->size[1];
    u2range->size[0] = 1;
    u2range->size[1] = n;
    emxEnsureCapacity((emxArray__common*)u2range, k, (int)sizeof(double));
    if (n > 0) {
      u2range->data[0] = anew;
      if (n > 1) {
        u2range->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          u2range->data[k] = anew + (double)k;
          u2range->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          u2range->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          u2range->data[nm1d2] = anew + (double)nm1d2;
          u2range->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = u2range->size[0] * u2range->size[1];
    u2range->size[0] = 1;
    emxEnsureCapacity((emxArray__common*)u2range, k, (int)sizeof(double));
    nm1d2 = u2range->size[0];
    k = u2range->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      u2range->data[k] /= rows - 1.0;
    }
  } else {
    anew = -rows / 2.0;
    y = rows / 2.0;
    ndbl = rows / 2.0 - 1.0;
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

    k = u2range->size[0] * u2range->size[1];
    u2range->size[0] = 1;
    u2range->size[1] = n;
    emxEnsureCapacity((emxArray__common*)u2range, k, (int)sizeof(double));
    if (n > 0) {
      u2range->data[0] = anew;
      if (n > 1) {
        u2range->data[n - 1] = apnd;
        k = n - 1;
        nm1d2 = k / 2;
        for (k = 1; k < nm1d2; k++) {
          u2range->data[k] = anew + (double)k;
          u2range->data[(n - k) - 1] = apnd - (double)k;
        }

        if (nm1d2 << 1 == n - 1) {
          u2range->data[nm1d2] = (anew + apnd) / 2.0;
        } else {
          u2range->data[nm1d2] = anew + (double)nm1d2;
          u2range->data[nm1d2 + 1] = apnd - (double)nm1d2;
        }
      }
    }

    k = u2range->size[0] * u2range->size[1];
    u2range->size[0] = 1;
    emxEnsureCapacity((emxArray__common*)u2range, k, (int)sizeof(double));
    nm1d2 = u2range->size[0];
    k = u2range->size[1];
    nm1d2 *= k;
    for (k = 0; k < nm1d2; k++) {
      u2range->data[k] /= rows;
    }
  }

  meshgrid(u1range, u2range, u1, u2);

  //  Quadrant shift so that filters are constructed with 0 frequency at
  //  the corners
  emxFree_real_T(&u2range);
  emxFree_real_T(&u1range);
  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    eml_ifftshift(u1, nm1d2 + 1);
  }

  for (nm1d2 = 0; nm1d2 < 2; nm1d2++) {
    eml_ifftshift(u2, nm1d2 + 1);
  }

  emxInit_real_T(&r1, 2);

  //  Construct spatial frequency values in terms of normalised radius from
  //  centre.
  power(u1, radius);
  power(u2, r1);
  k = radius->size[0] * radius->size[1];
  emxEnsureCapacity((emxArray__common*)radius, k, (int)sizeof(double));
  nm1d2 = radius->size[0];
  k = radius->size[1];
  nm1d2 *= k;
  for (k = 0; k < nm1d2; k++) {
    radius->data[k] += r1->data[k];
  }

  emxFree_real_T(&r1);
  b_sqrt(radius);
}

//
// File trailer for filtergrid.cpp
//
// [EOF]
//
