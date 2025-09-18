//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: perfft2.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rt_nonfinite.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "perfft2.h"
#include "logGaborFilter_emxutil.h"
#include "fft2.h"
#include "rdivide.h"
#include "cos.h"
#include "meshgrid.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *im
//                emxArray_creal_T *P
// Return Type  : void
//
void perfft2(const emxArray_real_T *im, emxArray_creal_T *P)
{
  unsigned int uv2[2];
  int i5;
  emxArray_real_T *s;
  int cdiff;
  int nm1d2;
  emxArray_real_T *b_s;
  int ndbl;
  emxArray_real_T *c_s;
  emxArray_real_T *d_s;
  int apnd;
  emxArray_real_T *y;
  emxArray_real_T *b_y;
  emxArray_real_T *r7;
  emxArray_real_T *r8;
  emxArray_real_T *cx;
  emxArray_real_T *cy;
  emxArray_creal_T *r9;
  emxArray_real_T *r10;
  emxArray_creal_T *S;

  //  PERFFT2  2D Fourier transform of Moisan's periodic image component
  //
  //  Usage: [P, S, p, s] = perfft2(im)
  //
  //  Argument:  im - Image to be transformed
  //  Returns:    P - 2D fft of periodic image component
  //              S - 2D fft of smooth component
  //              p - Periodic component (spatial domain)
  //              s - Smooth component (spatial domain)
  //
  //  Moisan's "Periodic plus Smooth Image Decomposition" decomposes an image
  //  into two components
  //         im = p + s
  //  where s is the 'smooth' component with mean 0 and p is the 'periodic'
  //  component which has no sharp discontinuities when one moves cyclically across 
  //  the image boundaries.
  //
  //  This wonderful decomposition is very useful when one wants to obtain an FFT of 
  //  an image with minimal artifacts introduced from the boundary discontinuities. 
  //  The image p gathers most of the image information but avoids periodization 
  //  artifacts.
  //
  //  The typical use of this function is to obtain a 'periodic only' fft of an
  //  image
  //    >>  P = perfft2(im);
  //
  //  Displaying the amplitude spectrum of P will yield a clean spectrum without the 
  //  typical vertical-horizontal 'cross' arising from the image boundaries that you 
  //  would normally see.
  //
  //  Note if you are using the function to perform filtering in the frequency
  //  domain you may want to retain s (the smooth component in the spatial domain) 
  //  and add it back to the filtered result at the end.
  //
  //  The computational cost of obtaining the 'periodic only' FFT involves taking an 
  //  additional FFT.
  //
  //
  //  Reference:
  //  This code is adapted from Lionel Moisan's Scilab function 'perdecomp.sci'  
  //  "Periodic plus Smooth Image Decomposition" 07/2012 available at
  //
  //    http://www.mi.parisdescartes.fr/~moisan/p+s
  //
  //  Paper:
  //  L. Moisan, "Periodic plus Smooth Image Decomposition", Journal of
  //  Mathematical Imaging and Vision, vol 39:2, pp. 161-179, 2011.
  //  Peter Kovesi
  //  Centre for Exploration Targeting
  //  The University of Western Australia
  //  peter.kovesi at uwa edu au
  //  September 2012
  //  Compute the boundary image which is equal to the image discontinuity
  //  values across the boundaries at the edges and is 0 elsewhere
  for (i5 = 0; i5 < 2; i5++) {
    uv2[i5] = (unsigned int)im->size[i5];
  }

  emxInit_real_T(&s, 2);
  i5 = s->size[0] * s->size[1];
  s->size[0] = (int)uv2[0];
  s->size[1] = (int)uv2[1];
  emxEnsureCapacity((emxArray__common *)s, i5, (int)sizeof(double));
  cdiff = (int)uv2[0] * (int)uv2[1];
  for (i5 = 0; i5 < cdiff; i5++) {
    s->data[i5] = 0.0;
  }

  cdiff = im->size[1] - 1;
  nm1d2 = im->size[0];
  for (i5 = 0; i5 <= cdiff; i5++) {
    s->data[s->size[0] * i5] = im->data[im->size[0] * i5] - im->data[(nm1d2 +
      im->size[0] * i5) - 1];
  }

  emxInit_real_T(&b_s, 2);
  cdiff = s->size[1];
  ndbl = s->size[0] - 1;
  i5 = b_s->size[0] * b_s->size[1];
  b_s->size[0] = 1;
  b_s->size[1] = cdiff;
  emxEnsureCapacity((emxArray__common *)b_s, i5, (int)sizeof(double));
  for (i5 = 0; i5 < cdiff; i5++) {
    b_s->data[b_s->size[0] * i5] = -s->data[s->size[0] * i5];
  }

  cdiff = b_s->size[1];
  for (i5 = 0; i5 < cdiff; i5++) {
    s->data[ndbl + s->size[0] * i5] = b_s->data[b_s->size[0] * i5];
  }

  emxFree_real_T(&b_s);
  emxInit_real_T1(&c_s, 1);
  cdiff = s->size[0];
  nm1d2 = im->size[1];
  i5 = c_s->size[0];
  c_s->size[0] = cdiff;
  emxEnsureCapacity((emxArray__common *)c_s, i5, (int)sizeof(double));
  for (i5 = 0; i5 < cdiff; i5++) {
    c_s->data[i5] = (s->data[i5] + im->data[i5]) - im->data[i5 + im->size[0] *
      (nm1d2 - 1)];
  }

  cdiff = c_s->size[0];
  for (i5 = 0; i5 < cdiff; i5++) {
    s->data[i5] = c_s->data[i5];
  }

  emxFree_real_T(&c_s);
  emxInit_real_T1(&d_s, 1);
  cdiff = s->size[0];
  ndbl = s->size[1] - 1;
  nm1d2 = im->size[1];
  apnd = s->size[1] - 1;
  i5 = d_s->size[0];
  d_s->size[0] = cdiff;
  emxEnsureCapacity((emxArray__common *)d_s, i5, (int)sizeof(double));
  for (i5 = 0; i5 < cdiff; i5++) {
    d_s->data[i5] = (s->data[i5 + s->size[0] * ndbl] - im->data[i5]) + im->
      data[i5 + im->size[0] * (nm1d2 - 1)];
  }

  cdiff = d_s->size[0];
  for (i5 = 0; i5 < cdiff; i5++) {
    s->data[i5 + s->size[0] * apnd] = d_s->data[i5];
  }

  emxFree_real_T(&d_s);

  //  Generate grid upon which to compute the filter for the boundary image in
  //  the frequency domain.  Note that cos() is cyclic hence the grid values can 
  //  range from 0 .. 2*pi rather than 0 .. pi and then pi .. 0
  if (im->size[1] - 1 < 0) {
    ndbl = 0;
    apnd = -1;
  } else {
    ndbl = (int)floor(((double)im->size[1] - 1.0) + 0.5);
    apnd = ndbl;
    cdiff = (ndbl - im->size[1]) + 1;
    if (fabs((double)cdiff) < 4.4408920985006262E-16 * fabs((double)im->size[1]
         - 1.0)) {
      ndbl++;
      apnd = im->size[1] - 1;
    } else if (cdiff > 0) {
      apnd = ndbl - 1;
    } else {
      ndbl++;
    }
  }

  emxInit_real_T(&y, 2);
  i5 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = ndbl;
  emxEnsureCapacity((emxArray__common *)y, i5, (int)sizeof(double));
  if (ndbl > 0) {
    y->data[0] = 0.0;
    if (ndbl > 1) {
      y->data[ndbl - 1] = apnd;
      i5 = ndbl - 1;
      nm1d2 = i5 / 2;
      for (cdiff = 1; cdiff < nm1d2; cdiff++) {
        y->data[cdiff] = cdiff;
        y->data[(ndbl - cdiff) - 1] = apnd - cdiff;
      }

      if (nm1d2 << 1 == ndbl - 1) {
        y->data[nm1d2] = (double)apnd / 2.0;
      } else {
        y->data[nm1d2] = nm1d2;
        y->data[nm1d2 + 1] = apnd - nm1d2;
      }
    }
  }

  if (im->size[0] - 1 < 0) {
    ndbl = 0;
    apnd = -1;
  } else {
    ndbl = (int)floor(((double)im->size[0] - 1.0) + 0.5);
    apnd = ndbl;
    cdiff = (ndbl - im->size[0]) + 1;
    if (fabs((double)cdiff) < 4.4408920985006262E-16 * fabs((double)im->size[0]
         - 1.0)) {
      ndbl++;
      apnd = im->size[0] - 1;
    } else if (cdiff > 0) {
      apnd = ndbl - 1;
    } else {
      ndbl++;
    }
  }

  emxInit_real_T(&b_y, 2);
  i5 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = ndbl;
  emxEnsureCapacity((emxArray__common *)b_y, i5, (int)sizeof(double));
  if (ndbl > 0) {
    b_y->data[0] = 0.0;
    if (ndbl > 1) {
      b_y->data[ndbl - 1] = apnd;
      i5 = ndbl - 1;
      nm1d2 = i5 / 2;
      for (cdiff = 1; cdiff < nm1d2; cdiff++) {
        b_y->data[cdiff] = cdiff;
        b_y->data[(ndbl - cdiff) - 1] = apnd - cdiff;
      }

      if (nm1d2 << 1 == ndbl - 1) {
        b_y->data[nm1d2] = (double)apnd / 2.0;
      } else {
        b_y->data[nm1d2] = nm1d2;
        b_y->data[nm1d2 + 1] = apnd - nm1d2;
      }
    }
  }

  emxInit_real_T(&r7, 2);
  i5 = r7->size[0] * r7->size[1];
  r7->size[0] = 1;
  r7->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)r7, i5, (int)sizeof(double));
  nm1d2 = im->size[1];
  cdiff = y->size[0] * y->size[1];
  for (i5 = 0; i5 < cdiff; i5++) {
    r7->data[i5] = 6.2831853071795862 * y->data[i5] / (double)nm1d2;
  }

  emxFree_real_T(&y);
  emxInit_real_T(&r8, 2);
  i5 = r8->size[0] * r8->size[1];
  r8->size[0] = 1;
  r8->size[1] = b_y->size[1];
  emxEnsureCapacity((emxArray__common *)r8, i5, (int)sizeof(double));
  nm1d2 = im->size[0];
  cdiff = b_y->size[0] * b_y->size[1];
  for (i5 = 0; i5 < cdiff; i5++) {
    r8->data[i5] = 6.2831853071795862 * b_y->data[i5] / (double)nm1d2;
  }

  emxFree_real_T(&b_y);
  emxInit_real_T(&cx, 2);
  emxInit_real_T(&cy, 2);
  emxInit_creal_T(&r9, 2);
  emxInit_real_T(&r10, 2);
  meshgrid(r7, r8, cx, cy);

  //  Generate FFT of smooth component
  b_cos(cx);
  b_cos(cy);
  fft2(s, r9);
  i5 = r10->size[0] * r10->size[1];
  r10->size[0] = cx->size[0];
  r10->size[1] = cx->size[1];
  emxEnsureCapacity((emxArray__common *)r10, i5, (int)sizeof(double));
  cdiff = cx->size[0] * cx->size[1];
  emxFree_real_T(&r8);
  emxFree_real_T(&r7);
  emxFree_real_T(&s);
  for (i5 = 0; i5 < cdiff; i5++) {
    r10->data[i5] = 2.0 * ((2.0 - cx->data[i5]) - cy->data[i5]);
  }

  emxFree_real_T(&cy);
  emxFree_real_T(&cx);
  emxInit_creal_T(&S, 2);
  rdivide(r9, r10, S);

  //  The (1,1) element of the filter will be 0 so S(1,1) may be Inf or NaN
  S->data[0].re = 0.0;
  S->data[0].im = 0.0;

  //  Enforce 0 mean
  fft2(im, P);
  i5 = P->size[0] * P->size[1];
  emxEnsureCapacity((emxArray__common *)P, i5, (int)sizeof(creal_T));
  cdiff = P->size[0];
  nm1d2 = P->size[1];
  cdiff *= nm1d2;
  emxFree_real_T(&r10);
  emxFree_creal_T(&r9);
  for (i5 = 0; i5 < cdiff; i5++) {
    P->data[i5].re -= S->data[i5].re;
    P->data[i5].im -= S->data[i5].im;
  }

  emxFree_creal_T(&S);

  //  FFT of periodic component
}

//
// File trailer for perfft2.cpp
//
// [EOF]
//
