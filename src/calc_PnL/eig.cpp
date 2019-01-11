//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eig.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "eig.h"
#include "sqrt.h"
#include "mod.h"
#include "isfinite.h"
#include "calc_PnL_rtwutil.h"

// Function Declarations
static void b_eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn);
static void b_eml_matlab_zlascl(double cfrom, double cto, creal_T A_data[], int
  A_size[1]);
static void eml_matlab_zggbal(creal_T A_data[], int A_size[2], int *ilo, int
  *ihi, int rscale_data[], int rscale_size[1]);
static void eml_matlab_zgghrd(int ilo, int ihi, creal_T A_data[], int A_size[2]);
static void eml_matlab_zhgeqz(const creal_T A_data[], const int A_size[2], int
  ilo, int ihi, int *info, creal_T alpha1_data[], int alpha1_size[1], creal_T
  beta1_data[], int beta1_size[1]);
static double eml_matlab_zlangeM(const creal_T x_data[], const int x_size[2]);
static double eml_matlab_zlanhs(const creal_T A_data[], const int A_size[2], int
  ilo, int ihi);
static void eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn, creal_T *r);
static void eml_matlab_zlascl(double cfrom, double cto, creal_T A_data[], int
  A_size[2]);
static void eml_xgeev(const creal_T A_data[], const int A_size[2], int *info,
                      creal_T alpha1_data[], int alpha1_size[1], creal_T
                      beta1_data[], int beta1_size[1]);

// Function Definitions

//
// Arguments    : const creal_T f
//                const creal_T g
//                double *cs
//                creal_T *sn
// Return Type  : void
//
static void b_eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn)
{
  double scale;
  double f2s;
  double g2;
  double fs_re;
  double fs_im;
  double gs_re;
  double gs_im;
  boolean_T guard1 = false;
  double g2s;
  scale = fabs(f.re);
  f2s = fabs(f.im);
  if (f2s > scale) {
    scale = f2s;
  }

  f2s = fabs(g.re);
  g2 = fabs(g.im);
  if (g2 > f2s) {
    f2s = g2;
  }

  if (f2s > scale) {
    scale = f2s;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));

    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    f2s = g2;
    if (1.0 > g2) {
      f2s = 1.0;
    }

    if (scale <= f2s * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        scale = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / scale;
        sn->im = -gs_im / scale;
      } else {
        g2s = sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / g2s;
        f2s = fabs(f.re);
        g2 = fabs(f.im);
        if (g2 > f2s) {
          f2s = g2;
        }

        if (f2s > 1.0) {
          scale = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / scale;
          fs_im = f.im / scale;
        } else {
          f2s = 7.4428285367870146E+137 * f.re;
          g2 = 7.4428285367870146E+137 * f.im;
          scale = rt_hypotd_snf(f2s, g2);
          fs_re = f2s / scale;
          fs_im = g2 / scale;
        }

        gs_re /= g2s;
        gs_im = -gs_im / g2s;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
      }
    } else {
      f2s = sqrt(1.0 + g2 / scale);
      *cs = 1.0 / f2s;
      scale += g2;
      fs_re = f2s * fs_re / scale;
      fs_im = f2s * fs_im / scale;
      sn->re = fs_re * gs_re - fs_im * -gs_im;
      sn->im = fs_re * -gs_im + fs_im * gs_re;
    }
  }
}

//
// Arguments    : double cfrom
//                double cto
//                creal_T A_data[]
//                int A_size[1]
// Return Type  : void
//
static void b_eml_matlab_zlascl(double cfrom, double cto, creal_T A_data[], int
  A_size[1])
{
  double cfromc;
  double ctoc;
  boolean_T notdone;
  double cfrom1;
  double cto1;
  double mul;
  int loop_ub;
  int i20;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    loop_ub = A_size[0];
    for (i20 = 0; i20 < loop_ub; i20++) {
      A_data[i20].re *= mul;
      A_data[i20].im *= mul;
    }
  }
}

//
// Arguments    : creal_T A_data[]
//                int A_size[2]
//                int *ilo
//                int *ihi
//                int rscale_data[]
//                int rscale_size[1]
// Return Type  : void
//
static void eml_matlab_zggbal(creal_T A_data[], int A_size[2], int *ilo, int
  *ihi, int rscale_data[], int rscale_size[1])
{
  int ii;
  int nzcount;
  int32_T exitg2;
  int i;
  int j;
  boolean_T found;
  boolean_T exitg5;
  int jj;
  boolean_T exitg6;
  boolean_T guard2 = false;
  int A_size_idx_1;
  creal_T b_A_data[225];
  double atmp_re;
  double atmp_im;
  int32_T exitg1;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T guard1 = false;
  rscale_size[0] = A_size[0];
  ii = A_size[0];
  for (nzcount = 0; nzcount < ii; nzcount++) {
    rscale_data[nzcount] = 0;
  }

  *ilo = 1;
  *ihi = A_size[0];
  if (A_size[0] <= 1) {
    *ihi = 1;
    rscale_data[0] = 1;
  } else {
    do {
      exitg2 = 0;
      i = 0;
      j = 0;
      found = false;
      ii = *ihi;
      exitg5 = false;
      while ((!exitg5) && (ii > 0)) {
        nzcount = 0;
        i = ii;
        j = *ihi;
        jj = 1;
        exitg6 = false;
        while ((!exitg6) && (jj <= *ihi)) {
          guard2 = false;
          if ((A_data[(ii + A_size[0] * (jj - 1)) - 1].re != 0.0) || (A_data[(ii
                + A_size[0] * (jj - 1)) - 1].im != 0.0) || (ii == jj)) {
            if (nzcount == 0) {
              j = jj;
              nzcount = 1;
              guard2 = true;
            } else {
              nzcount = 2;
              exitg6 = true;
            }
          } else {
            guard2 = true;
          }

          if (guard2) {
            jj++;
          }
        }

        if (nzcount < 2) {
          found = true;
          exitg5 = true;
        } else {
          ii--;
        }
      }

      if (!found) {
        exitg2 = 2;
      } else {
        jj = A_size[0];
        A_size_idx_1 = A_size[1];
        ii = A_size[0] * A_size[1];
        for (nzcount = 0; nzcount < ii; nzcount++) {
          b_A_data[nzcount] = A_data[nzcount];
        }

        if (i != *ihi) {
          for (ii = 0; ii + 1 <= A_size[0]; ii++) {
            atmp_re = b_A_data[(i + jj * ii) - 1].re;
            atmp_im = b_A_data[(i + jj * ii) - 1].im;
            b_A_data[(i + jj * ii) - 1] = b_A_data[(*ihi + jj * ii) - 1];
            b_A_data[(*ihi + jj * ii) - 1].re = atmp_re;
            b_A_data[(*ihi + jj * ii) - 1].im = atmp_im;
          }
        }

        if (j != *ihi) {
          for (ii = 0; ii + 1 <= *ihi; ii++) {
            atmp_re = b_A_data[ii + jj * (j - 1)].re;
            atmp_im = b_A_data[ii + jj * (j - 1)].im;
            b_A_data[ii + jj * (j - 1)] = b_A_data[ii + jj * (*ihi - 1)];
            b_A_data[ii + jj * (*ihi - 1)].re = atmp_re;
            b_A_data[ii + jj * (*ihi - 1)].im = atmp_im;
          }
        }

        A_size[0] = jj;
        A_size[1] = A_size_idx_1;
        for (nzcount = 0; nzcount < A_size_idx_1; nzcount++) {
          for (ii = 0; ii < jj; ii++) {
            A_data[ii + A_size[0] * nzcount] = b_A_data[ii + jj * nzcount];
          }
        }

        rscale_data[*ihi - 1] = j;
        (*ihi)--;
        if (*ihi == 1) {
          rscale_data[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        i = 0;
        j = 0;
        found = false;
        jj = *ilo;
        exitg3 = false;
        while ((!exitg3) && (jj <= *ihi)) {
          nzcount = 0;
          i = *ihi;
          j = jj;
          ii = *ilo;
          exitg4 = false;
          while ((!exitg4) && (ii <= *ihi)) {
            guard1 = false;
            if ((A_data[(ii + A_size[0] * (jj - 1)) - 1].re != 0.0) || (A_data
                 [(ii + A_size[0] * (jj - 1)) - 1].im != 0.0) || (ii == jj)) {
              if (nzcount == 0) {
                i = ii;
                nzcount = 1;
                guard1 = true;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              guard1 = true;
            }

            if (guard1) {
              ii++;
            }
          }

          if (nzcount < 2) {
            found = true;
            exitg3 = true;
          } else {
            jj++;
          }
        }

        if (!found) {
          exitg1 = 1;
        } else {
          jj = A_size[0];
          A_size_idx_1 = A_size[1];
          ii = A_size[0] * A_size[1];
          for (nzcount = 0; nzcount < ii; nzcount++) {
            b_A_data[nzcount] = A_data[nzcount];
          }

          if (i != *ilo) {
            for (ii = *ilo - 1; ii + 1 <= A_size[0]; ii++) {
              atmp_re = b_A_data[(i + jj * ii) - 1].re;
              atmp_im = b_A_data[(i + jj * ii) - 1].im;
              b_A_data[(i + jj * ii) - 1] = b_A_data[(*ilo + jj * ii) - 1];
              b_A_data[(*ilo + jj * ii) - 1].re = atmp_re;
              b_A_data[(*ilo + jj * ii) - 1].im = atmp_im;
            }
          }

          if (j != *ilo) {
            for (ii = 0; ii + 1 <= *ihi; ii++) {
              atmp_re = b_A_data[ii + jj * (j - 1)].re;
              atmp_im = b_A_data[ii + jj * (j - 1)].im;
              b_A_data[ii + jj * (j - 1)] = b_A_data[ii + jj * (*ilo - 1)];
              b_A_data[ii + jj * (*ilo - 1)].re = atmp_re;
              b_A_data[ii + jj * (*ilo - 1)].im = atmp_im;
            }
          }

          A_size[0] = jj;
          A_size[1] = A_size_idx_1;
          for (nzcount = 0; nzcount < A_size_idx_1; nzcount++) {
            for (ii = 0; ii < jj; ii++) {
              A_data[ii + A_size[0] * nzcount] = b_A_data[ii + jj * nzcount];
            }
          }

          rscale_data[*ilo - 1] = j;
          (*ilo)++;
          if (*ilo == *ihi) {
            rscale_data[*ilo - 1] = *ilo;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }
  }
}

//
// Arguments    : int ilo
//                int ihi
//                creal_T A_data[]
//                int A_size[2]
// Return Type  : void
//
static void eml_matlab_zgghrd(int ilo, int ihi, creal_T A_data[], int A_size[2])
{
  int jcol;
  int jrow;
  creal_T s;
  double c;
  int j;
  double stemp_re;
  double stemp_im;
  double A_data_im;
  double A_data_re;
  if ((A_size[0] <= 1) || (ihi < ilo + 2)) {
  } else {
    for (jcol = ilo - 1; jcol + 1 < ihi - 1; jcol++) {
      for (jrow = ihi - 1; jrow + 1 > jcol + 2; jrow--) {
        eml_matlab_zlartg(A_data[(jrow + A_size[0] * jcol) - 1], A_data[jrow +
                          A_size[0] * jcol], &c, &s, &A_data[(jrow + A_size[0] *
          jcol) - 1]);
        A_data[jrow + A_size[0] * jcol].re = 0.0;
        A_data[jrow + A_size[0] * jcol].im = 0.0;
        for (j = jcol + 1; j + 1 <= ihi; j++) {
          stemp_re = c * A_data[(jrow + A_size[0] * j) - 1].re + (s.re *
            A_data[jrow + A_size[0] * j].re - s.im * A_data[jrow + A_size[0] * j]
            .im);
          stemp_im = c * A_data[(jrow + A_size[0] * j) - 1].im + (s.re *
            A_data[jrow + A_size[0] * j].im + s.im * A_data[jrow + A_size[0] * j]
            .re);
          A_data_im = A_data[(jrow + A_size[0] * j) - 1].im;
          A_data_re = A_data[(jrow + A_size[0] * j) - 1].re;
          A_data[jrow + A_size[0] * j].re = c * A_data[jrow + A_size[0] * j].re
            - (s.re * A_data[(jrow + A_size[0] * j) - 1].re + s.im * A_data
               [(jrow + A_size[0] * j) - 1].im);
          A_data[jrow + A_size[0] * j].im = c * A_data[jrow + A_size[0] * j].im
            - (s.re * A_data_im - s.im * A_data_re);
          A_data[(jrow + A_size[0] * j) - 1].re = stemp_re;
          A_data[(jrow + A_size[0] * j) - 1].im = stemp_im;
        }

        s.re = -s.re;
        s.im = -s.im;
        for (j = ilo - 1; j + 1 <= ihi; j++) {
          stemp_re = c * A_data[j + A_size[0] * jrow].re + (s.re * A_data[j +
            A_size[0] * (jrow - 1)].re - s.im * A_data[j + A_size[0] * (jrow - 1)]
            .im);
          stemp_im = c * A_data[j + A_size[0] * jrow].im + (s.re * A_data[j +
            A_size[0] * (jrow - 1)].im + s.im * A_data[j + A_size[0] * (jrow - 1)]
            .re);
          A_data_im = A_data[j + A_size[0] * jrow].im;
          A_data_re = A_data[j + A_size[0] * jrow].re;
          A_data[j + A_size[0] * (jrow - 1)].re = c * A_data[j + A_size[0] *
            (jrow - 1)].re - (s.re * A_data[j + A_size[0] * jrow].re + s.im *
                              A_data[j + A_size[0] * jrow].im);
          A_data[j + A_size[0] * (jrow - 1)].im = c * A_data[j + A_size[0] *
            (jrow - 1)].im - (s.re * A_data_im - s.im * A_data_re);
          A_data[j + A_size[0] * jrow].re = stemp_re;
          A_data[j + A_size[0] * jrow].im = stemp_im;
        }
      }
    }
  }
}

//
// Arguments    : const creal_T A_data[]
//                const int A_size[2]
//                int ilo
//                int ihi
//                int *info
//                creal_T alpha1_data[]
//                int alpha1_size[1]
//                creal_T beta1_data[]
//                int beta1_size[1]
// Return Type  : void
//
static void eml_matlab_zhgeqz(const creal_T A_data[], const int A_size[2], int
  ilo, int ihi, int *info, creal_T alpha1_data[], int alpha1_size[1], creal_T
  beta1_data[], int beta1_size[1])
{
  int A_size_idx_0;
  int jp1;
  int jm1;
  creal_T b_A_data[225];
  double eshift_re;
  double eshift_im;
  creal_T ctemp;
  double rho_re;
  double rho_im;
  double anorm;
  double temp;
  double b_atol;
  double sigma2_re;
  double ascale;
  boolean_T failed;
  int j;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  int ifirst;
  int istart;
  int ilast;
  int ilastm1;
  int ifrstm;
  int ilastm;
  int iiter;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  int jiter;
  int32_T exitg1;
  boolean_T exitg3;
  boolean_T ilazro;
  boolean_T b_guard1 = false;
  creal_T t1;
  creal_T d;
  boolean_T exitg2;
  double temp2;
  double tempr;
  A_size_idx_0 = A_size[0];
  jp1 = A_size[0] * A_size[1];
  for (jm1 = 0; jm1 < jp1; jm1++) {
    b_A_data[jm1] = A_data[jm1];
  }

  *info = 0;
  if ((A_size[0] == 1) && (A_size[1] == 1)) {
    ihi = 1;
  }

  alpha1_size[0] = A_size[0];
  jp1 = A_size[0];
  for (jm1 = 0; jm1 < jp1; jm1++) {
    alpha1_data[jm1].re = 0.0;
    alpha1_data[jm1].im = 0.0;
  }

  beta1_size[0] = A_size[0];
  jp1 = A_size[0];
  for (jm1 = 0; jm1 < jp1; jm1++) {
    beta1_data[jm1].re = 1.0;
    beta1_data[jm1].im = 0.0;
  }

  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  rho_re = 0.0;
  rho_im = 0.0;
  anorm = eml_matlab_zlanhs(A_data, A_size, ilo, ihi);
  temp = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (temp > 2.2250738585072014E-308) {
    b_atol = temp;
  }

  sigma2_re = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    sigma2_re = anorm;
  }

  ascale = 1.0 / sigma2_re;
  failed = true;
  for (j = ihi; j + 1 <= A_size[0]; j++) {
    alpha1_data[j] = A_data[j + A_size[0] * j];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    ifrstm = ilo;
    ilastm = ihi;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 1;
    do {
      exitg1 = 0;
      if (jiter <= 30 * ((ihi - ilo) + 1)) {
        if (ilast + 1 == ilo) {
          goto60 = true;
        } else if (fabs(b_A_data[ilast + A_size_idx_0 * ilastm1].re) + fabs
                   (b_A_data[ilast + A_size_idx_0 * ilastm1].im) <= b_atol) {
          b_A_data[ilast + A_size_idx_0 * ilastm1].re = 0.0;
          b_A_data[ilast + A_size_idx_0 * ilastm1].im = 0.0;
          goto60 = true;
        } else {
          j = ilastm1;
          exitg3 = false;
          while ((!exitg3) && (j + 1 >= ilo)) {
            if (j + 1 == ilo) {
              ilazro = true;
            } else if (fabs(b_A_data[j + A_size_idx_0 * (j - 1)].re) + fabs
                       (b_A_data[j + A_size_idx_0 * (j - 1)].im) <= b_atol) {
              b_A_data[j + A_size_idx_0 * (j - 1)].re = 0.0;
              b_A_data[j + A_size_idx_0 * (j - 1)].im = 0.0;
              ilazro = true;
            } else {
              ilazro = false;
            }

            if (ilazro) {
              ifirst = j + 1;
              goto70 = true;
              exitg3 = true;
            } else {
              j--;
            }
          }
        }

        if (goto60 || goto70) {
          ilazro = true;
        } else {
          ilazro = false;
        }

        if (!ilazro) {
          jp1 = alpha1_size[0];
          for (jm1 = 0; jm1 < jp1; jm1++) {
            alpha1_data[jm1].re = rtNaN;
            alpha1_data[jm1].im = 0.0;
          }

          jp1 = beta1_size[0];
          for (jm1 = 0; jm1 < jp1; jm1++) {
            beta1_data[jm1].re = rtNaN;
            beta1_data[jm1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else {
          b_guard1 = false;
          if (goto60) {
            goto60 = false;
            alpha1_data[ilast] = b_A_data[ilast + A_size_idx_0 * ilast];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              failed = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              ilastm = ilast + 1;
              if (ifrstm > ilast + 1) {
                ifrstm = ilo;
              }

              b_guard1 = true;
            }
          } else {
            if (goto70) {
              goto70 = false;
              iiter++;
              ifrstm = ifirst;
              if (b_mod(iiter) != 0) {
                temp = -(b_A_data[ilast + A_size_idx_0 * ilast].re -
                         b_A_data[ilastm1 + A_size_idx_0 * ilastm1].re);
                sigma2_re = -(b_A_data[ilast + A_size_idx_0 * ilast].im -
                              b_A_data[ilastm1 + A_size_idx_0 * ilastm1].im);
                if (sigma2_re == 0.0) {
                  t1.re = temp / 2.0;
                  t1.im = 0.0;
                } else if (temp == 0.0) {
                  t1.re = 0.0;
                  t1.im = sigma2_re / 2.0;
                } else {
                  t1.re = temp / 2.0;
                  t1.im = sigma2_re / 2.0;
                }

                d.re = (t1.re * t1.re - t1.im * t1.im) + (b_A_data[ilastm1 +
                  A_size_idx_0 * ilast].re * b_A_data[ilast + A_size_idx_0 *
                  ilastm1].re - b_A_data[ilastm1 + A_size_idx_0 * ilast].im *
                  b_A_data[ilast + A_size_idx_0 * ilastm1].im);
                d.im = (t1.re * t1.im + t1.im * t1.re) + (b_A_data[ilastm1 +
                  A_size_idx_0 * ilast].re * b_A_data[ilast + A_size_idx_0 *
                  ilastm1].im + b_A_data[ilastm1 + A_size_idx_0 * ilast].im *
                  b_A_data[ilast + A_size_idx_0 * ilastm1].re);
                b_sqrt(&d);
                rho_re = b_A_data[ilastm1 + A_size_idx_0 * ilastm1].re - (t1.re
                  - d.re);
                rho_im = b_A_data[ilastm1 + A_size_idx_0 * ilastm1].im - (t1.im
                  - d.im);
                sigma2_re = b_A_data[ilastm1 + A_size_idx_0 * ilastm1].re -
                  (t1.re + d.re);
                anorm = b_A_data[ilastm1 + A_size_idx_0 * ilastm1].im - (t1.im +
                  d.im);
                if (rt_hypotd_snf(rho_re - b_A_data[ilast + A_size_idx_0 * ilast]
                                  .re, rho_im - b_A_data[ilast + A_size_idx_0 *
                                  ilast].im) <= rt_hypotd_snf(sigma2_re -
                     b_A_data[ilast + A_size_idx_0 * ilast].re, anorm -
                     b_A_data[ilast + A_size_idx_0 * ilast].im)) {
                  sigma2_re = rho_re;
                  anorm = rho_im;
                  rho_re = t1.re - d.re;
                  rho_im = t1.im - d.im;
                } else {
                  rho_re = t1.re + d.re;
                  rho_im = t1.im + d.im;
                }
              } else {
                eshift_re += b_A_data[ilast + A_size_idx_0 * ilastm1].re;
                eshift_im += b_A_data[ilast + A_size_idx_0 * ilastm1].im;
                sigma2_re = eshift_re;
                anorm = eshift_im;
              }

              j = ilastm1;
              jp1 = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                ctemp.re = b_A_data[j + A_size_idx_0 * j].re - sigma2_re;
                ctemp.im = b_A_data[j + A_size_idx_0 * j].im - anorm;
                temp = ascale * (fabs(ctemp.re) + fabs(ctemp.im));
                temp2 = ascale * (fabs(b_A_data[jp1 + A_size_idx_0 * j].re) +
                                  fabs(b_A_data[jp1 + A_size_idx_0 * j].im));
                tempr = temp;
                if (temp2 > temp) {
                  tempr = temp2;
                }

                if ((tempr < 1.0) && (tempr != 0.0)) {
                  temp /= tempr;
                  temp2 /= tempr;
                }

                if ((fabs(b_A_data[j + A_size_idx_0 * (j - 1)].re) + fabs
                     (b_A_data[j + A_size_idx_0 * (j - 1)].im)) * temp2 <= temp *
                    b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  jp1 = j;
                  j--;
                }
              }

              if (!goto90) {
                istart = ifirst;
                if (ifirst == ilastm1 + 1) {
                  ctemp.re = rho_re;
                  ctemp.im = rho_im;
                } else {
                  ctemp.re = b_A_data[(ifirst + A_size_idx_0 * (ifirst - 1)) - 1]
                    .re - sigma2_re;
                  ctemp.im = b_A_data[(ifirst + A_size_idx_0 * (ifirst - 1)) - 1]
                    .im - anorm;
                }

                goto90 = true;
              }
            }

            if (goto90) {
              goto90 = false;
              b_eml_matlab_zlartg(ctemp, b_A_data[istart + A_size_idx_0 *
                                  (istart - 1)], &temp, &t1);
              j = istart;
              jm1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  eml_matlab_zlartg(b_A_data[(j + A_size_idx_0 * jm1) - 1],
                                    b_A_data[j + A_size_idx_0 * jm1], &temp, &t1,
                                    &b_A_data[(j + A_size_idx_0 * jm1) - 1]);
                  b_A_data[j + A_size_idx_0 * jm1].re = 0.0;
                  b_A_data[j + A_size_idx_0 * jm1].im = 0.0;
                }

                for (jp1 = j - 1; jp1 + 1 <= ilastm; jp1++) {
                  d.re = temp * b_A_data[(j + A_size_idx_0 * jp1) - 1].re +
                    (t1.re * b_A_data[j + A_size_idx_0 * jp1].re - t1.im *
                     b_A_data[j + A_size_idx_0 * jp1].im);
                  d.im = temp * b_A_data[(j + A_size_idx_0 * jp1) - 1].im +
                    (t1.re * b_A_data[j + A_size_idx_0 * jp1].im + t1.im *
                     b_A_data[j + A_size_idx_0 * jp1].re);
                  sigma2_re = b_A_data[(j + A_size_idx_0 * jp1) - 1].im;
                  anorm = b_A_data[(j + A_size_idx_0 * jp1) - 1].re;
                  b_A_data[j + A_size_idx_0 * jp1].re = temp * b_A_data[j +
                    A_size_idx_0 * jp1].re - (t1.re * b_A_data[(j + A_size_idx_0
                    * jp1) - 1].re + t1.im * b_A_data[(j + A_size_idx_0 * jp1) -
                    1].im);
                  b_A_data[j + A_size_idx_0 * jp1].im = temp * b_A_data[j +
                    A_size_idx_0 * jp1].im - (t1.re * sigma2_re - t1.im * anorm);
                  b_A_data[(j + A_size_idx_0 * jp1) - 1] = d;
                }

                t1.re = -t1.re;
                t1.im = -t1.im;
                jp1 = j;
                if (ilast + 1 < j + 2) {
                  jp1 = ilast - 1;
                }

                for (jm1 = ifrstm - 1; jm1 + 1 <= jp1 + 2; jm1++) {
                  d.re = temp * b_A_data[jm1 + A_size_idx_0 * j].re + (t1.re *
                    b_A_data[jm1 + A_size_idx_0 * (j - 1)].re - t1.im *
                    b_A_data[jm1 + A_size_idx_0 * (j - 1)].im);
                  d.im = temp * b_A_data[jm1 + A_size_idx_0 * j].im + (t1.re *
                    b_A_data[jm1 + A_size_idx_0 * (j - 1)].im + t1.im *
                    b_A_data[jm1 + A_size_idx_0 * (j - 1)].re);
                  sigma2_re = b_A_data[jm1 + A_size_idx_0 * j].im;
                  anorm = b_A_data[jm1 + A_size_idx_0 * j].re;
                  b_A_data[jm1 + A_size_idx_0 * (j - 1)].re = temp *
                    b_A_data[jm1 + A_size_idx_0 * (j - 1)].re - (t1.re *
                    b_A_data[jm1 + A_size_idx_0 * j].re + t1.im * b_A_data[jm1 +
                    A_size_idx_0 * j].im);
                  b_A_data[jm1 + A_size_idx_0 * (j - 1)].im = temp *
                    b_A_data[jm1 + A_size_idx_0 * (j - 1)].im - (t1.re *
                    sigma2_re - t1.im * anorm);
                  b_A_data[jm1 + A_size_idx_0 * j] = d;
                }

                jm1 = j - 1;
                j++;
              }
            }

            b_guard1 = true;
          }

          if (b_guard1) {
            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (failed) {
      *info = ilast + 1;
      for (jp1 = 0; jp1 + 1 <= ilast + 1; jp1++) {
        alpha1_data[jp1].re = rtNaN;
        alpha1_data[jp1].im = 0.0;
        beta1_data[jp1].re = rtNaN;
        beta1_data[jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (j = 0; j + 1 < ilo; j++) {
      alpha1_data[j] = b_A_data[j + A_size_idx_0 * j];
    }
  }
}

//
// Arguments    : const creal_T x_data[]
//                const int x_size[2]
// Return Type  : double
//
static double eml_matlab_zlangeM(const creal_T x_data[], const int x_size[2])
{
  double y;
  int i4;
  int k;
  boolean_T exitg1;
  double absxk;
  y = 0.0;
  i4 = x_size[0] * x_size[1];
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i4 - 1)) {
    absxk = rt_hypotd_snf(x_data[k].re, x_data[k].im);
    if (rtIsNaN(absxk)) {
      y = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }

      k++;
    }
  }

  return y;
}

//
// Arguments    : const creal_T A_data[]
//                const int A_size[2]
//                int ilo
//                int ihi
// Return Type  : double
//
static double eml_matlab_zlanhs(const creal_T A_data[], const int A_size[2], int
  ilo, int ihi)
{
  double f;
  double scale;
  double sumsq;
  boolean_T firstNonZero;
  int j;
  int i5;
  int i;
  double reAij;
  double imAij;
  double temp2;
  f = 0.0;
  if (ilo > ihi) {
  } else {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      i5 = j + 1;
      if (ihi < j + 1) {
        i5 = ihi;
      }

      for (i = ilo; i <= i5; i++) {
        reAij = A_data[(i + A_size[0] * (j - 1)) - 1].re;
        imAij = A_data[(i + A_size[0] * (j - 1)) - 1].im;
        if (reAij != 0.0) {
          reAij = fabs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = reAij;
            firstNonZero = false;
          } else if (scale < reAij) {
            temp2 = scale / reAij;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = reAij;
          } else {
            temp2 = reAij / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          reAij = fabs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = reAij;
            firstNonZero = false;
          } else if (scale < reAij) {
            temp2 = scale / reAij;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = reAij;
          } else {
            temp2 = reAij / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    f = scale * sqrt(sumsq);
  }

  return f;
}

//
// Arguments    : const creal_T f
//                const creal_T g
//                double *cs
//                creal_T *sn
//                creal_T *r
// Return Type  : void
//
static void eml_matlab_zlartg(const creal_T f, const creal_T g, double *cs,
  creal_T *sn, creal_T *r)
{
  double scale;
  double f2s;
  double g2;
  double fs_re;
  double fs_im;
  double gs_re;
  double gs_im;
  int count;
  int rescaledir;
  boolean_T guard1 = false;
  double g2s;
  scale = fabs(f.re);
  f2s = fabs(f.im);
  if (f2s > scale) {
    scale = f2s;
  }

  f2s = fabs(g.re);
  g2 = fabs(g.im);
  if (g2 > f2s) {
    f2s = g2;
  }

  if (f2s > scale) {
    scale = f2s;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  count = 0;
  rescaledir = 0;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      count++;
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));

    rescaledir = 1;
    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        count++;
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));

      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    f2s = g2;
    if (1.0 > g2) {
      f2s = 1.0;
    }

    if (scale <= f2s * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        f2s = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / f2s;
        sn->im = -gs_im / f2s;
      } else {
        g2s = sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / g2s;
        f2s = fabs(f.re);
        g2 = fabs(f.im);
        if (g2 > f2s) {
          f2s = g2;
        }

        if (f2s > 1.0) {
          f2s = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / f2s;
          fs_im = f.im / f2s;
        } else {
          g2 = 7.4428285367870146E+137 * f.re;
          scale = 7.4428285367870146E+137 * f.im;
          f2s = rt_hypotd_snf(g2, scale);
          fs_re = g2 / f2s;
          fs_im = scale / f2s;
        }

        gs_re /= g2s;
        gs_im = -gs_im / g2s;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
        r->re = *cs * f.re + (sn->re * g.re - sn->im * g.im);
        r->im = *cs * f.im + (sn->re * g.im + sn->im * g.re);
      }
    } else {
      f2s = sqrt(1.0 + g2 / scale);
      r->re = f2s * fs_re;
      r->im = f2s * fs_im;
      *cs = 1.0 / f2s;
      f2s = scale + g2;
      g2 = r->re / f2s;
      f2s = r->im / f2s;
      sn->re = g2 * gs_re - f2s * -gs_im;
      sn->im = g2 * -gs_im + f2s * gs_re;
      if (rescaledir > 0) {
        for (rescaledir = 1; rescaledir <= count; rescaledir++) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
        }
      } else {
        if (rescaledir < 0) {
          for (rescaledir = 1; rescaledir <= count; rescaledir++) {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
          }
        }
      }
    }
  }
}

//
// Arguments    : double cfrom
//                double cto
//                creal_T A_data[]
//                int A_size[2]
// Return Type  : void
//
static void eml_matlab_zlascl(double cfrom, double cto, creal_T A_data[], int
  A_size[2])
{
  double cfromc;
  double ctoc;
  boolean_T notdone;
  double cfrom1;
  double cto1;
  double mul;
  int loop_ub;
  int i18;
  int b_loop_ub;
  int i19;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    loop_ub = A_size[1];
    for (i18 = 0; i18 < loop_ub; i18++) {
      b_loop_ub = A_size[0];
      for (i19 = 0; i19 < b_loop_ub; i19++) {
        A_data[i19 + A_size[0] * i18].re *= mul;
        A_data[i19 + A_size[0] * i18].im *= mul;
      }
    }
  }
}

//
// Arguments    : const creal_T A_data[]
//                const int A_size[2]
//                int *info
//                creal_T alpha1_data[]
//                int alpha1_size[1]
//                creal_T beta1_data[]
//                int beta1_size[1]
// Return Type  : void
//
static void eml_xgeev(const creal_T A_data[], const int A_size[2], int *info,
                      creal_T alpha1_data[], int alpha1_size[1], creal_T
                      beta1_data[], int beta1_size[1])
{
  int b_A_size[2];
  int ilo;
  int ihi;
  creal_T b_A_data[225];
  double anrm;
  boolean_T ilascl;
  double anrmto;
  int rscale_size[1];
  int rscale_data[15];
  b_A_size[0] = A_size[0];
  b_A_size[1] = A_size[1];
  ilo = A_size[0] * A_size[1];
  for (ihi = 0; ihi < ilo; ihi++) {
    b_A_data[ihi] = A_data[ihi];
  }

  *info = 0;
  anrm = eml_matlab_zlangeM(A_data, A_size);
  if (!b_isfinite(anrm)) {
    alpha1_size[0] = A_size[0];
    ilo = A_size[0];
    for (ihi = 0; ihi < ilo; ihi++) {
      alpha1_data[ihi].re = rtNaN;
      alpha1_data[ihi].im = 0.0;
    }

    beta1_size[0] = A_size[0];
    ilo = A_size[0];
    for (ihi = 0; ihi < ilo; ihi++) {
      beta1_data[ihi].re = rtNaN;
      beta1_data[ihi].im = 0.0;
    }
  } else {
    ilascl = false;
    anrmto = anrm;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      anrmto = 6.7178761075670888E-139;
      ilascl = true;
    } else {
      if (anrm > 1.4885657073574029E+138) {
        anrmto = 1.4885657073574029E+138;
        ilascl = true;
      }
    }

    if (ilascl) {
      eml_matlab_zlascl(anrm, anrmto, b_A_data, b_A_size);
    }

    eml_matlab_zggbal(b_A_data, b_A_size, &ilo, &ihi, rscale_data, rscale_size);
    eml_matlab_zgghrd(ilo, ihi, b_A_data, b_A_size);
    eml_matlab_zhgeqz(b_A_data, b_A_size, ilo, ihi, info, alpha1_data,
                      alpha1_size, beta1_data, beta1_size);
    if ((*info != 0) || (!ilascl)) {
    } else {
      b_eml_matlab_zlascl(anrmto, anrm, alpha1_data, alpha1_size);
    }
  }
}

//
// Arguments    : const creal_T A_data[]
//                const int A_size[2]
//                creal_T V_data[]
//                int V_size[1]
// Return Type  : void
//
void eig(const creal_T A_data[], const int A_size[2], creal_T V_data[], int
         V_size[1])
{
  creal_T b_A_data[225];
  int b_A_size[2];
  int info;
  int i3;
  int beta1_size[1];
  creal_T beta1_data[15];
  double V_data_re;
  double V_data_im;
  double brm;
  double bim;
  double d;
  b_A_size[0] = A_size[0];
  b_A_size[1] = A_size[1];
  info = A_size[0] * A_size[1];
  for (i3 = 0; i3 < info; i3++) {
    b_A_data[i3].re = A_data[i3].re;
    b_A_data[i3].im = A_data[i3].im;
  }

  eml_xgeev(b_A_data, b_A_size, &info, V_data, V_size, beta1_data, beta1_size);
  info = V_size[0];
  for (i3 = 0; i3 < info; i3++) {
    V_data_re = V_data[i3].re;
    V_data_im = V_data[i3].im;
    if (beta1_data[i3].im == 0.0) {
      if (V_data[i3].im == 0.0) {
        V_data[i3].re /= beta1_data[i3].re;
        V_data[i3].im = 0.0;
      } else if (V_data[i3].re == 0.0) {
        V_data[i3].re = 0.0;
        V_data[i3].im = V_data_im / beta1_data[i3].re;
      } else {
        V_data[i3].re /= beta1_data[i3].re;
        V_data[i3].im = V_data_im / beta1_data[i3].re;
      }
    } else if (beta1_data[i3].re == 0.0) {
      if (V_data[i3].re == 0.0) {
        V_data[i3].re = V_data[i3].im / beta1_data[i3].im;
        V_data[i3].im = 0.0;
      } else if (V_data[i3].im == 0.0) {
        V_data[i3].re = 0.0;
        V_data[i3].im = -(V_data_re / beta1_data[i3].im);
      } else {
        V_data[i3].re = V_data[i3].im / beta1_data[i3].im;
        V_data[i3].im = -(V_data_re / beta1_data[i3].im);
      }
    } else {
      brm = fabs(beta1_data[i3].re);
      bim = fabs(beta1_data[i3].im);
      if (brm > bim) {
        bim = beta1_data[i3].im / beta1_data[i3].re;
        d = beta1_data[i3].re + bim * beta1_data[i3].im;
        V_data[i3].re = (V_data[i3].re + bim * V_data[i3].im) / d;
        V_data[i3].im = (V_data_im - bim * V_data_re) / d;
      } else if (bim == brm) {
        if (beta1_data[i3].re > 0.0) {
          bim = 0.5;
        } else {
          bim = -0.5;
        }

        if (beta1_data[i3].im > 0.0) {
          d = 0.5;
        } else {
          d = -0.5;
        }

        V_data[i3].re = (V_data[i3].re * bim + V_data[i3].im * d) / brm;
        V_data[i3].im = (V_data_im * bim - V_data_re * d) / brm;
      } else {
        bim = beta1_data[i3].re / beta1_data[i3].im;
        d = beta1_data[i3].im + bim * beta1_data[i3].re;
        V_data[i3].re = (bim * V_data[i3].re + V_data[i3].im) / d;
        V_data[i3].im = (bim * V_data_im - V_data_re) / d;
      }
    }
  }
}

//
// File trailer for eig.cpp
//
// [EOF]
//
