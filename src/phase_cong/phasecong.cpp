//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: phasecong.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//

// Include Files
#include "rt_nonfinite.h"
#include "logGaborFilter.h"
#include "phasecong.h"
#include "logGaborFilter_emxutil.h"
#include "rdivide.h"
#include "acos.h"
#include "sqrt.h"
#include "power.h"
#include "atan.h"
#include "exp.h"
#include "median.h"
#include "ifft2.h"
#include "perfft2.h"
#include "logGaborFilter_rtwutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *im
//                const emxArray_real_T *lgf
//                const emxArray_creal_T *H
//                double mult
//                double k
//                double cutOff
//                double g
//                double deviationGain
//                double noiseMethod
//                emxArray_real_T *sumf (e)
//                emxArray_real_T *sumh1 (oy)
//                emxArray_real_T *sumh2 (ox)
//                emxArray_real_T *e
//                emxArray_real_T *pc
// Return Type  : void
//
void phasecong(const emxArray_real_T *im, const emxArray_real_T *lgf, const
		emxArray_creal_T *H, double mult, double k, double cutOff, double g, double
		deviationGain, double noiseMethod, emxArray_real_T *sumf, emxArray_real_T *sumh1, 
        emxArray_real_T *sumh2, emxArray_real_T *e , emxArray_real_T *pc)
{
	emxArray_creal_T *IM;
	emxArray_real_T *sumAn;
	int i9;
	int n;
	emxArray_real_T *maxAn;
	double tau;
	int s;
	emxArray_creal_T *IMF;
	emxArray_real_T *f;
	emxArray_real_T *An;
	emxArray_real_T *weight;
	emxArray_real_T *energy;
	emxArray_real_T *r11;
	emxArray_real_T *ztemp;
	emxArray_creal_T *r12;
	emxArray_real_T *r13;
	emxArray_creal_T *b_IMF;
	int loop_ub;
	int b_k;
	double extremum;
	double IMF_im;
	double H_re;
	double H_im;
	int b_sumAn[1];
	emxArray_real_T c_sumAn;
	int d_sumAn[1];
	emxArray_real_T *b_maxAn;
	emxArray_real_T *r14;
	double T;
	double totalTau;
	emxArray_real_T *b_sumh2;
	emxArray_real_T *e_sumAn;
	int iv1[2];
	emxArray_real_T *b_ztemp;
	emxArray_real_T *c_ztemp;
	emxArray_real_T *b_weight;
	emxArray_real_T *b_energy;
	emxInit_creal_T(&IM, 2);
	emxInit_real_T(&sumAn, 2);
	
	//  Used to prevent division by zero.
	perfft2(im, IM);

	//  Periodic Fourier transform of image
	i9 = sumAn->size[0] * sumAn->size[1];
	sumAn->size[0] = im->size[0];
	sumAn->size[1] = im->size[1];
	emxEnsureCapacity((emxArray__common *)sumAn, i9, (int)sizeof(double));
	n = im->size[0] * im->size[1];
	for (i9 = 0; i9 < n; i9++) {
		sumAn->data[i9] = 0.0;
	}

	//  Matrix for accumulating filter response
	//  amplitude values.
	n = im->size[0] * im->size[1];
	for (i9 = 0; i9 < n; i9++) {
		sumf->data[i9] = 0.0;
	}

	n = im->size[0] * im->size[1];
	for (i9 = 0; i9 < n; i9++) {
		sumh1->data[i9] = 0.0;
	}

	n = im->size[0] * im->size[1];
	for (i9 = 0; i9 < n; i9++) {
		sumh2->data[i9] = 0.0;
	}

	emxInit_real_T(&maxAn, 2);
	tau = 1.0;
	i9 = maxAn->size[0] * maxAn->size[1];
	maxAn->size[0] = 1;
	maxAn->size[1] = 1;
	emxEnsureCapacity((emxArray__common *)maxAn, i9, (int)sizeof(double));
	maxAn->data[0] = 1.0;
	s = 0;
	emxInit_creal_T(&IMF, 2);
	emxInit_real_T(&f, 2);
	emxInit_real_T(&An, 2);
	emxInit_real_T(&weight, 2);
	emxInit_real_T(&energy, 2);
	emxInit_real_T(&r11, 2);
	emxInit_real_T(&ztemp, 2);
	emxInit_creal_T(&r12, 2);
	emxInit_real_T(&r13, 2);
	emxInit_creal_T(&b_IMF, 2);
	while (s <= lgf->size[2] - 1) {
		i9 = IMF->size[0] * IMF->size[1];
		IMF->size[0] = IM->size[0];
		IMF->size[1] = IM->size[1];
		emxEnsureCapacity((emxArray__common *)IMF, i9, (int)sizeof(creal_T));
		n = IM->size[1];
		for (i9 = 0; i9 < n; i9++) {
			loop_ub = IM->size[0];
			for (b_k = 0; b_k < loop_ub; b_k++) {
				IMF->data[b_k + IMF->size[0] * i9].re = lgf->data[(b_k + lgf->size[0] *
					i9) + lgf->size[0] * lgf->size[1] * s] * IM->data[b_k + IM->size[0] *
					i9].re;
				IMF->data[b_k + IMF->size[0] * i9].im = lgf->data[(b_k + lgf->size[0] *
					i9) + lgf->size[0] * lgf->size[1] * s] * IM->data[b_k + IM->size[0] *
					i9].im;
			}
		}

		//  Bandpassed image in the frequency domain.
		ifft2(IMF, r12);
		i9 = f->size[0] * f->size[1];
		f->size[0] = r12->size[0];
		f->size[1] = r12->size[1];
		emxEnsureCapacity((emxArray__common *)f, i9, (int)sizeof(double));
		n = r12->size[0] * r12->size[1];
		for (i9 = 0; i9 < n; i9++) {
			f->data[i9] = r12->data[i9].re;
		}

		//  Bandpassed image in spatial domain.
		i9 = b_IMF->size[0] * b_IMF->size[1];
		b_IMF->size[0] = IMF->size[0];
		b_IMF->size[1] = IMF->size[1];
		emxEnsureCapacity((emxArray__common *)b_IMF, i9, (int)sizeof(creal_T));
		n = IMF->size[0] * IMF->size[1];
		for (i9 = 0; i9 < n; i9++) {
			extremum = IMF->data[i9].re;
			IMF_im = IMF->data[i9].im;
			H_re = H->data[i9].re;
			H_im = H->data[i9].im;
			b_IMF->data[i9].re = extremum * H_re - IMF_im * H_im;
			b_IMF->data[i9].im = extremum * H_im + IMF_im * H_re;
		}

		ifft2(b_IMF, IMF);

		//  Bandpassed monogenic filtering, real part of h contains
		//  convolution result with h1, imaginary part
		//  contains convolution result with h2.
		i9 = weight->size[0] * weight->size[1];
		weight->size[0] = IMF->size[0];
		weight->size[1] = IMF->size[1];
		emxEnsureCapacity((emxArray__common *)weight, i9, (int)sizeof(double));
		n = IMF->size[0] * IMF->size[1];
		for (i9 = 0; i9 < n; i9++) {
			weight->data[i9] = IMF->data[i9].re;
		}

		i9 = energy->size[0] * energy->size[1];
		energy->size[0] = IMF->size[0];
		energy->size[1] = IMF->size[1];
		emxEnsureCapacity((emxArray__common *)energy, i9, (int)sizeof(double));
		n = IMF->size[0] * IMF->size[1];
		for (i9 = 0; i9 < n; i9++) {
			energy->data[i9] = IMF->data[i9].im;
		}

		power(f, An);
		power(weight, r13);
		power(energy, r11);
		i9 = An->size[0] * An->size[1];
		emxEnsureCapacity((emxArray__common *)An, i9, (int)sizeof(double));
		n = An->size[0];
		b_k = An->size[1];
		n *= b_k;
		for (i9 = 0; i9 < n; i9++) {
			An->data[i9] = (An->data[i9] + r13->data[i9]) + r11->data[i9];
		}

		b_sqrt(An);

		//  Amplitude of this scale component.
		i9 = sumAn->size[0] * sumAn->size[1];
		emxEnsureCapacity((emxArray__common *)sumAn, i9, (int)sizeof(double));
		n = sumAn->size[0];
		b_k = sumAn->size[1];
		n *= b_k;
		for (i9 = 0; i9 < n; i9++) {
			sumAn->data[i9] += An->data[i9];
		}

		//  Sum of component amplitudes over scale.
		n = sumf->size[0];
		b_k = sumf->size[1];
		n *= b_k;
		for (i9 = 0; i9 < n; i9++) {
			sumf->data[i9] += f->data[i9];
		}

		n = sumh1->size[0];
		b_k = sumh1->size[1];
		n *= b_k;
		for (i9 = 0; i9 < n; i9++) {
			sumh1->data[i9] += weight->data[i9];
		}

		n = sumh2->size[0];
		b_k = sumh2->size[1];
		n *= b_k;
		for (i9 = 0; i9 < n; i9++) {
			sumh2->data[i9] += energy->data[i9];
		}

		//  At the smallest scale estimate noise characteristics from the
		//  distribution of the filter amplitude responses stored in sumAn.
		//  tau is the Rayleigh parameter that is used to describe the
		//  distribution.
		if (1 + s == 1) {
			if (noiseMethod == -1.0) {
				//  Use median to estimate noise statistics
				b_sumAn[0] = sumAn->size[0] * sumAn->size[1];
				c_sumAn = *sumAn;
				c_sumAn.size = (int *)&b_sumAn;
				c_sumAn.numDimensions = 1;
				extremum = median(&c_sumAn);
				tau = extremum / 1.1774100225154747;
			}
			else {
				if (noiseMethod == -2.0) {
					//  Use mode to estimate noise statistics
					d_sumAn[0] = sumAn->size[0] * sumAn->size[1];
					c_sumAn = *sumAn;
					c_sumAn.size = (int *)&d_sumAn;
					c_sumAn.numDimensions = 1;
					tau = rayleighmode(&c_sumAn);
				}
			}

			i9 = maxAn->size[0] * maxAn->size[1];
			maxAn->size[0] = An->size[0];
			maxAn->size[1] = An->size[1];
			emxEnsureCapacity((emxArray__common *)maxAn, i9, (int)sizeof(double));
			n = An->size[0] * An->size[1];
			for (i9 = 0; i9 < n; i9++) {
				maxAn->data[i9] = An->data[i9];
			}
		}
		else {
			//  Record maximum amplitude of components across scales.  This is needed
			//  to determine the frequency spread weighting.
			i9 = f->size[0] * f->size[1];
			f->size[0] = maxAn->size[0];
			f->size[1] = maxAn->size[1];
			emxEnsureCapacity((emxArray__common *)f, i9, (int)sizeof(double));
			n = maxAn->size[0] * maxAn->size[1];
			for (i9 = 0; i9 < n; i9++) {
				f->data[i9] = maxAn->data[i9];
			}

			if (maxAn->size[0] <= An->size[0]) {
				n = maxAn->size[0];
			}
			else {
				n = An->size[0];
			}

			if (maxAn->size[1] <= An->size[1]) {
				b_k = maxAn->size[1];
			}
			else {
				b_k = An->size[1];
			}

			i9 = ztemp->size[0] * ztemp->size[1];
			ztemp->size[0] = n;
			ztemp->size[1] = b_k;
			emxEnsureCapacity((emxArray__common *)ztemp, i9, (int)sizeof(double));
			i9 = maxAn->size[0] * maxAn->size[1];
			maxAn->size[0] = n;
			maxAn->size[1] = b_k;
			emxEnsureCapacity((emxArray__common *)maxAn, i9, (int)sizeof(double));
			n = ztemp->size[0] * ztemp->size[1];
			for (b_k = 0; b_k + 1 <= n; b_k++) {
				if ((f->data[b_k] >= An->data[b_k]) || rtIsNaN(An->data[b_k])) {
					extremum = f->data[b_k];
				}
				else {
					extremum = An->data[b_k];
				}

				maxAn->data[b_k] = extremum;
			}
		}

		s++;
	}

	emxFree_creal_T(&b_IMF);
	emxFree_creal_T(&r12);
	emxFree_real_T(&ztemp);
	emxFree_real_T(&An);
	emxFree_creal_T(&IMF);
	emxFree_creal_T(&IM);
	emxInit_real_T(&b_maxAn, 2);

	//  For each scale
	//  Form weighting that penalizes frequency distributions that are
	//  particularly narrow.  Calculate fractional 'width' of the frequencies
	//  present by taking the sum of the filter response amplitudes and dividing
	//  by the maximum component amplitude at each point on the image.  If
	//  there is only one non-zero component width takes on a value of 0, if
	//  all components are equal width is 1.
	i9 = b_maxAn->size[0] * b_maxAn->size[1];
	b_maxAn->size[0] = maxAn->size[0];
	b_maxAn->size[1] = maxAn->size[1];
	emxEnsureCapacity((emxArray__common *)b_maxAn, i9, (int)sizeof(double));
	n = maxAn->size[0] * maxAn->size[1];
	for (i9 = 0; i9 < n; i9++) {
		b_maxAn->data[i9] = maxAn->data[i9] + 0.0001;
	}

	emxFree_real_T(&maxAn);
	d_rdivide(sumAn, b_maxAn, f);
	i9 = f->size[0] * f->size[1];
	emxEnsureCapacity((emxArray__common *)f, i9, (int)sizeof(double));
	n = f->size[0];
	b_k = f->size[1];
	n *= b_k;
	emxFree_real_T(&b_maxAn);
	for (i9 = 0; i9 < n; i9++) {
		f->data[i9]--;
	}

	//  Now calculate the sigmoidal weighting function.
	b_rdivide(f, (double)lgf->size[2] - 1.0, r13);
	i9 = r13->size[0] * r13->size[1];
	emxEnsureCapacity((emxArray__common *)r13, i9, (int)sizeof(double));
	i9 = r13->size[0];
	b_k = r13->size[1];
	n = i9 * b_k;
	for (i9 = 0; i9 < n; i9++) {
		r13->data[i9] = (cutOff - r13->data[i9]) * g;
	}

	emxInit_real_T(&r14, 2);
	b_exp(r13);
	i9 = r14->size[0] * r14->size[1];
	r14->size[0] = r13->size[0];
	r14->size[1] = r13->size[1];
	emxEnsureCapacity((emxArray__common *)r14, i9, (int)sizeof(double));
	n = r13->size[0] * r13->size[1];
	for (i9 = 0; i9 < n; i9++) {
		r14->data[i9] = 1.0 + r13->data[i9];
	}

	c_rdivide(r14, weight);

	//  Automatically determine noise threshold
	//
	//  Assuming the noise is Gaussian the response of the filters to noise will
	//  form Rayleigh distribution.  We use the filter responses at the smallest
	//  scale as a guide to the underlying noise level because the smallest scale
	//  filters spend most of their time responding to noise, and only
	//  occasionally responding to features. Either the median, or the mode, of
	//  the distribution of filter responses can be used as a robust statistic to
	//  estimate the distribution mean and standard deviation as these are related 
	//  to the median or mode by fixed constants.  The response of the larger
	//  scale filters to noise can then be estimated from the smallest scale
	//  filter response according to their relative bandwidths.
	//
	//  This code assumes that the expected reponse to noise on the phase
	//  congruency calculation is simply the sum of the expected noise responses
	//  of each of the filters.  This is a simplistic overestimate, however these
	//  two quantities should be related by some constant that will depend on the
	//  filter bank being used.  Appropriate tuning of the parameter 'k' will
	//  allow you to produce the desired output. (though the value of k seems to
	//  be not at all critical)
	emxFree_real_T(&r14);
	if (noiseMethod >= 0.0) {
		//  We are using a fixed noise threshold
		T = noiseMethod;

		//  use supplied noiseMethod value as the threshold
	}
	else {
		//  Estimate the effect of noise on the sum of the filter responses as
		//  the sum of estimated individual responses (this is a simplistic
		//  overestimate). As the estimated noise response at succesive scales
		//  is scaled inversely proportional to bandwidth we have a simple
		//  geometric sum.
		totalTau = tau * (1.0 - rt_powd_snf(1.0 / mult, (double)lgf->size[2])) /
			(1.0 - 1.0 / mult);

		//  Calculate mean and std dev from tau using fixed relationship
		//  between these parameters and tau. See
		//  http://mathworld.wolfram.com/RayleighDistribution.html
		//  Expected mean and std
		//  values of noise energy
		T = totalTau * 1.2533141373155001 + k * (totalTau * 0.65513637756203358);

		//  Noise threshold
	}

	emxInit_real_T(&b_sumh2, 2);

	// ------ Final computation of key quantities -------
	

    //  Overall energy
	power(sumf, energy);
	power(sumh1, r13);
	power(sumh2, r11);
	i9 = energy->size[0] * energy->size[1];
	emxEnsureCapacity((emxArray__common *)energy, i9, (int)sizeof(double));
	n = energy->size[0];
	b_k = energy->size[1];
	n *= b_k;
	emxFree_real_T(&b_sumh2);
	for (i9 = 0; i9 < n; i9++) {
		energy->data[i9] = (energy->data[i9] + r13->data[i9]) + r11->data[i9];
	}

	emxInit_real_T(&e_sumAn, 2);
	b_sqrt(energy);

    for (i9 = 0; i9 < n; i9++) {
        e->data[i9] = energy->data[i9];
    }

	
	//  Compute phase congruency.  The original measure,
	//  PC = energy/sumAn
	//  is proportional to the weighted cos(phasedeviation).  This is not very
	//  localised so this was modified to
	//  PC = cos(phasedeviation) - |sin(phasedeviation)|
	//  (Note this was actually calculated via dot and cross products.)  This measure 
	//  approximates
	//  PC = 1 - phasedeviation.
	//  However, rather than use dot and cross products it is simpler and more
	//  efficient to simply use acos(energy/sumAn) to obtain the weighted phase
	//  deviation directly.  Note, in the expression below the noise threshold is
	//  not subtracted from energy immediately as this would interfere with the
	//  phase deviation computation.  Instead it is applied as a weighting as a
	//  fraction by which energy exceeds the noise threshold.  This weighting is
	//  applied in addition to the weighting for frequency spread.  Note also the
	//  phase deviation gain factor which acts to sharpen up the edge response. A
	//  value of 1.5 seems to work well.  Sensible values are from 1 to about 2.
	i9 = e_sumAn->size[0] * e_sumAn->size[1];
	e_sumAn->size[0] = sumAn->size[0];
	e_sumAn->size[1] = sumAn->size[1];
	emxEnsureCapacity((emxArray__common *)e_sumAn, i9, (int)sizeof(double));
	n = sumAn->size[0] * sumAn->size[1];
	for (i9 = 0; i9 < n; i9++) {
		e_sumAn->data[i9] = sumAn->data[i9] + 0.0001;
	}

	emxFree_real_T(&sumAn);
	d_rdivide(energy, e_sumAn, f);
	b_acos(f);
	i9 = f->size[0] * f->size[1];
	emxEnsureCapacity((emxArray__common *)f, i9, (int)sizeof(double));
	n = f->size[0];
	b_k = f->size[1];
	n *= b_k;
	emxFree_real_T(&e_sumAn);
	for (i9 = 0; i9 < n; i9++) {
		f->data[i9] = 1.0 - deviationGain * f->data[i9];
	}

	for (i9 = 0; i9 < 2; i9++) {
		iv1[i9] = f->size[i9];
	}

	emxInit_real_T(&b_ztemp, 2);
	i9 = b_ztemp->size[0] * b_ztemp->size[1];
	b_ztemp->size[0] = iv1[0];
	b_ztemp->size[1] = iv1[1];
	emxEnsureCapacity((emxArray__common *)b_ztemp, i9, (int)sizeof(double));
	i9 = r13->size[0] * r13->size[1];
	r13->size[0] = iv1[0];
	r13->size[1] = iv1[1];
	emxEnsureCapacity((emxArray__common *)r13, i9, (int)sizeof(double));
	n = b_ztemp->size[0] * b_ztemp->size[1];
	b_k = 0;
	emxFree_real_T(&b_ztemp);
	while (b_k + 1 <= n) {
		if (f->data[b_k] >= 0.0) {
			extremum = f->data[b_k];
		}
		else {
			extremum = 0.0;
		}

		r13->data[b_k] = extremum;
		b_k++;
	}

	i9 = f->size[0] * f->size[1];
	f->size[0] = energy->size[0];
	f->size[1] = energy->size[1];
	emxEnsureCapacity((emxArray__common *)f, i9, (int)sizeof(double));
	n = energy->size[0] * energy->size[1];
	for (i9 = 0; i9 < n; i9++) {
		f->data[i9] = energy->data[i9] - T;
	}

	for (i9 = 0; i9 < 2; i9++) {
		iv1[i9] = f->size[i9];
	}

	emxInit_real_T(&c_ztemp, 2);
	i9 = c_ztemp->size[0] * c_ztemp->size[1];
	c_ztemp->size[0] = iv1[0];
	c_ztemp->size[1] = iv1[1];
	emxEnsureCapacity((emxArray__common *)c_ztemp, i9, (int)sizeof(double));
	i9 = r11->size[0] * r11->size[1];
	r11->size[0] = iv1[0];
	r11->size[1] = iv1[1];
	emxEnsureCapacity((emxArray__common *)r11, i9, (int)sizeof(double));
	n = c_ztemp->size[0] * c_ztemp->size[1];
	b_k = 0;
	emxFree_real_T(&c_ztemp);
	while (b_k + 1 <= n) {
		if (f->data[b_k] >= 0.0) {
			extremum = f->data[b_k];
		}
		else {
			extremum = 0.0;
		}

		r11->data[b_k] = extremum;
		b_k++;
	}

	emxFree_real_T(&f);
	emxInit_real_T(&b_weight, 2);
	i9 = b_weight->size[0] * b_weight->size[1];
	b_weight->size[0] = weight->size[0];
	b_weight->size[1] = weight->size[1];
	emxEnsureCapacity((emxArray__common *)b_weight, i9, (int)sizeof(double));
	n = weight->size[0] * weight->size[1];
	for (i9 = 0; i9 < n; i9++) {
		b_weight->data[i9] = weight->data[i9] * r13->data[i9] * r11->data[i9];
	}

	emxFree_real_T(&r13);
	emxFree_real_T(&r11);
	emxFree_real_T(&weight);
	emxInit_real_T(&b_energy, 2);
	i9 = b_energy->size[0] * b_energy->size[1];
	b_energy->size[0] = energy->size[0];
	b_energy->size[1] = energy->size[1];
	emxEnsureCapacity((emxArray__common *)b_energy, i9, (int)sizeof(double));
	n = energy->size[0] * energy->size[1];
	for (i9 = 0; i9 < n; i9++) {
		b_energy->data[i9] = energy->data[i9] + 0.0001;
	}

	emxFree_real_T(&energy);
	d_rdivide(b_weight, b_energy, pc);
	emxFree_real_T(&b_energy);
	emxFree_real_T(&b_weight);
}

//
// Arguments    : const emxArray_real_T *data
// Return Type  : double
//
double rayleighmode(const emxArray_real_T *data)
{
	double rmode;
	int ixstart;
	int high_i;
	double kd;
	int nm1d2;
	boolean_T exitg2;
	double y;
	double anew;
	double apnd;
	double ndbl;
	double cdiff;
	emxArray_real_T *edges;
	int k;
	emxArray_real_T *n;
	unsigned int outsize_idx_0;
	int xind;
	int exitg1;
	int mid_i;

	// -------------------------------------------------------------------------
	//  RAYLEIGHMODE
	//
	//  Computes mode of a vector/matrix of data that is assumed to come from a
	//  Rayleigh distribution.
	//
	//  Usage:  rmode = rayleighmode(data, nbins)
	//
	//  Arguments:  data  - data assumed to come from a Rayleigh distribution
	//              nbins - Optional number of bins to use when forming histogram
	//                      of the data to determine the mode.
	//
	//  Mode is computed by forming a histogram of the data over 50 bins and then
	//  finding the maximum value in the histogram.  Mean and standard deviation
	//  can then be calculated from the mode as they are related by fixed
	//  constants.
	//
	//  mean = mode * sqrt(pi/2)
	//  std dev = mode * sqrt((4-pi)/2)
	//
	//  See
	//  http://mathworld.wolfram.com/RayleighDistribution.html
	//  http://en.wikipedia.org/wiki/Rayleigh_distribution
	//
	//  Default number of histogram bins to use
	ixstart = 1;
	high_i = data->size[0];
	kd = data->data[0];
	nm1d2 = data->size[0];
	if (nm1d2 > 1) {
		if (rtIsNaN(data->data[0])) {
			nm1d2 = 2;
			exitg2 = false;
			while ((!exitg2) && (nm1d2 <= high_i)) {
				ixstart = nm1d2;
				if (!rtIsNaN(data->data[nm1d2 - 1])) {
					kd = data->data[nm1d2 - 1];
					exitg2 = true;
				}
				else {
					nm1d2++;
				}
			}
		}

		nm1d2 = data->size[0];
		if (ixstart < nm1d2) {
			while (ixstart + 1 <= high_i) {
				if (data->data[ixstart] > kd) {
					kd = data->data[ixstart];
				}

				ixstart++;
			}
		}
	}

	y = kd / 50.0;
	if (rtIsNaN(y) || rtIsNaN(kd)) {
		high_i = 1;
		anew = rtNaN;
		apnd = kd;
	}
	else if ((y == 0.0) || ((0.0 < kd) && (y < 0.0)) || ((kd < 0.0) && (y > 0.0)))
	{
		high_i = 0;
		anew = 0.0;
		apnd = kd;
	}
	else if (rtIsInf(kd)) {
		high_i = 1;
		anew = rtNaN;
		apnd = kd;
	}
	else if (rtIsInf(y)) {
		high_i = 1;
		anew = 0.0;
		apnd = kd;
	}
	else {
		anew = 0.0;
		ndbl = floor(kd / y + 0.5);
		apnd = ndbl * y;
		if (y > 0.0) {
			cdiff = apnd - kd;
		}
		else {
			cdiff = kd - apnd;
		}

		if (fabs(cdiff) < 4.4408920985006262E-16 * fabs(kd)) {
			ndbl++;
			apnd = kd;
		}
		else if (cdiff > 0.0) {
			apnd = (ndbl - 1.0) * y;
		}
		else {
			ndbl++;
		}

		if (ndbl >= 0.0) {
			high_i = (int)ndbl;
		}
		else {
			high_i = 0;
		}
	}

	emxInit_real_T(&edges, 2);
	ixstart = edges->size[0] * edges->size[1];
	edges->size[0] = 1;
	edges->size[1] = high_i;
	emxEnsureCapacity((emxArray__common *)edges, ixstart, (int)sizeof(double));
	if (high_i > 0) {
		edges->data[0] = anew;
		if (high_i > 1) {
			edges->data[high_i - 1] = apnd;
			ixstart = high_i - 1;
			nm1d2 = ixstart / 2;
			for (k = 1; k < nm1d2; k++) {
				kd = (double)k * y;
				edges->data[k] = anew + kd;
				edges->data[(high_i - k) - 1] = apnd - kd;
			}

			if (nm1d2 << 1 == high_i - 1) {
				edges->data[nm1d2] = (anew + apnd) / 2.0;
			}
			else {
				kd = (double)nm1d2 * y;
				edges->data[nm1d2] = anew + kd;
				edges->data[nm1d2 + 1] = apnd - kd;
			}
		}
	}

	emxInit_real_T1(&n, 1);
	outsize_idx_0 = (unsigned int)edges->size[1];
	ixstart = n->size[0];
	n->size[0] = (int)outsize_idx_0;
	emxEnsureCapacity((emxArray__common *)n, ixstart, (int)sizeof(double));
	nm1d2 = (int)outsize_idx_0;
	for (ixstart = 0; ixstart < nm1d2; ixstart++) {
		n->data[ixstart] = 0.0;
	}

	xind = 0;
	k = 0;
	do {
		exitg1 = 0;
		high_i = data->size[0];
		if (k <= high_i - 1) {
			nm1d2 = 0;
			if ((!(edges->size[1] == 0)) && (!rtIsNaN(data->data[xind]))) {
				if ((data->data[xind] >= edges->data[0]) && (data->data[xind] <
					edges->data[edges->size[1] - 1])) {
					nm1d2 = 1;
					ixstart = 2;
					high_i = edges->size[1];
					while (high_i > ixstart) {
						mid_i = (nm1d2 >> 1) + (high_i >> 1);
						if (((nm1d2 & 1) == 1) && ((high_i & 1) == 1)) {
							mid_i++;
						}

						if (data->data[xind] >= edges->data[mid_i - 1]) {
							nm1d2 = mid_i;
							ixstart = mid_i + 1;
						}
						else {
							high_i = mid_i;
						}
					}
				}

				if (data->data[xind] == edges->data[edges->size[1] - 1]) {
					nm1d2 = edges->size[1];
				}
			}

			if (nm1d2 > 0) {
				n->data[nm1d2 - 1]++;
			}

			xind++;
			k++;
		}
		else {
			exitg1 = 1;
		}
	} while (exitg1 == 0);

	high_i = n->size[0];
	kd = n->data[0];
	ixstart = 1;
	if (n->size[0] > 1) {
		for (nm1d2 = 2; nm1d2 <= high_i; nm1d2++) {
			if (n->data[nm1d2 - 1] > kd) {
				kd = n->data[nm1d2 - 1];
				ixstart = nm1d2;
			}
		}
	}

	emxFree_real_T(&n);

	//  Find maximum and index of maximum in histogram
	rmode = (edges->data[ixstart - 1] + edges->data[ixstart]) / 2.0;
	emxFree_real_T(&edges);
	return rmode;
}

//
// File trailer for phasecong.cpp
//
// [EOF]
//
