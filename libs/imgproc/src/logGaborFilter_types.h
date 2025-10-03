//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: logGaborFilter_types.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Jan-2016 06:44:05
//
#ifndef __LOGGABORFILTER_TYPES_H__
#  define __LOGGABORFILTER_TYPES_H__

// Include Files
#  include "rtwtypes.h"

// Type Definitions
#  ifndef struct_emxArray__common
#    define struct_emxArray__common

struct emxArray__common {
  void* data;
  int* size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#  endif  // struct_emxArray__common

#  ifndef struct_emxArray_creal_T
#    define struct_emxArray_creal_T

struct emxArray_creal_T {
  creal_T* data;
  int* size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#  endif  // struct_emxArray_creal_T

#  ifndef struct_emxArray_int32_T
#    define struct_emxArray_int32_T

struct emxArray_int32_T {
  int* data;
  int* size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#  endif  // struct_emxArray_int32_T

#  ifndef struct_emxArray_real_T
#    define struct_emxArray_real_T

struct emxArray_real_T {
  double* data;
  int* size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#  endif  // struct_emxArray_real_T
#endif

//
// File trailer for logGaborFilter_types.h
//
// [EOF]
//
