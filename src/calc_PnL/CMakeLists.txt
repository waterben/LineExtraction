cmake_minimum_required(VERSION 2.6)

# Add Libraries

add_library(lib_calc_PnL ${LIBRARY_TYPE} abs.cpp calcampose.cpp calc_PnL.cpp calc_PnL_emxAPI.cpp calc_PnL_emxutil.cpp  calc_PnL_initialize.cpp calc_PnL_rtwutil.cpp calc_PnL_terminate.cpp camParams.cpp cross.cpp det.cpp eig.cpp eye.cpp isfinite.cpp matrixFromPoints.cpp mean.cpp mod.cpp norm.cpp PnL.cpp polyval.cpp R_and_T.cpp roots.cpp rtGetInf.cpp rtGetNaN.cpp rt_nonfinite.cpp sign.cpp sqrt.cpp svd.cpp interface/_coder_calc_PnL_info.c mpower.cpp power.cpp)

#target_link_libraries(lib_calc_PnL lib_lsd lib_line_tracker ${OpenCV_LIBS})

