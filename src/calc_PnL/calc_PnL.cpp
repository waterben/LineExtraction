//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: calc_PnL.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 22-Jul-2015 17:51:38
//

// Include Files
#include "rt_nonfinite.h"
#include "PnL.h"
#include "R_and_T.h"
#include "calc_PnL.h"
#include "norm.h"
#include "camParams.h"
#include "matrixFromPoints.h"

// Function Definitions

//
// test the pose estimation from line correspondences problem
// Arguments    : void
// Return Type  : void
//
void calc_PnL()
{
  double startPoint3D_w[36];
  double endPoint3D_w[36];
  double V[36];
  int i;
  double RPnLppT_cw_data[3];
  double a[3];
  double dir[3];
  int i0;
  int i1;
  static const double startPoint3D[36] = { 1.39917, -1.37095, 0.27404, -0.99917,
    0.49774, 1.39917, 1.39917, -1.37095, 0.27404, 0.49774, -0.99917, -0.99917,
    -0.81133, -0.59523, 1.70284, 0.81133, -0.29626, -0.81133, -0.81133, -0.59523,
    1.70284, -0.29626, 0.81133, 0.81133, 3.15065, 2.62171, 2.50799, 1.24935,
    0.51965, 3.15065, 3.15065, 2.62171, 2.50799, 0.51965, 1.24935, 1.24935 };

  static const double b_a[9] = { 6.123233995736766E-17, 1.0, 0.0, -1.0,
    6.123233995736766E-17, 0.0, 0.0, 0.0, 1.0 };

  static const signed char T_cw[3] = { 0, 0, -5 };

  static const double endPoint3D[36] = { 1.77095, 0.12596, -0.09774, 0.49774,
    0.12596, 0.12596, -0.09774, -0.09774, 1.77095, 1.77095, -1.37095, 0.27404,
    0.59523, -1.70284, 0.29628, -0.29626, -1.70284, -1.70284, 0.29628, 0.29628,
    0.59523, 0.59523, -0.59523, 1.70284, 1.77829, 1.89201, 3.88035, 0.51965,
    1.89201, 1.89201, 3.88035, 3.88035, 1.77829, 1.77829, 2.62171, 2.50799 };

  double c_ox;
  double p3Dw1[96];
  double xs[36];
  double xe[36];
  static const double startPoint2d[24] = { 0.027380952380952381,
    -0.1380952380952381, 0.11166666666666666, -0.019285714285714285,
    0.0011904761904761906, 0.027380952380952381, 0.027380952380952381,
    -0.1380952380952381, 0.11166666666666666, 0.0011904761904761906,
    -0.019285714285714285, -0.019285714285714285, 0.022142857142857141,
    0.061666666666666668, 0.069523809523809529, -0.016904761904761905,
    -0.15476190476190477, 0.022142857142857141, 0.022142857142857141,
    0.061666666666666668, 0.069523809523809529, -0.15476190476190477,
    -0.016904761904761905, -0.016904761904761905 };

  static const double endPoint2d[24] = { 0.15095238095238095,
    -0.13095238095238096, -0.00095238095238095238, 0.0011904761904761906,
    -0.13095238095238096, -0.13095238095238096, -0.00095238095238095238,
    -0.00095238095238095238, 0.15095238095238095, 0.15095238095238095,
    -0.1380952380952381, 0.11166666666666666, -0.068571428571428575,
    -0.0830952380952381, 0.16523809523809524, -0.15476190476190477,
    -0.0830952380952381, -0.0830952380952381, 0.16523809523809524,
    0.16523809523809524, -0.068571428571428575, -0.068571428571428575,
    0.061666666666666668, 0.069523809523809529 };

  double b_xs[36];
  double b_xe[36];
  double b_V[36];
  double b_startPoint3D_w[36];
  int RPnLR_cw_size[2];
  double RPnLR_cw_data[9];
  int RPnLppT_cw_size[1];
  int RPnLppR_cw_size[2];
  double RPnLppR_cw_data[9];
  double c_M[12];
  static const double dv0[48] = { 2035.0, 1173.0, 1340.0, 1339.0, 2389.0, 1372.0,
    1839.0, 1009.0, 1925.0, 430.0, 2035.0, 1173.0, 2035.0, 1173.0, 1340.0,
    1339.0, 2389.0, 1372.0, 1925.0, 430.0, 1839.0, 1009.0, 1839.0, 1009.0,
    2554.0, 792.0, 1370.0, 731.0, 1916.0, 1774.0, 1925.0, 430.0, 1370.0, 731.0,
    1370.0, 731.0, 1916.0, 1774.0, 1916.0, 1774.0, 2554.0, 792.0, 2554.0, 792.0,
    1340.0, 1339.0, 2389.0, 1372.0 };

  double c_fy;
  double c_fx;
  double c_oy;

  // please set the number of lines (numOfLines>=4)
  // please set the Std of image noise. (noiseStd = 0 or 10 pixels)
  // Rotation matrix from camera to world frame R_cw
  // 2*rand(1)-1;
  // 2*rand(1)-1;
  // 2*rand(1)-1;
  //  translation from camera to world frame T_cw
  // T_cw = 2*rand(3,1);
  // % Known camera parameters
  //  field of view angle in rad (rad = deg*pi/180)
  // fov = 60/180*pi;
  // fov = 1.5708; % => f=1
  //  dim of image plane
  // fov = 2 * atan(w/2*f);
  // f = (w/2)/tan(fov/2);
  //       % p1 p2
  //      % p6 p4
  //        % p8 p5
  //       % p7 p3
  //       % p3 p4
  //       % p1 p4
  //       % p1 p5
  //      % p6 p5
  //        % p8 p2
  //       % p3 p2
  //       % p7 p6
  //  p7 p8
  //      % p1 p2
  //     % p6 p4
  //     % p8 p5
  //     % p7 p3
  //     % p3 p4
  //     % p1 p4
  //     % p1 p5
  //     % p6 p5
  //      % p8 p2
  //      % p3 p2
  //    % p7 p6
  //  p7 p8
  //       % p1 p2
  //      % p6 p4
  //      % p8 p5
  //      % p7 p3
  //       % p3 p4
  //       % p1 p4
  //       % p1 p5
  //      % p6 p5
  //      % p8 p2
  //       % p3 p2
  //      % p7 p6
  //  p7 p8
  //     % p1 p2
  //     % p6 p4
  //     % p8 p5
  //     % p7 p3
  //     % p3 p4
  //     % p1 p4
  //    % p1 p5
  //    % p6 p5
  //    % p8 p2
  //    % p3 p2
  //   % p7 p6
  //  p7 p8
  //  Compute M from given camera parameters
  // M2 = camMatrix(R_cw',T_cw,fov, w, h);
  //  eMat(1:4,:);
  //  eMat(5:8,:);
  // generate the 3D line direction V and point P in world frame
  for (i = 0; i < 12; i++) {
    // P_w = R_cw * P_c + T_cw i.e. P_c = R_wc (P_w - T_cw);
    for (i0 = 0; i0 < 3; i0++) {
      RPnLppT_cw_data[i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        RPnLppT_cw_data[i0] += b_a[i0 + 3 * i1] * startPoint3D[i + 12 * i1];
      }

      startPoint3D_w[i + 12 * i0] = RPnLppT_cw_data[i0] + (double)T_cw[i0];
      a[i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        a[i0] += b_a[i0 + 3 * i1] * endPoint3D[i + 12 * i1];
      }

      endPoint3D_w[i + 12 * i0] = a[i0] + (double)T_cw[i0];
      startPoint3D_w[i + 12 * i0] = startPoint3D[i + 12 * i0];

      // P_w = R_cw * P_c + T_cw i.e. P_c = R_wc (P_w - T_cw);
      endPoint3D_w[i + 12 * i0] = endPoint3D[i + 12 * i0];

      //     dir = R_cw*(endPoint3D(i,:) - startPoint3D(i,:))';         % == (endPoint3D_w(i,:) - startPoint3D_w(i,:))' 
      dir[i0] = endPoint3D[i + 12 * i0] - startPoint3D[i + 12 * i0];
    }

    c_ox = norm(dir);
    for (i0 = 0; i0 < 3; i0++) {
      V[i + 12 * i0] = dir[i0] / c_ox;
    }
  }

  for (i0 = 0; i0 < 12; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      p3Dw1[i1 + (i0 << 2)] = startPoint3D_w[i0 + 12 * i1];
    }
  }

  for (i0 = 0; i0 < 12; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      p3Dw1[i1 + ((i0 + 12) << 2)] = endPoint3D_w[i0 + 12 * i1];
    }
  }

  for (i0 = 0; i0 < 24; i0++) {
    p3Dw1[3 + (i0 << 2)] = 1.0;
  }

  // 49.134 grad;
  //  f und das offset rausrechnen
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 2; i0++) {
      RPnLppT_cw_data[i0] = startPoint2d[i + 12 * i0];
    }

    RPnLppT_cw_data[2] = 1.0;
    for (i0 = 0; i0 < 3; i0++) {
      xs[i0 + 3 * i] = RPnLppT_cw_data[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      RPnLppT_cw_data[i0] = endPoint2d[i + 12 * i0];
    }

    RPnLppT_cw_data[2] = 1.0;
    for (i0 = 0; i0 < 3; i0++) {
      xe[i0 + 3 * i] = RPnLppT_cw_data[i0];
    }
  }

  // call the PnL() function to estimate camera pose.
  for (i0 = 0; i0 < 36; i0++) {
    b_xs[i0] = xs[i0];
    b_xe[i0] = xe[i0];
  }

  for (i0 = 0; i0 < 12; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_V[i1 + 3 * i0] = V[i0 + 12 * i1];
      b_startPoint3D_w[i1 + 3 * i0] = startPoint3D_w[i0 + 12 * i1];
    }
  }

  b_PnL(b_xs, b_xe, b_V, b_startPoint3D_w, RPnLR_cw_data, RPnLR_cw_size, dir);

  // xs
  // xe
  // call the R_and_T() function to refine the estimated camera pose.
  //  display('Refined:')
  for (i0 = 0; i0 < 12; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_startPoint3D_w[i1 + 3 * i0] = startPoint3D_w[i0 + 12 * i1];
      b_xs[i1 + 3 * i0] = endPoint3D_w[i0 + 12 * i1];
    }
  }

  b_R_and_T(xs, xe, b_startPoint3D_w, b_xs, RPnLR_cw_data, RPnLR_cw_size, dir,
            RPnLppR_cw_data, RPnLppR_cw_size, RPnLppT_cw_data, RPnLppT_cw_size);

  // % Compute 2D points of box
  // box2d = projectPoints(box,M);
  matrixFromPoints(p3Dw1, dv0, c_M);

  //  display(c_M)
  // c_M = matrixFromPoints(p3Dw, cam2);
  camParams(c_M, *(double (*)[4])&p3Dw1[0], RPnLppR_cw_data, dir, &c_ox, &c_oy,
            &c_fx, &c_fy);

  //  display(c_R)
  //  display(c_T)
  //  display(c_ox)
  //  display(c_oy)
  //  display(c_fx)
  //  display(c_fy)
}

//
// File trailer for calc_PnL.cpp
//
// [EOF]
//
