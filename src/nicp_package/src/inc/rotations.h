#pragma once
#include <Eigen/Core>

inline Eigen::Matrix3f Rx(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    1,  0,  0,
    0,  c, -s,
    0,  s,  c;
  return R;
}


inline Eigen::Matrix3f Ry(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    c,  0,  s,
    0,  1,  0,
   -s,  0,  c;
  return R;
}

inline Eigen::Matrix3f Rz(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    c, -s, 0,
    s,  c, 0,
    0,  0, 1;
  return R;
}

inline Eigen::Matrix2f Rtheta(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix2f R;
  R <<
    c, -s, 
    s,  c;
  return R;
}

