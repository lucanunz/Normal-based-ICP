#pragma once
#include "eigen_kdtree.h"
#include "Eigen/Geometry"
#include <iostream>
#include <vector>

using Vector2f = Eigen::Vector2f;
using Vector4f = Eigen::Vector4f;

// A point with a normal is a 4D vector
// the first two components represent the point
// the second two components represent the normal
using PointNormal2f = Vector4f; // dim 0,1: coordinates, dim 2,3: normal


// operator * between isometry
// transforms coordinates and rotates normal
inline PointNormal2f operator*(const Eigen::Isometry2f& T,
                               const PointNormal2f p) {
  PointNormal2f result;
  result.head<2>()=T*p.head<2>();
  result.tail<2>()=T.linear()*p.tail<2>();
  return result;
}

class NICP {
protected:
  struct PointNormalPair{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointNormalPair(const PointNormal2f& fixed_, const PointNormal2f& moving_):
      _fixed(fixed_),
      _moving(moving_){};
    
    PointNormalPair(){}
    PointNormal2f _fixed;
    PointNormal2f _moving;
  };
  using PointNormalPairVector=std::vector<PointNormalPair, Eigen::aligned_allocator<PointNormalPair>>;

public:
  using ContainerType = std::vector<PointNormal2f, Eigen::aligned_allocator<PointNormal2f> >;

  NICP(const ContainerType& fixed_,
      const ContainerType& moving_,
      int min_points_in_leaf);

  void computeCorrespondences();
  
  // for me to test
  void computeCorrespondencesFake();
  
  void optimizeCorrespondences();
  
  void run(int max_iterations);

  void draw(std::ostream& os);
  
  const Eigen::Isometry2f& X() const {return _X;}
  Eigen::Isometry2f& X()  {return _X;}
  inline int numCorrespondences() const {return _correspondences.size();}
  inline int numKernelized() const {return _num_kernelized;}
  inline int numInliers() const {return _num_inliers;}
  inline const Eigen::Matrix<float, 3,1>& dx() const {return _dx;}
 
protected:
  using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
  
  const ContainerType& _fixed_original;
  ContainerType _fixed;
  const ContainerType& _moving;
  Eigen::Isometry2f _X=Eigen::Isometry2f::Identity();
  TreeNodeType _kd_tree;
  float _ball_radius=1.f;
  float _kernel_chi2 = 1;
  float _chi2_sum=0;

  PointNormalPairVector _correspondences;
  int _num_kernelized=0;
  int _num_inliers=0;
  Eigen::Matrix<float, 3,1> _dx;
};
