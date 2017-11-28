#ifndef SOPHUS_TEST_LOCAL_PARAMETERIZATION_SE3_HPP
#define SOPHUS_TEST_LOCAL_PARAMETERIZATION_SE3_HPP

#include <ceres/local_parameterization.h>
#include <ceres/ceres.h>
#include <ceres/types.h>
#include <ceres/autodiff_cost_function.h>
#include <sophus/se3.hpp>

namespace Sophus {
namespace test {

////from https://github.com/adrelino/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
struct SophusSE3Plus{
    template<typename T>
    bool operator()(const T* x_raw, const T* delta_raw, T* x_plus_delta_raw) const {
        const Eigen::Map< const Sophus::SE3Group<T> > x(x_raw);
        const Eigen::Map< const Eigen::Matrix<T,6,1> > delta(delta_raw);
        Eigen::Map< Sophus::SE3Group<T> > x_plus_delta(x_plus_delta_raw);
        x_plus_delta = x * Sophus::SE3Group<T>::exp(delta);
        return true;
      }
};

class LocalParameterizationSE3 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSE3() {}

  // SE3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<SE3d const> const T(T_raw);
    Eigen::Map<Vector6d const> const delta(delta_raw);
    Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * SE3d::exp(delta);
    return true;
  }

  // Jacobian of SE3 plus operation for Ceres
  //
  // dx T * exp(x)  with  x=0
  //
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const {
    Eigen::Map<SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_raw);
    jacobian = T.internalJacobian().transpose();
    return true;
  }

  virtual int GlobalSize() const { return SE3d::num_parameters; }

  virtual int LocalSize() const { return SE3d::DoF; }
};

//https://groups.google.com/forum/#!topic/ceres-solver/a9JhUIWOn1I
static ceres::LocalParameterization* getParameterization(bool automaticDiff){
    if(automaticDiff){
        cout<<"automatic diff sophusSE3 local parameterization"<<endl;
        return new ceres::AutoDiffLocalParameterization<SophusSE3Plus,Sophus::SE3d::num_parameters, Sophus::SE3d::DoF>;
    }
    else{
        cout<<"analytic diff sophusSE3 local parameterization"<<endl;
        return new LocalParameterizationSE3();
    }
}

}  // namespace test
}  // namespace Sophus

#endif
