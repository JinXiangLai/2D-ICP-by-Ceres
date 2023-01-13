
#include <ceres/ceres.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>

// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T &angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

class ICP2DResiaual : public ceres::SizedCostFunction<2, 3> {
 public:
  ICP2DResiaual(const Eigen::Vector2d src_p, const Eigen::Vector2d tar_p)
      : src_p_(src_p), tar_p_(tar_p) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const {
    // initialize parameters
    double const *yaw_t = parameters[0];
    const double yaw = yaw_t[0];
    const double x = yaw_t[1];
    const double y = yaw_t[2];
    Eigen::Matrix<double, 2, 2> R;
    R << ceres::cos(yaw), -ceres::sin(yaw), ceres::sin(yaw), ceres::cos(yaw);
    Eigen::Matrix<double, 2, 1> t;
    t << x, y;

    // calculate residuals
    const Eigen::Matrix<double, 2, 1> tar = R * src_p_ + t;
    residuals[0] = tar_p_(0) - tar(0);
    residuals[1] = tar_p_(1) - tar(1);

    if (!jacobians) return true;

    const double sx = src_p_(0);
    const double sy = src_p_(1);
    
    // Analytic Jacobians with Multiple Residuals, see:
    // https://groups.google.com/g/ceres-solver/c/nVZdc4hu5zw
    // calculate jacobians
    //     yaw  x  y
    // r1              ri = tar_p_(i) - tar(i)
    // r2
    jacobians[0][0] = (sin(yaw)*sx) + (cos(yaw)*sy); // dr1/dyaw
    jacobians[0][1] = -1.; // dr1/dx
    jacobians[0][2] = 0.;  // dr1/dy
    jacobians[0][3] = (-cos(yaw)*sx) + (sin(yaw)*sy); // dr2/dyaw
    jacobians[0][4] = 0.;  // dr2/dx
    jacobians[0][5] = -1.; // dr2/dy

    return true;
  }

  template <typename T>
  bool operator()(const T *yaw_t, T *residuals) const {
    Eigen::Matrix<T, 2, 2> R;
    R << ceres::cos(yaw_t[0]), -ceres::sin(yaw_t[0]), ceres::sin(yaw_t[0]),
        ceres::cos(yaw_t[0]);
    Eigen::Matrix<T, 2, 1> t;
    t << yaw_t[1], yaw_t[2];

    const Eigen::Matrix<T, 2, 1> tar = R * src_p_ + t;
    residuals[0] = tar_p_(0) - tar(0);
    residuals[1] = tar_p_(1) - tar(1);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector2d src_p,
                                     const Eigen::Vector2d tar_p) {
    return (new ceres::AutoDiffCostFunction<ICP2DResiaual, 2, 3>(
        new ICP2DResiaual(src_p, tar_p)));
  }

 private:
  const Eigen::Vector2d src_p_;
  const Eigen::Vector2d tar_p_;
};