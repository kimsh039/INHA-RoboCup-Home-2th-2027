#pragma once
#include <Eigen/Core>
#include <limits>

namespace collision_mask {
inline bool within_sensor_radius(const Eigen::Vector3d &sensor_point,double radius) {
  return radius>0. && sensor_point.squaredNorm()<=radius*radius;
}
// Broad phase only: passing this bound still requires the original OBB test.
// A small outward rounding allowance cannot enlarge the actual self mask.
struct Bounds {
  Eigen::Vector3d low=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d high=Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  Bounds()=default;
  Bounds(const Eigen::Matrix3d &rotation,const Eigen::Vector3d &center,
         const Eigen::Vector3d &half,double padding) {
    const Eigen::Vector3d extent=rotation.cwiseAbs()*(half.array()+padding).matrix();
    const double rounding=1e-9*(1.+center.cwiseAbs().maxCoeff()+extent.maxCoeff());
    low=(center-extent).array()-rounding;
    high=(center+extent).array()+rounding;
  }
  void include(const Bounds &other) {low=low.cwiseMin(other.low);high=high.cwiseMax(other.high);}
  bool contains(const Eigen::Vector3d &point) const {
    return (point.array()>=low.array()).all() && (point.array()<=high.array()).all();
  }
};
} // namespace collision_mask
