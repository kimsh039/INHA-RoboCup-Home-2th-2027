#pragma once
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace collision_floor {
using V3=Eigen::Vector3d;
struct Plane {
  // odom coordinates, z = a*x + b*y + c. Never changes a robot TF.
  V3 coefficients=V3::Zero();
  int64_t stamp=0;
  size_t samples=0,inliers=0;
  double residual_p95=0.;
  double height(const V3 &p) const {return p.z()-coefficients.dot(V3(p.x(),p.y(),1.));}
};
struct Config {
  double nominal=-.1425,band=.02,radius=3.,search_band=.12;
  double inlier_distance=.012,max_tilt=.0872664626,max_offset=.05,min_ratio=.55;
  size_t minimum_points=150,max_samples=2048,iterations=128;
};
inline std::optional<Plane> estimate(const std::vector<V3> &points,const std::vector<uint8_t> &eligible,
    const V3 &pose,int64_t stamp,const Config &cfg) {
  std::minstd_rand random(uint32_t(stamp)^uint32_t(uint64_t(stamp)>>32));
  std::vector<V3> sample;sample.reserve(cfg.max_samples);size_t seen=0;
  for(size_t i=0;i<points.size();++i) {
    if(!eligible[i])continue;
    const V3 local=points[i]-V3(pose.x(),pose.y(),0.);
    const double squared=local.head<2>().squaredNorm();
    if(squared<.65*.65 || squared>cfg.radius*cfg.radius || std::abs(local.z()-cfg.nominal)>cfg.search_band)continue;
    ++seen;
    if(sample.size()<cfg.max_samples)sample.push_back(local);
    else {const size_t index=std::uniform_int_distribution<size_t>(0,seen-1)(random);if(index<sample.size())sample[index]=local;}
  }
  if(sample.size()<cfg.minimum_points)return std::nullopt;
  auto plausible=[&](const V3 &c) {
    return c.allFinite() && c.head<2>().norm()<=std::tan(cfg.max_tilt) && std::abs(c.z()-cfg.nominal)<=cfg.max_offset;
  };
  size_t best=0;V3 coefficients=V3::Zero();
  for(size_t iteration=0;iteration<cfg.iterations;++iteration) {
    const V3 &a=sample[random()%sample.size()],&b=sample[random()%sample.size()],&c=sample[random()%sample.size()];
    const V3 normal=(b-a).cross(c-a);
    if(std::abs(normal.z())<1e-9)continue;
    const V3 candidate(-normal.x()/normal.z(),-normal.y()/normal.z(),normal.dot(a)/normal.z());
    if(!plausible(candidate))continue;
    size_t count=0;
    for(const auto &p : sample)if(std::abs(p.z()-candidate.dot(V3(p.x(),p.y(),1.)))<=cfg.inlier_distance)++count;
    if(count>best){best=count;coefficients=candidate;}
  }
  const size_t required=std::max(cfg.minimum_points,size_t(std::ceil(cfg.min_ratio*sample.size())));
  if(best<required)return std::nullopt;
  size_t count=0;V3 low,high;std::vector<double> residuals;
  for(int pass=0;pass<3;++pass) {
    Eigen::Matrix3d normal=Eigen::Matrix3d::Zero();V3 rhs=V3::Zero();count=0;
    low=V3::Constant(1e30);high=-low;
    for(const auto &p : sample)if(std::abs(p.z()-coefficients.dot(V3(p.x(),p.y(),1.)))<=cfg.inlier_distance) {
      const V3 row(p.x(),p.y(),1.);normal+=row*row.transpose();rhs+=row*p.z();++count;
      low=low.cwiseMin(p);high=high.cwiseMax(p);
    }
    // Require a floor patch, rather than a line of low returns.
    if(count<required || (high.x()-low.x())<.8 || (high.y()-low.y())<.8 || normal.determinant()<1e-6)return std::nullopt;
    const auto decomposition=normal.ldlt();
    if(!decomposition.isPositive())return std::nullopt;
    coefficients=decomposition.solve(rhs);
    if(!plausible(coefficients))return std::nullopt;
  }
  for(const auto &p : sample) {
    const double residual=std::abs(p.z()-coefficients.dot(V3(p.x(),p.y(),1.)));
    if(residual<=cfg.inlier_distance)residuals.push_back(residual);
  }
  if(residuals.size()<required)return std::nullopt;
  std::sort(residuals.begin(),residuals.end());
  Plane out;out.coefficients=coefficients;
  out.coefficients.z()-=coefficients.x()*pose.x()+coefficients.y()*pose.y();
  out.stamp=stamp;out.samples=sample.size();out.inliers=residuals.size();
  out.residual_p95=residuals[size_t(.95*(residuals.size()-1))];return out;
}
} // namespace collision_floor
