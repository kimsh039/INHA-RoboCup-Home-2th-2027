// The broad phase must preserve the original classification at OBB surfaces,
// across rotations and with negative coordinates. It never decides masking.
#include "self_mask_bounds.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <random>

int main() {
  using V3=Eigen::Vector3d;
  // Distances are in the measurement frame and include vertical separation.
  if(!collision_mask::within_sensor_radius(V3(0.,0.,.15),.15))return 3;
  if(!collision_mask::within_sensor_radius(V3(.1,0.,0.),.15))return 4;
  if(collision_mask::within_sensor_radius(V3(.150001,0.,0.),.15))return 5;
  if(collision_mask::within_sensor_radius(V3(.1,.1,.1),.15))return 6;
  if(collision_mask::within_sensor_radius(V3(0.,0.,.2),.15))return 7;
  if(collision_mask::within_sensor_radius(V3::Zero(),0.))return 8;
  std::mt19937 random(20261015);
  std::uniform_real_distribution<double> draw(-1.,1.);
  size_t checked=0;
  for(int b=0;b<64;++b) {
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(draw(random)*3.141592653589793,
      V3(draw(random),draw(random),draw(random)).normalized()).toRotationMatrix();
    const V3 center(draw(random)*10,draw(random)*10,draw(random)*2);
    const V3 half=V3(draw(random),draw(random),draw(random)).cwiseAbs()+V3::Constant(.001);
    const double padding=.005;
    const collision_mask::Bounds bounds(rotation,center,half,padding);
    collision_mask::Bounds overall;overall.include(bounds);
    auto check=[&](const V3 &point) {
      ++checked;
      const bool original=((rotation.transpose()*(point-center)).cwiseAbs().array()<=half.array()+padding).all();
      return !original || (bounds.contains(point) && overall.contains(point));
    };
    for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)
      for(double offset : {-.00000001,0.,.00000001}) {
        const V3 local=V3(x,y,z).array()*(half.array()+padding+offset);
        if(!check(center+rotation*local))return 1;
      }
    for(int i=0;i<128;++i) {
      const V3 local=V3(draw(random),draw(random),draw(random)).array()*(half.array()+padding)*1.5;
      if(!check(center+rotation*local))return 2;
    }
  }
  std::cout<<checked<<" self-mask broad-phase comparisons passed"<<std::endl;
}
