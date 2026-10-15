#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_set>

namespace collision_grid {
struct Key {
  int x,y,z;
  bool operator==(const Key &other) const {return x==other.x && y==other.y && z==other.z;}
};
struct Hash {
  size_t operator()(const Key &k) const {
    size_t h=std::hash<int>{}(k.x);
    h^=std::hash<int>{}(k.y)+0x9e3779b9+(h<<6)+(h>>2);
    return h^(std::hash<int>{}(k.z)+0x9e3779b9+(h<<6)+(h>>2));
  }
};
inline Key coarse_key(const Key &key) {
  return {int(std::floor(key.x/8.)),int(std::floor(key.y/8.)),int(std::floor(key.z/8.))};
}
// Visits actual beam/cell intersections, including a partial terminal cell.
template<class Visitor>
void segments(const Eigen::Vector3d &origin,const Eigen::Vector3d &direction,
              double begin,double end,double size,Visitor visit) {
  if(end<=begin)return;
  const auto inside=(origin+direction*(begin+std::min(1e-8,(end-begin)/2.))).eval();
  std::array<int,3> index{},step{};Eigen::Vector3d next,increment;
  for(int a=0;a<3;++a) {
    index[a]=int(std::floor(inside[a]/size));step[a]=direction[a]>=0?1:-1;
    if(std::abs(direction[a])<1e-12)next[a]=increment[a]=std::numeric_limits<double>::infinity();
    else {next[a]=((index[a]+(step[a]>0?1:0))*size-origin[a])/direction[a];increment[a]=size/std::abs(direction[a]);}
  }
  double entry=begin;
  for(int n=0;n<4096;++n) {
    const double crossing=next.minCoeff();
    if(!std::isfinite(crossing))return;
    visit(Key{index[0],index[1],index[2]},entry,std::min(crossing,end),crossing<=end+1e-10);
    if(crossing>=end-1e-10)return;
    for(int a=0;a<3;++a)if(next[a]<=crossing+1e-10){index[a]+=step[a];next[a]+=increment[a];}
    entry=crossing;
  }
}
// Coarse cells only skip regions with no occupied fine cells. The same 4 cm
// intersections are still used to clear occupied cells; no rays are discarded.
template<class Visitor>
void occupied_cells(const Eigen::Vector3d &origin,const Eigen::Vector3d &direction,
                    double limit,double voxel,const std::unordered_set<Key,Hash> &occupied,Visitor visit) {
  segments(origin,direction,0.,limit,voxel*8.,[&](const Key &coarse,double entry,double exit,bool) {
    if(!occupied.count(coarse))return;
    segments(origin,direction,entry,exit,voxel,[&](const Key &fine,double,double end,bool complete) {
      if(complete && end<limit-1e-10)visit(fine);
    });
  });
}
} // namespace collision_grid
