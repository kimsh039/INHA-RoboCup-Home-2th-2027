#pragma once
#include "ray_grid.hpp"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace collision_filter {
// Radius outlier removal on original deskewed returns, before voxelization.
// Neighbors exclude the query return; masked or out-of-range returns cannot
// support it. An isolated return never supplies a free-space clearing ray.
inline std::vector<uint8_t> isolated_points(const std::vector<Eigen::Vector3d> &points,
    const std::vector<uint8_t> &eligible,double radius,int minimum_neighbors) {
  std::vector<uint8_t> isolated(points.size(),0);
  if(minimum_neighbors==0)return isolated;
  using collision_grid::Key;
  using collision_grid::Hash;
  auto key=[&](const Eigen::Vector3d &point) {
    return Key{int(std::floor(point.x()/radius)),int(std::floor(point.y()/radius)),
               int(std::floor(point.z()/radius))};
  };
  std::unordered_map<Key,std::vector<size_t>,Hash> bins;bins.reserve(points.size());
  for(size_t i=0;i<points.size();++i)if(eligible[i])bins[key(points[i])].push_back(i);
  const double squared_radius=radius*radius;
  for(size_t i=0;i<points.size();++i)if(eligible[i]) {
    const Key cell=key(points[i]);int count=0;
    auto visit=[&](const Key &neighbor) {
      const auto found=bins.find(neighbor);
      if(found==bins.end())return;
      for(const auto j : found->second) {
        if(j!=i && (points[i]-points[j]).squaredNorm()<=squared_radius && ++count>=minimum_neighbors)return;
      }
    };
    // Dense surfaces usually find support in their own bin immediately.
    visit(cell);
    for(int x=-1;x<=1 && count<minimum_neighbors;++x)
      for(int y=-1;y<=1 && count<minimum_neighbors;++y)
        for(int z=-1;z<=1 && count<minimum_neighbors;++z)
          if(x || y || z)visit({cell.x+x,cell.y+y,cell.z+z});
    isolated[i]=count<minimum_neighbors;
  }
  return isolated;
}
} // namespace collision_filter
