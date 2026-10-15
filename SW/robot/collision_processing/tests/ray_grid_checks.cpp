// Independent slab intersections check that coarse skipping clears exactly
// the same occupied cells as a ray traversing the fine grid directly.
#include "ray_grid.hpp"
#include <iostream>
#include <random>
using namespace collision_grid;
using V3=Eigen::Vector3d;
int main() {
  if(!(coarse_key({-1,-8,-9})==Key{-1,-1,-2}))return 1;
  std::unordered_set<Key,Hash> occupied,coarse;
  for(int x=-8;x<=8;++x)for(int y=-8;y<=8;++y)for(int z=-8;z<=8;++z)
    if((x+2*y+3*z)%7==0){occupied.insert({x,y,z});coarse.insert(coarse_key({x,y,z}));}
  auto check=[&](const V3 &origin,const V3 &direction,double limit) {
    std::unordered_set<Key,Hash> actual,expected;
    occupied_cells(origin,direction,limit,.04,coarse,[&](const Key &key){if(occupied.count(key))actual.insert(key);});
    for(const auto &key : occupied) {
      const V3 low=V3(key.x,key.y,key.z)*.04,high=low+V3::Constant(.04);
      double enter=0.,exit=1e30;bool intersects=true;
      for(int axis=0;axis<3;++axis) {
        if(std::abs(direction[axis])<1e-12) {
          if(origin[axis]<low[axis] || origin[axis]>=high[axis])intersects=false;
        } else {
          double a=(low[axis]-origin[axis])/direction[axis],b=(high[axis]-origin[axis])/direction[axis];
          if(a>b)std::swap(a,b);
          enter=std::max(enter,a);exit=std::min(exit,b);
        }
      }
      if(intersects && exit>enter+1e-10 && exit<limit-1e-10)expected.insert(key);
    }
    if(actual!=expected){std::cerr<<"ray intersection mismatch: "<<actual.size()<<" / "<<expected.size()<<std::endl;return false;}
    return true;
  };
  if(!check(V3(-.19,.011,.013),V3(1,0,0),.51))return 2;
  if(!check(V3(.19,.011,.013),V3(-1,0,0),.51))return 3;
  std::mt19937 random(20261015);std::uniform_real_distribution<double> draw(-.29,.29);
  for(int i=0;i<250;++i) {
    const V3 origin(draw(random),draw(random),draw(random));
    const V3 direction=V3(draw(random),draw(random),draw(random)).normalized();
    if(!check(origin,direction,.8))return 4;
  }
  std::cout<<"253 ray-grid safety checks passed"<<std::endl;
  return 0;
}
