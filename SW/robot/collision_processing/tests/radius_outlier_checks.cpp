// Compare the spatial search with an exhaustive distance search, including
// negative coordinates and masked returns. Thin supported structures stay.
#include "radius_outlier.hpp"
#include <iostream>
#include <random>
using V3=Eigen::Vector3d;

bool check(const std::vector<V3> &points,const std::vector<uint8_t> &eligible,int minimum=1) {
  const auto actual=collision_filter::isolated_points(points,eligible,.05,minimum);
  for(size_t i=0;i<points.size();++i) {
    int neighbors=0;
    for(size_t j=0;j<points.size();++j)
      if(i!=j && eligible[j] && (points[i]-points[j]).norm()<=.05)++neighbors;
    const bool expected=eligible[i] && neighbors<minimum;
    if(bool(actual[i])!=expected){std::cerr<<"outlier mismatch at "<<i<<std::endl;return false;}
  }
  return true;
}

int main() {
  if(!check({V3::Zero()},{1}))return 1;
  if(!check({V3::Zero(),V3(.04,0,0)},{1,1}))return 2;
  if(!check({V3::Zero(),V3(.06,0,0)},{1,1}))return 3;
  if(!check({V3(-.025,0,0),V3(.025,0,0)},{1,1}))return 4;
  if(!check({V3::Zero(),V3(.049,.049,.049)},{1,1}))return 5;
  if(!check({V3::Zero(),V3(.01,0,0)},{1,0}))return 6;
  if(!check({V3::Zero(),V3(.01,0,0)},{0,1}))return 7;
  std::vector<V3> pole;
  for(int i=0;i<30;++i)pole.emplace_back(0,0,i*.02);
  if(!check(pole,std::vector<uint8_t>(pole.size(),1)))return 8;
  if(!check({V3::Zero()},{1},0))return 9;
  std::mt19937 random(20261015);std::uniform_real_distribution<double> draw(-.2,.2);
  for(int trial=0;trial<40;++trial) {
    std::vector<V3> points;std::vector<uint8_t> eligible;
    for(int i=0;i<160;++i){points.emplace_back(draw(random),draw(random),draw(random));eligible.push_back(i%7!=0);}
    if(!check(points,eligible,1) || !check(points,eligible,2))return 10;
  }
  std::cout<<"Radius outlier comparisons passed (singleton, boundaries, masks, thin pole, random clouds)"<<std::endl;
}
