#pragma once
#include "robot_geometry.hpp"
#include <nanoflann.hpp>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>

namespace collision_guard {
using collision_model::Box;
using collision_model::V3;
using V2=Eigen::Vector2d;
using Clock=std::chrono::steady_clock;
struct Expired : std::runtime_error {Expired():std::runtime_error("computation_timeout") {}};
inline void check(Clock::time_point deadline) {if(Clock::now()>deadline)throw Expired();}
struct Scene {
  std::vector<V3> points,sources;
  std::vector<double> observation_ages,floor_heights;
  using Metric=nanoflann::L2_Simple_Adaptor<double,Scene>;
  using Tree=nanoflann::KDTreeSingleIndexAdaptor<Metric,Scene,3,size_t>;
  Tree tree;
  Scene(std::vector<V3> p,std::vector<V3> s,std::vector<double> ages={},std::vector<double> heights={}):
      points(std::move(p)),sources(std::move(s)),observation_ages(std::move(ages)),floor_heights(std::move(heights)),
      // nanoflann 1.4 builds the index in its constructor.
      tree(3,*this,nanoflann::KDTreeSingleIndexAdaptorParams(32)) {}
  Scene(const Scene &)=delete;
  size_t kdtree_get_point_count() const {return points.size();}
  double kdtree_get_pt(size_t index,size_t axis) const {return points[index][axis];}
  template<class T> bool kdtree_get_bbox(T &) const {return false;}
};
struct Shape {
  std::vector<Box> boxes;
  double extent=0.;
  explicit Shape(std::vector<Box> value):boxes(std::move(value)) {
    for(const auto &box : boxes)for(int x : {-1,1})for(int y : {-1,1})for(int z : {-1,1}) {
      const V3 vertex=box.pose*(box.half.cwiseProduct(V3(x,y,z)));
      extent=std::max(extent,vertex.head<2>().norm());
    }
  }
};
struct Distance {
  double value=std::numeric_limits<double>::infinity();
  size_t point=0,box=0;
  bool found=false;
};
inline double signed_distance(const V3 &local,const V3 &half) {
  const V3 delta=local.cwiseAbs()-half;
  return delta.cwiseMax(0.).norm()+std::min(0.,delta.maxCoeff());
}
inline Distance nearest(const Scene &scene,const Shape &shape,const V3 &pose,double radius,Clock::time_point deadline,
    const V3 *from_pose=nullptr) {
  Distance result;check(deadline);
  if(scene.points.empty())return result;
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(pose.z(),V3::UnitZ()).toRotationMatrix();
  const V3 translation(pose.x(),pose.y(),0.);
  const Eigen::Matrix3d from_rotation=from_pose?Eigen::AngleAxisd(from_pose->z(),V3::UnitZ()).toRotationMatrix():rotation;
  const V3 from_translation=from_pose?V3(from_pose->x(),from_pose->y(),0.):translation;
  std::vector<std::pair<size_t,double>> candidates;candidates.reserve(512);
  for(size_t b=0;b<shape.boxes.size();++b) {
    check(deadline);const auto &box=shape.boxes[b];
    const V3 center=rotation*box.pose.translation()+translation;
    const Eigen::Matrix3d inverse=(rotation*box.pose.linear()).transpose();
    const V3 from_center=from_rotation*box.pose.translation()+from_translation;
    const Eigen::Matrix3d from_inverse=(from_rotation*box.pose.linear()).transpose();
    const double limit=box.half.norm()+radius;
    nanoflann::SearchParams params;params.sorted=false;params.eps=0.;
    // nanoflann's L2 API expects squared radii and uses a strict comparison.
    scene.tree.radiusSearch(center.data(),std::nextafter(limit*limit,std::numeric_limits<double>::infinity()),candidates,params);
    check(deadline);
    size_t count=0;
    for(const auto &[index,unused] : candidates) {
      static_cast<void>(unused);if((count++ & 255)==0)check(deadline);
      if(from_pose) {
        // Evaluate each moving part and source return, rather than comparing
        // global minima: an unrelated rear point cannot hide a front hazard.
        // Signed distances also detect increasing penetration inside a box.
        const V3 &source=scene.sources.empty()?scene.points[index]:scene.sources[index];
        const double before=signed_distance(from_inverse*(source-from_center),box.half);
        const double after=signed_distance(inverse*(source-center),box.half);
        if(after>=before-1e-9)continue;
      }
      const double value=((inverse*(scene.points[index]-center)).cwiseAbs()-box.half).cwiseMax(0.).norm();
      if(value<result.value)result={value,index,b,true};
      if(value==0.){check(deadline);return result;}
    }
  }
  check(deadline);return result;
}
struct Config {
  bool body_only=true;
  double clearance=0.,uncertainty=0.,voxel=.04,reaction=.3,linear_deceleration=.5,angular_deceleration=1.;
  double step=.04,minimum_scale=.1;
  double boundary() const {return clearance+uncertainty+(body_only?0.:std::sqrt(3.)*voxel/2.);}
};
// Segment/OBB contact in each moving part's local coordinates. This checks
// between prediction poses without adding a distance shell around the body.
inline bool contact(const V3 &from,const V3 &to,const V3 &half) {
  const V3 delta=to-from;double enter=0.,leave=1.;
  for(int axis=0;axis<3;++axis) {
    if(std::abs(delta[axis])<1e-12) {if(std::abs(from[axis])>half[axis])return false;continue;}
    double a=(-half[axis]-from[axis])/delta[axis],b=(half[axis]-from[axis])/delta[axis];
    if(a>b)std::swap(a,b);
    enter=std::max(enter,a);leave=std::min(leave,b);if(enter>leave)return false;
  }
  const V3 faces=from.cwiseAbs()-half;const double before=faces.maxCoeff();
  if(before<=0.) {
    // The signed distance inside a box is the maximum of its six affine
    // face distances. A nonnegative initial slope permits departing an
    // existing contact, but never moving further into it.
    double slope=-std::numeric_limits<double>::infinity();
    for(int axis=0;axis<3;++axis)if(faces[axis]>=before-1e-12)
      slope=std::max(slope,from[axis]==0.?std::abs(delta[axis]):std::copysign(1.,from[axis])*delta[axis]);
    if(slope>=-1e-12)return false;
  }
  return true;
}
inline Distance swept_contact(const Scene &scene,const Shape &shape,const V3 &from,const V3 &to,
    double movement,Clock::time_point deadline) {
  check(deadline);if(scene.points.empty())return {};
  const Eigen::Matrix3d r0=Eigen::AngleAxisd(from.z(),V3::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d r1=Eigen::AngleAxisd(to.z(),V3::UnitZ()).toRotationMatrix();
  const V3 t0(from.x(),from.y(),0.),t1(to.x(),to.y(),0.);
  std::vector<std::pair<size_t,double>> candidates;candidates.reserve(256);
  for(size_t b=0;b<shape.boxes.size();++b) {
    check(deadline);const auto &box=shape.boxes[b];
    const V3 c0=r0*box.pose.translation()+t0,c1=r1*box.pose.translation()+t1;
    const Eigen::Matrix3d i0=(r0*box.pose.linear()).transpose(),i1=(r1*box.pose.linear()).transpose();
    // This extra radius is broad phase only, never a collision margin.
    const double radius=box.half.norm()+movement;
    nanoflann::SearchParams params;params.sorted=false;params.eps=0.;
    scene.tree.radiusSearch(c1.data(),std::nextafter(radius*radius,std::numeric_limits<double>::infinity()),candidates,params);
    size_t count=0;
    for(const auto &[index,unused]:candidates) {
      static_cast<void>(unused);if((count++ & 255)==0)check(deadline);
      const V3 &point=scene.points[index];
      if(contact(i0*(point-c0),i1*(point-c1),box.half))return {0.,index,b,true};
    }
  }
  check(deadline);return {};
}
struct Phase {V2 velocity;double duration;bool braking;};
inline std::optional<V3> predict(const Scene &scene,const Shape &shape,V3 pose,
    const std::vector<Phase> &phases,const Config &cfg,double boundary,Clock::time_point deadline,
    Distance *obstruction=nullptr) {
  for(const auto &phase : phases) {
    V2 velocity=phase.velocity;
    if(velocity.cwiseAbs().maxCoeff()<=1e-6)continue;
    double elapsed=0.;
    while(elapsed<phase.duration-1e-9) {
      check(deadline);const double dt=std::min(cfg.step,phase.duration-elapsed);V2 following=velocity;
      if(phase.braking)for(int axis=0;axis<2;++axis) {
        const double deceleration=axis?cfg.angular_deceleration:cfg.linear_deceleration;
        following[axis]=std::copysign(std::max(0.,std::abs(velocity[axis])-deceleration*dt),velocity[axis]);
      }
      const V2 average=(velocity+following)/2.;const double theta=pose.z()+average.y()*dt/2.;
      const V3 previous=pose;
      pose+=V3(average.x()*std::cos(theta)*dt,average.x()*std::sin(theta)*dt,average.y()*dt);
      const double sampling=(std::abs(average.x())+std::abs(average.y())*shape.extent)*dt;
      if(cfg.body_only && boundary==0.) {
        const auto collision=swept_contact(scene,shape,previous,pose,sampling,deadline);
        if(collision.found) {if(obstruction)*obstruction=collision;return std::nullopt;}
        velocity=following;elapsed+=dt;continue;
      }
      const auto close=nearest(scene,shape,pose,boundary+sampling,deadline,&previous);
      if(close.value<boundary+sampling) {
        if(obstruction)*obstruction=close;
        return std::nullopt;
      }
      velocity=following;elapsed+=dt;
    }
  }
  check(deadline);return pose;
}
struct Result {
  std::string state="blocked",reason;
  V2 output=V2::Zero();double scale=0.;Distance current,obstruction;
};
inline Result evaluate(const Scene &scene,const Shape &shape,const V3 &pose,const V2 &measured,
    const V2 &command,bool command_fresh,const std::string &command_error,double source_age,
    const Config &cfg,Clock::time_point deadline) {
  Result out;const double boundary=cfg.boundary();
  out.current=nearest(scene,shape,pose,boundary+.1,deadline);
  if(!command_error.empty()){out.reason=command_error;return out;}
  if(!command_fresh){out.state="idle";out.reason="command_timeout";return out;}
  if(command.cwiseAbs().maxCoeff()<=1e-5){out.state="idle";out.reason="zero_command";return out;}
  const double latency=cfg.reaction+std::max(0.,source_age);
  const auto reaction=predict(scene,shape,pose,{{measured,latency,false}},cfg,boundary,deadline,&out.obstruction);
  if(reaction)for(const double scale : {1.,.75,.5,.25,cfg.minimum_scale}) {
    const V2 trial=command*scale;
    const double stopping=std::max(std::abs(trial.x())/cfg.linear_deceleration,std::abs(trial.y())/cfg.angular_deceleration);
    if(predict(scene,shape,*reaction,{{trial,latency,false},{trial,stopping,true}},cfg,boundary,deadline,&out.obstruction)) {
      out.scale=scale;out.output=trial;out.state=scale==1.?"clear":"slowed";
      out.reason=scale==1.?"clear_path":"braking_clearance";return out;
    }
  }
  out.reason="predicted_clearance_violation";return out;
}
} // namespace collision_guard
