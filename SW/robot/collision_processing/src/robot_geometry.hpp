#pragma once
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace collision_model {
using V3=Eigen::Vector3d;
using M3=Eigen::Matrix3d;
using Transform=Eigen::Isometry3d;
inline V3 vector(const YAML::Node &v) {
  if (!v.IsSequence() || v.size()!=3) throw std::runtime_error("invalid_geometry_vector");
  V3 out(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
  if (!out.allFinite()) throw std::runtime_error("invalid_geometry_vector");
  return out;
}
inline Transform transform(const YAML::Node &v) {
  Transform out=Transform::Identity();
  const auto r=v["rotation"];
  if (!r.IsSequence() || r.size()!=9) throw std::runtime_error("invalid_geometry_rotation");
  for (int i=0;i<3;++i) for (int j=0;j<3;++j) out.linear()(i,j)=r[3*i+j].as<double>();
  out.translation()=vector(v["translation"]);
  if (!out.matrix().allFinite() || !(out.linear().transpose()*out.linear()).isApprox(M3::Identity(),1e-6))
    throw std::runtime_error("invalid_geometry_rotation");
  return out;
}
struct Box {
  std::string name; Transform pose; V3 half;
  bool contains(const V3 &p, double padding) const {
    return ((pose.linear().transpose()*(p-pose.translation())).cwiseAbs().array()
            <= (half.array()+padding)).all();
  }
};
inline std::vector<Box> boxes(const YAML::Node &v) {
  std::vector<Box> out;
  for (const auto &b : v) {
    Box box{b["name"].as<std::string>(),transform(b),vector(b["half"])};
    if ((box.half.array()<0).any()) throw std::runtime_error("invalid_geometry_size");
    out.push_back(std::move(box));
  }
  return out;
}
struct Joint {
  std::string parent, name, kind;
  Transform origin; V3 axis; double lower, upper;
};
class Geometry {
 public:
  std::vector<Box> fixed;
  std::vector<Box> tracer;
  std::vector<Box> self_interiors;
  std::map<std::string,Joint> joints;
  std::map<std::string,std::vector<Box>> links;
  std::vector<std::string> required;
  std::vector<double> joint_reach;
  explicit Geometry(const std::string &path) {
    const auto root=YAML::LoadFile(path);
    fixed=boxes(root["fixed"]);
    if(root["tracer"])tracer=boxes(root["tracer"]);
    if(root["self_interiors"])self_interiors=boxes(root["self_interiors"]);
    required=root["required_joints"].as<std::vector<std::string>>();
    if (fixed.empty() || required.empty()) throw std::runtime_error("empty_robot_geometry");
    for (const auto &j : root["joints"]) {
      Joint item{j["parent"].as<std::string>(),j["name"].as<std::string>(),j["kind"].as<std::string>(),
                 transform(j),vector(j["axis"]),j["lower"].as<double>(),j["upper"].as<double>()};
      if (item.kind!="fixed" && item.axis.norm()<1e-9) throw std::runtime_error("invalid_joint_axis");
      if (item.axis.norm()>1e-9) item.axis.normalize();
      joints.emplace(j["child"].as<std::string>(),std::move(item));
    }
    for (const auto &link : root["arm_links"]) links.emplace(link["name"].as<std::string>(),boxes(link["boxes"]));
    joint_reach.resize(required.size(),0.);
    for(const auto &[link,items] : links) for(const auto &box : items) {
      double bound=box.pose.translation().norm()+box.half.norm();std::string child=link;
      for(int depth=0;depth<64 && joints.count(child);++depth) {
        const auto &j=joints.at(child);
        const auto found=std::find(required.begin(),required.end(),j.name);
        if(found!=required.end()) {
          const auto index=std::distance(required.begin(),found);
          joint_reach[index]=std::max(joint_reach[index],bound);
        }
        bound+=j.origin.translation().norm();
        if(j.kind=="prismatic")bound+=std::max(std::abs(j.lower),std::abs(j.upper));
        child=j.parent;
      }
    }
  }
  std::vector<Box> at(const std::vector<double> &positions, const std::string &base) const {
    std::map<std::string,Transform> cache{{base,Transform::Identity()}};
    std::map<std::string,double> q;
    for (size_t i=0;i<required.size();++i) q[required[i]]=positions.at(i);
    auto visit=[&](auto &&self,const std::string &link,int depth)->Transform {
      if (depth>64) throw std::runtime_error("invalid_joint_tree");
      if (cache.count(link)) return cache.at(link);
      const auto &j=joints.at(link);
      Transform motion=Transform::Identity();
      if (j.kind=="revolute") motion.linear()=Eigen::AngleAxisd(q.at(j.name),j.axis).toRotationMatrix();
      else if (j.kind=="prismatic") motion.translation()=j.axis*((j.lower+j.upper)/2.);
      else if (j.kind!="fixed") throw std::runtime_error("unsupported_joint_type");
      auto pose=self(self,j.parent,depth+1)*j.origin*motion;
      cache.emplace(link,pose); return pose;
    };
    auto out=fixed;
    for (const auto &[link,items] : links) {
      const auto pose=visit(visit,link,0);
      const auto &j=joints.at(link);
      for (const auto &item : items) {
        auto box=item; box.pose=pose*item.pose;
        if (j.kind=="prismatic") box.half+=(item.pose.linear().transpose()*j.axis).cwiseAbs()*((j.upper-j.lower)/2.);
        out.push_back(std::move(box));
      }
    }
    return out;
  }
};
} // namespace collision_model
