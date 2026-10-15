// Pure computation probe: no ROS initialization or publishers.
#include "guard_core.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using Json=nlohmann::json;
using namespace collision_guard;
V3 vector(const Json &j) {return V3(j[0].get<double>(),j[1].get<double>(),j[2].get<double>());}
int main(int argc,char **argv) {
  if(argc!=2)return 1;
  Json fixture;std::ifstream(argv[1])>>fixture;
  collision_model::Geometry model(fixture["geometry_file"].get<std::string>());
  Shape shape(model.at(fixture["joints"].get<std::vector<double>>(),"base_link"));
  Json results=Json::array();Config cfg;
  for(const auto &item : fixture["scenes"]) {
    std::vector<V3> points;for(const auto &p : item["points"])points.push_back(vector(p));
    Scene scene(points,points);
    for(const auto &query : item["queries"]) {
      const auto start=Clock::now();
      const auto out=evaluate(scene,shape,vector(query["pose"]),
        V2(query["measured"][0].get<double>(),query["measured"][1].get<double>()),
        V2(query["command"][0].get<double>(),query["command"][1].get<double>()),
        query.value("fresh",true),query.value("error",std::string()),query.value("source_age",.1),cfg,start+std::chrono::seconds(5));
      results.push_back({{"state",out.state},{"reason",out.reason},{"scale",out.scale},
        {"distance",std::isfinite(out.current.value)?Json(out.current.value):Json(nullptr)},
        {"processing_ms",std::chrono::duration<double,std::milli>(Clock::now()-start).count()}});
    }
  }
  // A missed deadline must reject both empty scenes and non-empty geometry.
  bool expired=false;
  try {Scene empty({},{});nearest(empty,shape,V3::Zero(),.2,Clock::now()-std::chrono::seconds(1));}
  catch(const Expired &){expired=true;}
  if(!expired)return 2;
  std::cout<<results.dump()<<std::endl;
}
