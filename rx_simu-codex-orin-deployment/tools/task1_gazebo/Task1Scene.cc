// MIT. Render updates run in Gazebo's SceneUpdate thread; physics owns scene time.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <mutex>
#include <deque>
#include <cmath>

using json=nlohmann::json;
class Task1Scene final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  std::mutex mutex_;
  json config_, state_;
  std::deque<json> commands_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
  gz::common::ConnectionPtr render_;
  gz::common::ConnectionPtr teardown_;
  double lastPub_=-1, transit_=-1;
  uint64_t revision_=1;
  bool active_=false;
  int applied_=0;
  gz::sim::Entity moving_=gz::sim::kNullEntity;
  std::map<std::string,gz::rendering::VisualPtr> visuals_;
  gz::rendering::ScenePtr scene_;
  bool described_=false;
  void SaveSemantic(double t) {
    state_["semantic_history"].push_back({{"stamp",t},{"revision",revision_},{"buoys",state_["buoys"]}});
  }
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &s,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &events) override {
    std::ifstream in(s->Get<std::string>("scenario"));in>>config_;
    state_=config_;
    state_["semantic_history"]=json::array();SaveSemantic(0.);
    pub_=node_.Advertise<gz::msgs::StringMsg>("/task1/scene_state");
    node_.Subscribe("/task1/scene_command",&Task1Scene::Command,this);
    render_=events.Connect<gz::sim::events::SceneUpdate>([this]{Render();});
    // Release Ogre-owned objects on the render thread before its shared
    // library unloads; the system destructor runs later on another thread.
    teardown_=events.Connect<gz::sim::events::RenderTeardown>([this]{
      render_.reset();visuals_.clear();scene_.reset();
    });
  }
  void Command(const gz::msgs::StringMsg &msg) {
    try {auto j=json::parse(msg.data());std::lock_guard<std::mutex> lock(mutex_);commands_.push_back(j);}
    catch(const std::exception &e){gzerr<<"Invalid Task1 command: "<<e.what()<<std::endl;}
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(info.paused)return;
    const double t=std::chrono::duration<double>(info.simTime).count();
    std::lock_guard<std::mutex> lock(mutex_);
    while(!commands_.empty()) {
      const auto cmd=commands_.front();commands_.pop_front();
      const auto op=cmd.value("op",std::string());
      if(op=="activate")active_=true;
      if(op=="transit"&&transit_<0)transit_=t;
      if(op=="set_states") {
        bool changed=false;
        for(auto &b:state_["buoys"]) {
          auto id=b["id"].get<std::string>();
          if(cmd.contains("states")&&cmd["states"].contains(id)) {
            int v=cmd["states"][id].get<int>();
            if(v>=0&&v<=4&&v!=b["state"].get<int>()){b["state"]=v;changed=true;}
          }
        }
        if(changed){++revision_;SaveSemantic(t);}
      }
    }
    if(transit_>=0) {
      auto events=config_.value("changes",json::array());
      while(applied_<int(events.size())&&t-transit_>=events[applied_]["after_s"].get<double>()) {
        const auto &ev=events[applied_++];
        for(auto &b:state_["buoys"])if(b["id"]==ev["id"])b["state"]=ev["state"];
        ++revision_;
        SaveSemantic(t);
      }
    }
    const auto mode=config_.value("dynamic",std::string("none"));
    if(mode!="none") {
      if(moving_==gz::sim::kNullEntity)
        moving_=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("task1_moving_obstacle"));
      const double elapsed=transit_<0?0:std::max(0.,t-transit_);
      // Smooth continuous kinematic obstacle. No deletion or pose discontinuities.
      double y=-14.+std::min(28.,.15*elapsed);
      if(mode=="block") {
        if(elapsed<80)y=-14.+.175*elapsed;
        else if(elapsed<150)y=0;
        else y=std::min(14.,.175*(elapsed-150));
      }
      double x=28.; // Legacy diagnostic fixture.
      if(config_.contains("dynamic_motion")) {
        const auto &m=config_["dynamic_motion"];
        x=m["x"].get<double>();const double y0=m["y_min"].get<double>(),y1=m["y_max"].get<double>();
        const double speed=m["speed_mps"].get<double>();
        const double since=std::max(0.,elapsed-m["depart_after_s"].get<double>());
        y=std::min(y1,y0+speed*since);
        if(mode=="block") {
          const double held=m["hold_y"].get<double>(),arrival=(held-y0)/speed,wait=m["hold_s"].get<double>();
          if(since>=arrival)y=std::min(y1,held+speed*std::max(0.,since-arrival-wait));
        }
      }
      state_["dynamic_pose"]={x,y};
      if(moving_!=gz::sim::kNullEntity)
        gz::sim::Model(moving_).SetWorldPoseCmd(ecm,gz::math::Pose3d(x,y,.45,0,0,0));
    }
    state_["stamp"]=t;state_["revision"]=revision_;state_["active"]=active_;
    state_["changes_applied"]=applied_;
    for(auto &b:state_["buoys"]) {
      const int semantic=b["state"];
      b["lit"]=(semantic==4)||(semantic>0&&std::fmod(t,2.)<1.);
    }
    if(t-lastPub_>=.099) {
      gz::msgs::StringMsg msg;msg.set_data(state_.dump());pub_.Publish(msg);lastPub_=t;
    }
  }
  void Render() {
    if(!scene_)scene_=gz::rendering::sceneFromFirstRenderEngine();
    auto scene=scene_;if(!scene)return;
    if(!described_&&scene->VisualCount()>20) {
      for(unsigned int i=0;i<scene->VisualCount();++i){auto v=scene->VisualByIndex(i);
        if(v&&v->Name().find("light_")!=std::string::npos)gzmsg<<"Task1 light visual: "<<v->Name()<<std::endl;}
      described_=true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &b:state_["buoys"]) {
      for(const auto &side:{std::string("top"),std::string("side")}) {
        std::string key=b["id"].get<std::string>()+"::body::light_"+side;
        auto &visual=visuals_[key];
        if(!visual) {
          for(unsigned int i=0;i<scene->VisualCount();++i) {
            auto candidate=scene->VisualByIndex(i);
            if(candidate&&candidate->Name().find(key)!=std::string::npos){visual=candidate;break;}
          }
        }
        if(!visual)continue;
        int semantic=b["state"];bool lit=b.value("lit",false)&&side=="top";
        gz::math::Color color(.005f,.005f,.005f,1.f);
        if(lit)color=gz::math::Color(semantic==1?1.f:0.f,semantic==2?1.f:0.f,semantic>=3?1.f:0.f,1.f);
        auto mat=visual->Material();
        if(!mat){auto material=scene->CreateMaterial();visual->SetMaterial(material);scene->DestroyMaterial(material);mat=visual->Material();}
        if(mat){mat->SetAmbient(color);mat->SetDiffuse(color);mat->SetEmissive(color);}
      }
    }
  }
};
GZ_ADD_PLUGIN(Task1Scene,gz::sim::System,Task1Scene::ISystemConfigure,Task1Scene::ISystemPreUpdate)
