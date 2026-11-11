#include "robotx_task1_nav/rules.hpp"
#include "robotx_task1_nav/dubins_estimate.hpp"
#include "robotx_task1_nav/hull_grid.hpp"
#include "robotx_task1_nav/traffic_layer.hpp"
#include "robotx_task1_nav/route_preference.hpp"
#include <mutex>
#include <set>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <dwb_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <robotx_task1_interfaces/msg/buoy_array.hpp>
#include <robotx_task1_interfaces/msg/mission_context.hpp>

namespace robotx_task1_nav {
using Observations=robotx_task1_interfaces::msg::BuoyArray;
using Context=robotx_task1_interfaces::msg::MissionContext;
double yaw(const geometry_msgs::msg::Quaternion &q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
Pose pose(const geometry_msgs::msg::Pose &p){return {p.position.x,p.position.y,yaw(p.orientation)};}
std::vector<Buoy> markers(const Observations &o){std::vector<Buoy> v;for(const auto &b:o.buoys)v.push_back({b.id,b.position.x,b.position.y,b.collision_radius,b.state});return v;}
bool valid(const Observations &o) {
  if(o.header.frame_id!="map"||o.buoys.size()!=10)return false;
  std::set<std::string> ids;int entry=0,exit=0;
  for(const auto &b:o.buoys){
    if(b.id.empty()||!ids.insert(b.id).second||b.state>4||!std::isfinite(b.position.x)||
       !std::isfinite(b.position.y)||!std::isfinite(b.collision_radius)||b.collision_radius<=0)return false;
    entry+=b.state==3;exit+=b.state==4;
  }return entry==1&&exit==1;
}
struct Input {
  std::mutex mutex;Observations observations;Context context;
  rclcpp::Subscription<Observations>::SharedPtr os;
  rclcpp::Subscription<Context>::SharedPtr cs;
  void attach(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
    os=node->create_subscription<Observations>("/task1/observations",10,[this](const Observations::SharedPtr m){
      std::lock_guard<std::mutex> l(mutex);
      if(valid(*m)&&m->revision>=observations.revision&&rclcpp::Time(m->header.stamp)>rclcpp::Time(observations.header.stamp))observations=*m;});
    cs=node->create_subscription<Context>("/task1/context",10,[this](const Context::SharedPtr m){
      std::lock_guard<std::mutex> l(mutex);
      if(m->header.frame_id=="map"&&m->revision>=context.revision&&rclcpp::Time(m->header.stamp)>rclcpp::Time(context.header.stamp))context=*m;});
  }
  bool read(const rclcpp::Time &now,Observations &o,Context &c) {
    std::lock_guard<std::mutex> l(mutex);o=observations;c=context;
    double oa=(now-rclcpp::Time(o.header.stamp)).seconds(),ca=(now-rclcpp::Time(c.header.stamp)).seconds();
    return o.buoys.size()==10&&o.survey_complete&&o.revision==c.revision&&oa>=-.1&&oa<=.5&&ca>=-.1&&ca<=.5;
  }
};

struct Visibility {
  double range=0,x=0,y=0;
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,const std::string &prefix) {
    auto parameter=[&](const std::string &key,double &value){
      if(!node->has_parameter(prefix+key))node->declare_parameter(prefix+key,value);
      node->get_parameter(prefix+key,value);
      if(!std::isfinite(value))throw std::runtime_error("invalid lidar visibility parameter");
    };
    parameter("visibility_clearance",range);parameter("lidar_x",x);parameter("lidar_y",y);
    if(range<0)throw std::runtime_error("negative lidar visibility clearance");
  }
  bool blocked(const HullGrid &grid,const Pose &p) const {
    return grid.unknownWithin(p.x+std::cos(p.h)*x-std::sin(p.h)*y,
                              p.y+std::sin(p.h)*x+std::cos(p.h)*y,range);
  }
};

class RulePlanner : public nav2_core::GlobalPlanner {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  Input input_;
  Visibility visibility_;
  double search_time_limit_=3.;
  double continuity_weight_=.2;
  std::mutex reference_mutex_;
  nav_msgs::msg::Path reference_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_sub_;
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &node,std::string name,
                 std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    node_=node.lock();map_=map;input_.attach(node_);visibility_.configure(node_,name+".");
    const auto key=name+".search_time_limit";
    if(!node_->has_parameter(key))node_->declare_parameter(key,search_time_limit_);
    node_->get_parameter(key,search_time_limit_);
    if(!std::isfinite(search_time_limit_)||search_time_limit_<=0.)
      throw std::runtime_error("invalid Task1 search time limit");
    const auto continuity=name+".continuity_weight";
    if(!node_->has_parameter(continuity))node_->declare_parameter(continuity,continuity_weight_);
    node_->get_parameter(continuity,continuity_weight_);
    if(!std::isfinite(continuity_weight_)||continuity_weight_<0.)throw std::runtime_error("invalid route continuity weight");
    reference_sub_=node_->create_subscription<nav_msgs::msg::Path>("/task1/reference_path",10,
      [this](const nav_msgs::msg::Path::SharedPtr p){
        std::lock_guard<std::mutex> lock(reference_mutex_);
        if(p->header.frame_id=="map"&&rclcpp::Time(p->header.stamp)>rclcpp::Time(reference_.header.stamp))reference_=*p;
      });
  }
  void cleanup()override{reference_sub_.reset();input_.os.reset();input_.cs.reset();map_.reset();node_.reset();}
  void activate()override{}
  void deactivate()override{}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                 const geometry_msgs::msg::PoseStamped &goal) override {
    Observations o;Context ctx;if(!input_.read(node_->now(),o,ctx))throw std::runtime_error("Task1 observation/context is stale or mismatched");
    const auto buoys=markers(o);auto cm=map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*cm->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(cm);
    auto footprint=map_->getRobotFootprint();
    HullGrid occupied;occupied.reset(*cm,footprint,buoys);
    RoutePreference preference;
    const Pose from=pose(start.pose),to=pose(goal.pose);
    {
      std::lock_guard<std::mutex> guard(reference_mutex_);
      if(ctx.enforce_passage&&continuity_weight_>0.&&
         rclcpp::Time(ctx.preferred_path_stamp)==rclcpp::Time(reference_.header.stamp)){
        std::vector<Pose> points;for(const auto &p:reference_.poses)points.push_back(pose(p.pose));
        preference.reset(*cm,points,from,to);
      }
    }
    auto cost=[&](const Pose &p){
      unsigned int mx,my;if(!cm->worldToMap(p.x,p.y,mx,my))return std::numeric_limits<double>::infinity();
      if(cm->getCost(mx,my)>=253)return std::numeric_limits<double>::infinity();
      double value=checker.footprintCostAtPose(p.x,p.y,p.h,footprint);
      if(value>=254||value<0)return std::numeric_limits<double>::infinity();
      if(occupied.collides(p))return std::numeric_limits<double>::infinity();
      if(visibility_.blocked(occupied,p))return std::numeric_limits<double>::infinity();
      // Edge-only grid collision checks miss a small marker entirely inside the hull.
      for(const auto &b:buoys){auto xy=body(p,b);bool inside=false;double distance=1e9;
        for(size_t i=0,j=footprint.size()-1;i<footprint.size();j=i++){
          auto a=footprint[j],c=footprint[i];double dx=c.x-a.x,dy=c.y-a.y;
          double t=std::clamp(((xy[0]-a.x)*dx+(xy[1]-a.y)*dy)/(dx*dx+dy*dy),0.,1.);
          distance=std::min(distance,std::hypot(xy[0]-a.x-t*dx,xy[1]-a.y-t*dy));
          if(((a.y>xy[1])!=(c.y>xy[1]))&&xy[0]<(c.x-a.x)*(xy[1]-a.y)/(c.y-a.y)+a.x)inside=!inside;
        }if(inside||distance<b.r)return std::numeric_limits<double>::infinity();
      }
      return 1.5*cm->getCost(mx,my)/252.+continuity_weight_*preference.distance(p);
    };
    std::vector<Pose> path;
    if(ctx.circle_direction!=0) {
      double cx=ctx.circle_center.x,cy=ctx.circle_center.y;
      double a0=std::atan2(from.y-cy,from.x-cx),a1=std::atan2(to.y-cy,to.x-cx);
      double sweep=wrap(a1-a0);if(sweep*ctx.circle_direction<0)sweep+=ctx.circle_direction*2*pi;
      if(std::abs(sweep)>pi)throw std::runtime_error("circle segment exceeds half turn");
      int n=std::max(2,int(std::ceil(std::abs(sweep)*ctx.circle_radius/.12)));
      path.push_back(from);
      for(int i=1;i<=n;++i){double a=a0+sweep*i/n;
        Pose q{cx+ctx.circle_radius*std::cos(a),cy+ctx.circle_radius*std::sin(a),wrap(a+ctx.circle_direction*pi/2)};
        if(!std::isfinite(cost(q)))throw std::runtime_error("circle arc footprint blocked");
        path.push_back(q);
      }
    } else {
      DubinsEstimate dubins(to);
      std::function<double(const Pose&)> estimate;
      if(ctx.enforce_passage)estimate=[&](const Pose &p){return dubins(p);};
      auto result=search(from,to,buoys,ctx.enforce_passage,ctx.passed_colors,cost,search_time_limit_,estimate);
      RCLCPP_INFO(node_->get_logger(),"Task1 search expanded %zu states",result.expanded);path=result.path;
    }
    nav_msgs::msg::Path result;result.header.frame_id="map";result.header.stamp=node_->now();
    for(size_t i=0;i<path.size();++i){
      int n=i==0?1:std::max(1,int(std::ceil(std::max(std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y)/.1,std::abs(wrap(path[i].h-path[i-1].h))/.1))));
      for(int j=1;j<=n;++j){const auto p=i==0?path[i]:interpolate(path[i-1],path[i],double(j)/n);
        geometry_msgs::msg::PoseStamped m;m.header=result.header;m.pose.position.x=p.x;m.pose.position.y=p.y;
        m.pose.orientation.z=std::sin(p.h/2);m.pose.orientation.w=std::cos(p.h/2);result.poses.push_back(m);
      }
    }return result;
  }
};

class FullHullCritic : public dwb_core::TrajectoryCritic {
  HullGrid occupied_;
  Input input_;Visibility visibility_;bool ready_=false;
 public:
  void onInit() override {input_.attach(node_.lock());visibility_.configure(node_.lock(),dwb_plugin_name_+"."+name_+".");}
  bool prepare(const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Path2D &) override {
    Observations o;Context c;ready_=input_.read(node_.lock()->now(),o,c);
    occupied_.reset(*costmap_ros_->getCostmap(),costmap_ros_->getRobotFootprint(),markers(o));return true;
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) override {
    if(!ready_)throw dwb_core::IllegalTrajectoryException(name_,"stale Task1 context");
    for(const auto &p:trajectory.poses){
      if(occupied_.collides({p.x,p.y,p.theta}))throw dwb_core::IllegalTrajectoryException(name_,"occupied cell inside full hull");
      if(visibility_.blocked(occupied_,{p.x,p.y,p.theta}))throw dwb_core::IllegalTrajectoryException(name_,"unknown body inside lidar clearing stand-off");
    }
    return 0.;
  }
};

class RuleCritic : public dwb_core::TrajectoryCritic {
  Input input_;std::vector<Buoy> buoys_;bool ready_=false,enforce_=false;
 public:
  void onInit() override {input_.attach(node_.lock());}
  bool prepare(const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Path2D &) override {
    Observations o;Context c;ready_=input_.read(node_.lock()->now(),o,c);
    buoys_=markers(o);enforce_=c.enforce_passage;return true;
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) override {
    if(!ready_)throw dwb_core::IllegalTrajectoryException(name_,"stale Task1 context");
    if(enforce_)for(size_t i=1;i<trajectory.poses.size();++i){const auto &a=trajectory.poses[i-1],&b=trajectory.poses[i];
      if(passage({a.x,a.y,a.theta},{b.x,b.y,b.theta},buoys_)<0)
        throw dwb_core::IllegalTrajectoryException(name_,"wrong-side nearest abeam buoy");
    }
    return 0.;
  }
};


class KnownBuoys : public nav2_costmap_2d::Layer {
  std::mutex mutex_;std::vector<Buoy> buoys_;
  uint64_t revision_=0;int64_t stamp_=0;
  rclcpp::Subscription<Observations>::SharedPtr sub_;
 public:
  void onInitialize() override {
    enabled_=true;current_=true;auto node=node_.lock();
    sub_=node->create_subscription<Observations>("/task1/observations",10,[this](const Observations::SharedPtr m){
      if(!valid(*m))return;
      std::lock_guard<std::mutex> lock(mutex_);auto t=rclcpp::Time(m->header.stamp).nanoseconds();
      if(t<=stamp_||m->revision<revision_)return;
      buoys_=markers(*m);stamp_=t;revision_=m->revision;});
  }
  void reset() override {current_=true;}
  bool isClearable() override{return false;}
  void updateBounds(double,double,double,double *minx,double *miny,double *maxx,double *maxy) override {
    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &b:buoys_){*minx=std::min(*minx,b.x-b.r);*maxx=std::max(*maxx,b.x+b.r);
      *miny=std::min(*miny,b.y-b.r);*maxy=std::max(*maxy,b.y+b.r);}
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &map,int min_i,int min_j,int max_i,int max_j) override {
    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &b:buoys_){int x0,y0,x1,y1;map.worldToMapEnforceBounds(b.x-b.r,b.y-b.r,x0,y0);map.worldToMapEnforceBounds(b.x+b.r,b.y+b.r,x1,y1);
      for(int y=std::max(y0,min_j);y<std::min(y1+1,max_j);++y)for(int x=std::max(x0,min_i);x<std::min(x1+1,max_i);++x){
        double wx,wy;map.mapToWorld(x,y,wx,wy);if(std::hypot(wx-b.x,wy-b.y)<=b.r+map.getResolution()*.71)map.setCost(x,y,254);
      }
    }current_=true;
  }
};
} // namespace
PLUGINLIB_EXPORT_CLASS(robotx_task1_nav::RulePlanner,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(robotx_task1_nav::RuleCritic,dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(robotx_task1_nav::FullHullCritic,dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(robotx_task1_nav::KnownBuoys,nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(robotx_task1_nav::TrafficLayer,nav2_costmap_2d::Layer)
