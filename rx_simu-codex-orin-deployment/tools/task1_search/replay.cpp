// Offline replay of recorded costmaps. OccupancyGrid loses some cost precision;
// this diagnoses search, and never substitutes for a live planning test.
#include "robotx_task1_nav/rules.hpp"
#include "robotx_task1_nav/dubins_estimate.hpp"
#include "robotx_task1_nav/hull_grid.hpp"
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <fstream>
#include <iostream>
using namespace robotx_task1_nav;
int main(int argc,char **argv) {
  const bool reference_mode=argc>=3&&std::string(argv[2])=="reference";
  const int optional=reference_mode?4:3;
  if(argc!=optional&&argc!=optional+3)throw std::runtime_error("replay INPUT euclidean|dubins [visibility_range lidar_x lidar_y], or INPUT reference PATH [visibility_range lidar_x lidar_y]");
  const double visibility=argc==optional+3?std::stod(argv[optional]):0.,
               lx=argc==optional+3?std::stod(argv[optional+1]):0.,ly=argc==optional+3?std::stod(argv[optional+2]):0.;
  std::ifstream in(argv[1]);unsigned w,h,n;double res,ox,oy;
  in>>w>>h>>res>>ox>>oy;
  Pose start,goal;int bits;
  in>>start.x>>start.y>>start.h>>goal.x>>goal.y>>goal.h>>bits;
  in>>n;std::vector<Buoy> buoys(n);for(auto &b:buoys)in>>b.id>>b.x>>b.y>>b.r>>b.state;
  in>>n;std::vector<geometry_msgs::msg::Point> footprint(n);for(auto &p:footprint)in>>p.x>>p.y;
  nav2_costmap_2d::Costmap2D map(w,h,res,ox,oy);
  for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){int v;in>>v;map.setCost(x,y,v);}
  if(!in)throw std::runtime_error("Incomplete input");
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&map);
  HullGrid occupied;occupied.reset(map,footprint,buoys);
  auto cost=[&](const Pose &p){
    unsigned mx,my;if(!map.worldToMap(p.x,p.y,mx,my)||map.getCost(mx,my)>=253)return INFINITY;
    double v=checker.footprintCostAtPose(p.x,p.y,p.h,footprint);
    if(v>=254||v<0)return INFINITY;
    if(occupied.collides(p))return INFINITY;
    if(occupied.unknownWithin(p.x+std::cos(p.h)*lx-std::sin(p.h)*ly,
                             p.y+std::sin(p.h)*lx+std::cos(p.h)*ly,visibility))return INFINITY;
    for(const auto &b:buoys){auto xy=body(p,b);bool inside=false;double d=1e9;
      for(size_t i=0,j=footprint.size()-1;i<footprint.size();j=i++){
        auto a=footprint[j],c=footprint[i];double dx=c.x-a.x,dy=c.y-a.y;
        double t=std::clamp(((xy[0]-a.x)*dx+(xy[1]-a.y)*dy)/(dx*dx+dy*dy),0.,1.);
        d=std::min(d,std::hypot(xy[0]-a.x-t*dx,xy[1]-a.y-t*dy));
        if(((a.y>xy[1])!=(c.y>xy[1]))&&xy[0]<(c.x-a.x)*(xy[1]-a.y)/(c.y-a.y)+a.x)inside=!inside;
      }if(inside||d<b.r)return INFINITY;
    }return float(1.5*map.getCost(mx,my)/252.);
  };
  if(reference_mode) {
    std::ifstream source(argv[3]);std::vector<Pose> path;Pose point;
    while(source>>point.x>>point.y>>point.h)path.push_back(point);
    if(path.empty())throw std::runtime_error("empty reference path");
    size_t first=0;double nearest=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<path.size();++i){
      const double d=std::hypot(path[i].x-start.x,path[i].y-start.y)+.8*std::abs(wrap(path[i].h-start.h));
      if(d<nearest){nearest=d;first=i;}
    }
    size_t collision=0,rule=0,side=0;int passed=bits;
    std::ofstream detail(std::string(argv[1])+".reference.csv");
    detail<<"index,x,y,yaw,collision_clear,rule_clear,passage_bits\n";
    for(size_t i=first;i<path.size();++i){
      const Pose p=path[i];const bool clear=std::isfinite(cost(p)),legal=rule_clearance(p,buoys);
      const int event=i>first?passage(path[i-1],p,buoys):0;
      collision+=!clear;rule+=!legal;side+=event<0;if(event>=0)passed|=event;
      detail<<i<<','<<p.x<<','<<p.y<<','<<p.h<<','<<clear<<','<<legal<<','<<event<<'\n';
    }
    // Sample validation only: this does not create a connector from the FCU
    // pose, replan a path, grant motion or establish future traffic clearance.
    std::cout<<"{\"first_index\":"<<first<<",\"samples\":"<<path.size()-first
      <<",\"collision_blocked\":"<<collision<<",\"rule_blocked\":"<<rule
      <<",\"illegal_passages\":"<<side<<",\"passed_colors\":"<<passed<<"}\n";
    return 0;
  }
  DubinsEstimate dubins(goal);
  std::function<double(const Pose&)> estimate;
  if(std::string(argv[2])=="dubins")estimate=[&](const Pose &p){return dubins(p);};
  auto begin=std::chrono::steady_clock::now();
  try{
    auto result=search(start,goal,buoys,true,bits,cost,3.,estimate);
    std::cout<<"passed states="<<result.expanded<<" poses="<<result.path.size();
    std::ofstream path(std::string(argv[1])+"."+argv[2]+".path");
    for(const auto &p:result.path)path<<p.x<<' '<<p.y<<' '<<p.h<<'\n';
  }catch(const std::exception &e){std::cout<<"failed "<<e.what();}
  std::cout<<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<'\n';
}
