#pragma once
#include "robotx_task1_nav/rules.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace robotx_task1_nav {
// Soft cost only: callers must perform all current hull/rule checks first.
// This grid never writes into a measured or inferred obstacle layer.
class RoutePreference {
  const nav2_costmap_2d::Costmap2D *map_=nullptr;
  std::vector<float> distance_;
 public:
  bool reset(const nav2_costmap_2d::Costmap2D &map,const std::vector<Pose> &path,
             const Pose &start,const Pose &goal) {
    map_=nullptr;distance_.clear();
    if(std::any_of(path.begin(),path.end(),[](const Pose &p){return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.h);}))return false;
    if(path.empty()||std::hypot(path.back().x-goal.x,path.back().y-goal.y)>.5||
       std::abs(wrap(path.back().h-goal.h))>.25)return false;
    size_t first=0;double best=1e9;
    for(size_t i=0;i<path.size();++i){
      double d=std::hypot(path[i].x-start.x,path[i].y-start.y)+.8*std::abs(wrap(path[i].h-start.h));
      if(d<best){best=d;first=i;}
    }
    if(std::hypot(path[first].x-start.x,path[first].y-start.y)>2.||
       std::abs(wrap(path[first].h-start.h))>1.2)return false;
    map_=&map;const int width=map.getSizeInCellsX(),height=map.getSizeInCellsY();
    const double limit=6.,res=map.getResolution();const int radius=int(std::ceil(limit/res));
    distance_.assign(width*height,limit);
    for(size_t i=first;i<path.size();++i){
      unsigned mx,my;if(!map.worldToMap(path[i].x,path[i].y,mx,my))continue;
      for(int y=std::max(0,int(my)-radius);y<=std::min(height-1,int(my)+radius);++y)
        for(int x=std::max(0,int(mx)-radius);x<=std::min(width-1,int(mx)+radius);++x){
          double wx,wy;map.mapToWorld(x,y,wx,wy);
          auto &v=distance_[y*width+x];v=std::min(v,float(std::hypot(wx-path[i].x,wy-path[i].y)));
        }
    }
    return true;
  }
  double distance(const Pose &p)const {
    unsigned x,y;if(!map_||!map_->worldToMap(p.x,p.y,x,y))return 0.;
    return distance_[y*map_->getSizeInCellsX()+x];
  }
};
} // namespace
