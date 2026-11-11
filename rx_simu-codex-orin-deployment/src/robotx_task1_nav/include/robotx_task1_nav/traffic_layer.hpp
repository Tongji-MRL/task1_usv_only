#pragma once
#include <nav2_costmap_2d/static_layer.hpp>

namespace robotx_task1_nav {
// Humble 1.1.20 reads use_maximum at node scope. Also, its two merge helpers
// handle unknown cells differently in fixed/rolling maps. This additional
// layer only marks inferred lethal cells; an empty inference is not evidence
// of free water and must not clear any preceding layer or unknown cell.
class TrafficLayer : public nav2_costmap_2d::StaticLayer {
public:
  void updateCosts(nav2_costmap_2d::Costmap2D &master,int min_i,int min_j,int max_i,int max_j) override {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
    if(!enabled_||!map_received_in_update_bounds_)return;
    if(map_frame_!=global_frame_) {
      current_=false;
      RCLCPP_ERROR(logger_,"Traffic inference and costmap must share their configured map frame");
      return;
    }
    for(int j=min_j;j<max_j;++j)for(int i=min_i;i<max_i;++i){
      double x,y;unsigned mx,my;master.mapToWorld(i,j,x,y);
      if(worldToMap(x,y,mx,my)&&getCost(mx,my)==nav2_costmap_2d::LETHAL_OBSTACLE)
        master.setCost(i,j,nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    current_=true;
  }
};
}
