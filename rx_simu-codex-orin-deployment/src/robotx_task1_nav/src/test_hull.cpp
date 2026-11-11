#include "robotx_task1_nav/hull_grid.hpp"
#include "robotx_task1_nav/route_preference.hpp"
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <iostream>
using namespace robotx_task1_nav;
void check(bool value,const char *label){if(!value)throw std::runtime_error(label);}
int main(){
  nav2_costmap_2d::Costmap2D map(200,200,.05,-5,-5,0);
  std::vector<geometry_msgs::msg::Point> fp;
  for(auto xy:std::vector<std::array<double,2>>{{-2,-1},{2,-1},{2,1},{-2,1}}){geometry_msgs::msg::Point p;p.x=xy[0];p.y=xy[1];fp.push_back(p);}
  HullGrid hull;nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> edge(&map);
  map.setCost(124,105,254);hull.reset(map,fp);
  check(edge.footprintCostAtPose(0,0,0,fp)==0.,"fixture must evade the edge-only checker");
  check(hull.collides({0,0,0}),"interior lethal cell rejected");
  check(hull.unknownWithin(0,0,1.3),"unknown body inside visibility radius rejected");
  check(!hull.unknownWithin(0,0,.8),"distant unknown body excluded");
  hull.reset(map,fp,{{"off",1.225,.275,.36,0}});
  check(!hull.unknownWithin(0,0,1.3),"surveyed OFF body exempt only from visibility radius");
  check(hull.collides({0,0,0}),"known body still rejects physical hull intersection");
  check(!hull.collides({0,0,pi/2}),"rotated hull excludes the same cell");
  map.setCost(124,105,0);map.setCost(100,100,255);hull.reset(map,fp);
  check(hull.collides({0,0,0}),"unknown interior rejected");
  map.setCost(100,100,0);hull.reset(map,fp);check(!hull.collides({0,0,0}),"cleared cells disappear on refresh");
  map.setCost(140,100,254);hull.reset(map,fp);check(hull.collides({0,0,0}),"cell square touching hull edge rejected");
  RoutePreference preference;std::vector<Pose> route={{0,0,0},{1,0,0},{2,0,0}};
  check(preference.reset(map,route,{0,0,0},{2,0,0}),"same-goal reference accepted");
  check(preference.distance({1,2,0})>preference.distance({1,0,0})+1.,"continuity favors previous corridor");
  check(map.getCost(140,100)==254&&hull.collides({0,0,0}),"preference cannot clear a collision");
  check(!preference.reset(map,route,{0,0,0},{3,0,0}),"changed target invalidates preference");
  check(preference.distance({1,2,0})==0.,"invalid reference has no residual cost");
  check(!preference.reset(map,route,{0,0,pi},{2,0,0}),"opposite-direction leg is not reused");
  std::cout<<"16 hull, visibility and route preference checks passed\n";
}
