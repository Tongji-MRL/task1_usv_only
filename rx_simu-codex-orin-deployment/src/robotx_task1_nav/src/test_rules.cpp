#include "robotx_task1_nav/rules.hpp"
#include "robotx_task1_nav/dubins_estimate.hpp"
#include <iostream>
using namespace robotx_task1_nav;
void check(bool value,const char *label){if(!value)throw std::runtime_error(label);}
int main(){
  std::vector<Buoy> b{{"red",0,-3,.3,1},{"green",0,3,.3,2}};
  check(passage({-1,0,0},{1,0,0},b)==3,"red right and green left");
  check(passage({1,0,pi},{-1,0,pi},b)==-1,"heading reverses sides");
  check(passage({-1,0,0},{-1,0,pi},b)==0,"pivot is not passage");
  b[0].state=2;check(passage({-1,0,0},{1,0,0},b)==-1,"state change invalidates straight path");
  b.push_back({"off",0,-1,.3,0});check(passage({-1,0,0},{1,0,0},b)==2,"nearest OFF shields farther marker");
  std::vector<Buoy> rotated{{"red",3,0,.3,1},{"green",-3,0,.3,2}};
  check(passage({0,-1,pi/2},{0,1,pi/2},rotated)==3,"ENU heading +90 degrees");
  b={{"red",0,-3,.3,1},{"green",0,3,.3,2}};
  auto cost=[](const Pose &p){return std::abs(p.x)<10&&std::abs(p.y)<10?0.:std::numeric_limits<double>::infinity();};
  auto result=search({-5,0,0},{5,0,0},b,true,0,cost);
  check(result.path.size()>10,"lattice route");int bits=0;
  for(size_t i=1;i<result.path.size();++i){int k=passage(result.path[i-1],result.path[i],b);check(k>=0,"route legality");bits|=k;}
  check(bits==3,"both colors observed");
  b.push_back({"ambiguous",0,-3,.3,0});check(passage({-1,0,0},{1,0,0},b)==-1,"ambiguous tie cannot be credited");
  auto terminal=search({-.35,-.35,-pi/2},{0,0,-pi/2},{},false,0,cost);
  check(terminal.path.size()>3,"cannot snap a half-metre backward terminal segment");
  bool blocked=false;
  try{search({-5,0,0},{5,0,0},{},false,0,[](const Pose &p){return std::abs(p.x)<.4||std::abs(p.x)>8||std::abs(p.y)>1?
      std::numeric_limits<double>::infinity():0.;},.5);}catch(const std::runtime_error&){blocked=true;}
  check(blocked,"blocked corridor returns failure");
  check(!rule_clearance({0,0,pi},{{"red",0,-3,.3,1}}),"wrong-side abeam has planning margin");
  check(rule_clearance({-2,0,pi},{{"red",0,-3,.3,1}}),"margin is local to abeam");
  DubinsEstimate heading_goal({5,0,-pi/2});
  check(heading_goal({5,0,pi/2})>5.,"opposite final heading requires a forward loop");
  auto curved=search({-5,0,0},{5,0,-pi/2},{},true,3,cost,3.,[&](const Pose &p){return heading_goal(p);});
  check(std::abs(wrap(curved.path.back().h+pi/2))<1e-9,"forward search satisfies final heading");
  for(size_t i=1;i<curved.path.size();++i){const auto &a=curved.path[i-1],&b=curved.path[i];
    check((b.x-a.x)*std::cos(a.h)+(b.y-a.y)*std::sin(a.h)>-1e-6,"heuristic cannot introduce reverse motion");}
  std::cout<<"16 rule/search checks passed; states="<<result.expanded<<"\n";
}
