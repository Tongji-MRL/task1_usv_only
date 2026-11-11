#pragma once
// Geometric rule checking and forward SE(2) lattice search. All distances in metres.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotx_task1_nav {
constexpr double pi=3.14159265358979323846;
inline double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
struct Pose {double x,y,h;};
struct Buoy {std::string id;double x,y,r;int state;};
inline Pose interpolate(const Pose &a,const Pose &b,double t){
  return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),wrap(a.h+t*wrap(b.h-a.h))};
}
inline std::array<double,2> body(const Pose &p,const Buoy &b){
  const double dx=b.x-p.x,dy=b.y-p.y;
  return {std::cos(p.h)*dx+std::sin(p.h)*dy,-std::sin(p.h)*dx+std::cos(p.h)*dy};
}
// Positive bitmask is observed red/green; -1 is an illegal side event.
inline int passage(const Pose &a,const Pose &b,const std::vector<Buoy> &buoys) {
  if(std::hypot(b.x-a.x,b.y-a.y)<1e-6)return 0; // Heading-only rotation is not transit.
  int bits=0;
  for(const auto &marker:buoys) {
    if(marker.state!=1&&marker.state!=2)continue;
    const double x0=body(a,marker)[0],x1=body(b,marker)[0];
    if(x0*x1>0||std::abs(x1-x0)<1e-8)continue;
    double lo=0,hi=1;
    for(int k=0;k<20;++k){double mid=(lo+hi)/2;if(body(interpolate(a,b,mid),marker)[0]*x0>0)lo=mid;else hi=mid;}
    const Pose at=interpolate(a,b,(lo+hi)/2);const auto rel=body(at,marker);
    const double distance=std::hypot(marker.x-at.x,marker.y-at.y);
    bool nearest=true,tie=false;
    for(const auto &other:buoys) {
      if(other.id==marker.id)continue;
      if(body(at,other)[1]*rel[1]>0){
        double delta=std::hypot(other.x-at.x,other.y-at.y)-distance;
        if(delta<-.01){nearest=false;break;}
        if(std::abs(delta)<=.01)tie=true;
      }
    }
    if(!nearest)continue;
    if(tie)return -1;
    if((marker.state==1&&rel[1]>=0)||(marker.state==2&&rel[1]<=0))return -1;
    bits|=marker.state==1?1:2;
  }
  return bits;
}

// Keep the planned trajectory away from a forbidden abeam crossing. This is a
// planning margin only; the independent judge still evaluates the exact rule.
inline bool rule_clearance(const Pose &p,const std::vector<Buoy> &buoys) {
  for(const auto &b:buoys) {
    if(b.state!=1&&b.state!=2)continue;
    const auto rel=body(p,b);
    if(std::abs(rel[0])>1.0||(b.state==1?rel[1]<0:rel[1]>0))continue;
    const double d=std::hypot(b.x-p.x,b.y-p.y);bool shielded=false;
    for(const auto &other:buoys)if(other.id!=b.id&&body(p,other)[1]*rel[1]>0&&
        std::hypot(other.x-p.x,other.y-p.y)<d-.3){shielded=true;break;}
    if(!shielded)return false;
  }return true;
}

struct SearchResult {std::vector<Pose> path;size_t expanded=0;};
// Collision callback returns nonnegative traversal cost, infinity for forbidden hull poses.
inline SearchResult search(const Pose &start,const Pose &goal,const std::vector<Buoy> &buoys,
                           bool enforce,int initial_bits,const std::function<double(const Pose&)> &cost,
                           double timeout=3.,
                           const std::function<double(const Pose&)> &estimate={}) {
  struct Node {Pose p;double g;int parent,bits;};
  struct Q {double f;int i;bool operator<(const Q &o)const{return f>o.f;}};
  std::vector<Node> nodes;nodes.reserve(100000);std::priority_queue<Q> open;
  std::unordered_map<std::string,double> best;
  auto key=[](const Pose &p,int bits){return std::to_string(std::lround(p.x/.4))+","+
      std::to_string(std::lround(p.y/.4))+","+std::to_string(int(std::lround(wrap(p.h)*36/(2*pi))+36)%36)+","+std::to_string(bits);};
  // Weighted A*: favor timely legal routes over optimal length. Collision and
  // passage constraints remain hard checks on every interpolated edge.
  auto heuristic=[&](const Pose &p){return 1.4*(estimate?estimate(p):
      std::hypot(goal.x-p.x,goal.y-p.y)+.25*std::abs(wrap(goal.h-p.h)));};
  auto edge=[&](const Pose &a,const Pose &b,int &bits,double &penalty){
    int n=std::max(2,int(std::ceil(std::max(std::hypot(b.x-a.x,b.y-a.y)/.1,std::abs(wrap(b.h-a.h))/.06))));
    Pose previous=a;penalty=0;
    for(int j=1;j<=n;++j){const Pose q=interpolate(a,b,double(j)/n);double v=cost(q);
      if(!std::isfinite(v))return false;
      if(enforce){if(!rule_clearance(q,buoys))return false;int result=passage(previous,q,buoys);if(result<0)return false;bits|=result;}
      penalty+=v/n;previous=q;
    }return true;
  };
  if(!std::isfinite(cost(start)))throw std::runtime_error("start footprint is occupied");
  if(!std::isfinite(cost(goal)))throw std::runtime_error("goal footprint is occupied");
  nodes.push_back({start,0,-1,initial_bits});open.push({heuristic(start),0});best[key(start,initial_bits)]=0;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
  SearchResult result;
  while(!open.empty()&&nodes.size()<180000) {
    const int index=open.top().i;open.pop();const Node current=nodes[index];
    if(current.g>best[key(current.p,current.bits)]+1e-8)continue;
    if((++result.expanded%128)==0&&std::chrono::steady_clock::now()>deadline)break;
    const double goal_distance=std::hypot(goal.x-current.p.x,goal.y-current.p.y);
    const double goal_bearing=std::atan2(goal.y-current.p.y,goal.x-current.p.x);
    // A broad endpoint snap must be forward and heading-consistent. Otherwise
    // search to within half a position cell; do not create a sideways/reverse tail.
    if(goal_distance<.65&&std::abs(wrap(goal.h-current.p.h))<.36&&
       (goal_distance<.2||std::abs(wrap(goal_bearing-current.p.h))<.3)) {
      int bits=current.bits;double penalty;
      if(edge(current.p,goal,bits,penalty)&&(!enforce||bits==3)) {
        result.path.push_back(goal);
        for(int i=index;i>=0;i=nodes[i].parent)result.path.push_back(nodes[i].p);
        std::reverse(result.path.begin(),result.path.end());return result;
      }
    }
    for(int action=0;action<(enforce?5:7);++action) {
      // DWB tracks forward curves; a mid-course pivot is unsafe on a drifting
      // vessel near an abeam boundary. Pivots remain available for approach.
      const bool pivot=action>=5;
      const double turn=(pivot?(action==5?-1:1):action-2)*2*pi/36;
      const double length=pivot?0:.6;
      Pose next{current.p.x+length*std::cos(current.p.h+turn/2),
                current.p.y+length*std::sin(current.p.h+turn/2),wrap(current.p.h+turn)};
      int bits=current.bits;double penalty;
      if(!edge(current.p,next,bits,penalty))continue;
      const double g=current.g+(pivot?.28:.6)+.1*std::abs(turn)+penalty*(pivot?.1:.6);
      const auto k=key(next,bits);auto it=best.find(k);if(it!=best.end()&&it->second<=g)continue;
      best[k]=g;nodes.push_back({next,g,index,bits});open.push({g+heuristic(next),int(nodes.size()-1)});
    }
  }
  throw std::runtime_error("no legal path within lattice search budget (expanded "+std::to_string(result.expanded)+")");
}
} // namespace
