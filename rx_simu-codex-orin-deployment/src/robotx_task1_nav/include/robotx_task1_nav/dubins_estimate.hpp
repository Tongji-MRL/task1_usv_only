#pragma once
#include "robotx_task1_nav/rules.hpp"
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace robotx_task1_nav {
// A heading-aware estimate for the forward-only TRANSIT lattice. The chord
// length / turn angle defines its smallest circular turning radius. This is a
// weighted search ordering, not a collision-free path or optimality guarantee.
class DubinsEstimate {
  ompl::base::DubinsStateSpace space_{.6/(2*std::sin(pi/18))};
  ompl::base::State *from_,*to_;
 public:
  explicit DubinsEstimate(const Pose &goal):from_(space_.allocState()),to_(space_.allocState()) {
    auto p=to_->as<ompl::base::SE2StateSpace::StateType>();p->setXY(goal.x,goal.y);p->setYaw(goal.h);
  }
  ~DubinsEstimate(){space_.freeState(from_);space_.freeState(to_);}
  DubinsEstimate(const DubinsEstimate&)=delete;
  DubinsEstimate &operator=(const DubinsEstimate&)=delete;
  double operator()(const Pose &pose) {
    auto p=from_->as<ompl::base::SE2StateSpace::StateType>();p->setXY(pose.x,pose.y);p->setYaw(pose.h);
    return space_.distance(from_,to_);
  }
};
}
