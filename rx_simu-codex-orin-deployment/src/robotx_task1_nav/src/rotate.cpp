#include <cmath>
#include <dwb_critics/rotate_to_goal.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace robotx_task1_nav {
// Upstream 1.1.20 RotateToGoal permanently latches its window. A floating hull
// can drift outside it while a non-stateful goal checker still requires position.
// Reset that latch when position is outside; retain the upstream yaw/braking logic.
class RecoverableRotateCritic : public dwb_critics::RotateToGoalCritic {
  double tolerance_=.4;
 public:
  void onInit() override {
    dwb_critics::RotateToGoalCritic::onInit();
    node_.lock()->get_parameter(dwb_plugin_name_+".xy_goal_tolerance",tolerance_);
  }
  bool prepare(const geometry_msgs::msg::Pose2D &p,const nav_2d_msgs::msg::Twist2D &v,
               const geometry_msgs::msg::Pose2D &g,const nav_2d_msgs::msg::Path2D &path) override {
    if(std::hypot(p.x-g.x,p.y-g.y)>tolerance_)reset();
    return dwb_critics::RotateToGoalCritic::prepare(p,v,g,path);
  }
};

}
PLUGINLIB_EXPORT_CLASS(robotx_task1_nav::RecoverableRotateCritic,dwb_core::TrajectoryCritic)
