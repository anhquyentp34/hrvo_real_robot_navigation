#include <hrvo/hrvo_local_planner.h>

#include <pluginlib/class_list_macros.h>
#include <memory>

namespace hrvo_local_planner {

HRVOLocalPlanner::HRVOLocalPlanner()
{
  last_time_ = ros::Time::now();
  last_cmd_ = geometry_msgs::Twist();
  sim_initialized_ = false;
}

void HRVOLocalPlanner::initialize(std::string name,
                                  tf2_ros::Buffer* tf,
                                  costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) return;

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  ros::NodeHandle nh("~/" + name);

  // move_base style params
  nh.param("max_vel_x", max_vel_x_, 0.5);
  nh.param("max_vel_theta", max_vel_theta_, 1.0);
  nh.param("acc_lim_x", acc_lim_x_, 1.0);
  nh.param("acc_lim_theta", acc_lim_theta_, 2.0);

  nh.param("xy_goal_tolerance", xy_goal_tol_, 0.2);
  nh.param("yaw_goal_tolerance", yaw_goal_tol_, 0.2);
  nh.param("lookahead_dist", lookahead_dist_, 0.8);

  // HRVO params
  nh.param("neighbor_dist", neighbor_dist_, 5.0);
  nh.param("max_neighbors", max_neighbors_, 20);
  nh.param("robot_radius", robot_radius_, 0.30);
  nh.param("goal_radius", goal_radius_, 0.20);
  nh.param("pref_speed", pref_speed_, 0.4);
  nh.param("max_speed", max_speed_, 0.6);
  nh.param("uncertainty_offset", uncertainty_offset_, 0.0);
  nh.param("max_accel", max_accel_, 1.0);
  nh.param("time_step", time_step_, 0.1);

  // diff-drive mapping
  nh.param("heading_kp", heading_kp_, 2.0);
  nh.param("slowdown_cos", slowdown_cos_, true);
  nh.param("stop_yaw_error", stop_yaw_error_, 1.2);

  nh.param("goal_reset_every", goal_reset_every_, 200);

  last_time_ = ros::Time::now();
  last_cmd_ = geometry_msgs::Twist();
  
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);

  initialized_ = true;
  ROS_INFO_STREAM("HRVOLocalPlanner initialized. plugin name=" << name);
}

bool HRVOLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_) return false;

  global_plan_ = orig_global_plan;
  if (!global_plan_.empty()) goal_ = global_plan_.back();
  return true;
}

bool HRVOLocalPlanner::getRobotPose(geometry_msgs::PoseStamped& pose_out) const
{
  return costmap_ros_ && costmap_ros_->getRobotPose(pose_out);
}

geometry_msgs::PoseStamped HRVOLocalPlanner::getLookaheadTarget(const geometry_msgs::PoseStamped& robot_pose) const
{
  if (global_plan_.empty()) return robot_pose;

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  for (const auto& p : global_plan_) {
    const double dx = p.pose.position.x - rx;
    const double dy = p.pose.position.y - ry;
    if (std::hypot(dx, dy) >= lookahead_dist_) return p;
  }
  return global_plan_.back();
}

void HRVOLocalPlanner::resetSimulator(const geometry_msgs::PoseStamped& robot_pose,
                                      const geometry_msgs::PoseStamped& target)
{
  sim_ = std::make_unique<hrvo::Simulator>();  // reset object
  sim_->setTimeStep(static_cast<float>(time_step_));

  const float rx = static_cast<float>(robot_pose.pose.position.x);
  const float ry = static_cast<float>(robot_pose.pose.position.y);
  const float yaw = static_cast<float>(getYaw(robot_pose));

  const float tx = static_cast<float>(target.pose.position.x);
  const float ty = static_cast<float>(target.pose.position.y);

  last_goal_pos_ = hrvo::Vector2(tx, ty);
  robot_goal_id_ = sim_->addGoal(last_goal_pos_);

  robot_agent_id_ = sim_->addAgent(
    hrvo::Vector2(rx, ry),
    robot_goal_id_,
    static_cast<float>(neighbor_dist_),
    static_cast<std::size_t>(std::max(1, max_neighbors_)),
    static_cast<float>(robot_radius_),
    static_cast<float>(goal_radius_),
    static_cast<float>(pref_speed_),
    static_cast<float>(max_speed_),
    static_cast<float>(uncertainty_offset_),
    static_cast<float>(max_accel_),
    hrvo::Vector2(0.f, 0.f),
    yaw
  );

  sim_initialized_ = true;
  step_count_ = 0;
}

bool HRVOLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_ || global_plan_.empty()) return false;

  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) return false;

  const double d_goal = dist2D(robot_pose, goal_);
  const double yaw = getYaw(robot_pose);
  const double goal_yaw = getYaw(goal_);
  const double dyaw_goal = angles::shortest_angular_distance(yaw, goal_yaw);

  // goal reached: align yaw
  if (d_goal <= xy_goal_tol_) {
    geometry_msgs::Twist out;
    if (std::fabs(dyaw_goal) <= yaw_goal_tol_) {
      cmd_vel = geometry_msgs::Twist();
      last_cmd_ = cmd_vel;
      return true;
    }
    out.linear.x = 0.0;
    out.angular.z = clamp(dyaw_goal, -max_vel_theta_, max_vel_theta_);
    cmd_vel = out;
    last_cmd_ = cmd_vel;
    return true;
  }

  // pick local target
  const geometry_msgs::PoseStamped target = getLookaheadTarget(robot_pose);

  // init/reset sim
  if (!sim_initialized_) {
    resetSimulator(robot_pose, target);
  }

  // avoid goal growth: reset periodically (vì lib thường không có setGoalPosition)
  if (goal_reset_every_ > 0 && step_count_ >= goal_reset_every_) {
    resetSimulator(robot_pose, target);
  }

  // update robot pose
  sim_->setAgentPosition(robot_agent_id_,
                         hrvo::Vector2(static_cast<float>(robot_pose.pose.position.x),
                                       static_cast<float>(robot_pose.pose.position.y)));
  sim_->setAgentOrientation(robot_agent_id_, static_cast<float>(yaw));

  // update goal: nếu target thay đổi nhiều thì add goal mới + trỏ agent sang goal mới
  const float tx = static_cast<float>(target.pose.position.x);
  const float ty = static_cast<float>(target.pose.position.y);
  const hrvo::Vector2 new_goal(tx, ty);

  const float gdx = new_goal.getX() - last_goal_pos_.getX();
  const float gdy = new_goal.getY() - last_goal_pos_.getY();
  const float gdist = std::hypot(gdx, gdy);

  if (gdist > 0.15f) {
    last_goal_pos_ = new_goal;
    robot_goal_id_ = sim_->addGoal(last_goal_pos_);
    sim_->setAgentGoal(robot_agent_id_, robot_goal_id_);
  }

  // TODO: add neighbors/agents here (humans/robots khác)
  // sim_->setAgentPosition(nei_id,...), sim_->setAgentVelocity(nei_id,...)

  sim_->doStep();
  ++step_count_;

  // HRVO output: holonomic velocity (vx, vy)
  const hrvo::Vector2 v = sim_->getAgentVelocity(robot_agent_id_);
  const double vx = static_cast<double>(v.getX());
  const double vy = static_cast<double>(v.getY());

  // map to diff-drive (v, w)
  const double vnorm = std::hypot(vx, vy);
  const double vdir = std::atan2(vy, vx);
  const double dyaw = angles::shortest_angular_distance(yaw, vdir);

  geometry_msgs::Twist out;

  if (std::fabs(dyaw) > stop_yaw_error_) {
    out.linear.x = 0.0;
    out.angular.z = clamp(heading_kp_ * dyaw, -max_vel_theta_, max_vel_theta_);
  } else {
    double v_cmd = std::min(vnorm, max_vel_x_);
    if (slowdown_cos_) v_cmd *= std::max(0.0, std::cos(dyaw));
    out.linear.x = clamp(v_cmd, 0.0, max_vel_x_);
    out.angular.z = clamp(heading_kp_ * dyaw, -max_vel_theta_, max_vel_theta_);
  }

  // accel clamp (move_base style)
  const ros::Time now = ros::Time::now();
  const double dt = std::max(1e-3, (now - last_time_).toSec());
  last_time_ = now;

  const double dv_max = acc_lim_x_ * dt;
  const double dw_max = acc_lim_theta_ * dt;

  out.linear.x = clamp(out.linear.x,  last_cmd_.linear.x  - dv_max, last_cmd_.linear.x  + dv_max);
  out.angular.z = clamp(out.angular.z, last_cmd_.angular.z - dw_max, last_cmd_.angular.z + dw_max);

  // final clamp
  out.linear.x  = clamp(out.linear.x,  -max_vel_x_,     max_vel_x_);
  out.angular.z = clamp(out.angular.z, -max_vel_theta_, max_vel_theta_);

  cmd_vel = out;
  last_cmd_ = cmd_vel;

  // --- CHÈN ĐOẠN NÀY ĐỂ HIỆN TOPIC ---
  if (initialized_) {
    nav_msgs::Path path;
    path.header.frame_id = costmap_ros_->getGlobalFrameID(); // Thường là odom hoặc map
    path.header.stamp = ros::Time::now();

    // Điểm 1: Vị trí hiện tại của robot
    geometry_msgs::PoseStamped p;
    p.header = path.header;
    p.pose = robot_pose.pose;
    path.poses.push_back(p);

    // Điểm 2: Vị trí đích local (target) mà HRVO đang hướng tới
    geometry_msgs::PoseStamped p_target;
    p_target.header = path.header;
    p_target.pose = target.pose;
    path.poses.push_back(p_target);

    local_plan_pub_.publish(path);
  }
  // ------------------------------------
  return true;
}

bool HRVOLocalPlanner::isGoalReached()
{
  if (!initialized_ || global_plan_.empty()) return false;

  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) return false;

  const double d = dist2D(robot_pose, goal_);
  const double yaw = getYaw(robot_pose);
  const double goal_yaw = getYaw(goal_);
  const double dyaw = angles::shortest_angular_distance(yaw, goal_yaw);

  return (d <= xy_goal_tol_) && (std::fabs(dyaw) <= yaw_goal_tol_);
}

} // namespace hrvo_local_planner

PLUGINLIB_EXPORT_CLASS(hrvo_local_planner::HRVOLocalPlanner, nav_core::BaseLocalPlanner)