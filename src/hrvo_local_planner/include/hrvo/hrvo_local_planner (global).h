// FILE: include/hrvo/hrvo_local_planner.h
#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf2/utils.h>

#include <gazebo_msgs/ModelStates.h>
#include <hrvo/hrvo.h>

namespace hrvo_local_planner {

class HRVOLocalPlanner : public nav_core::BaseLocalPlanner {
public:
  HRVOLocalPlanner();

  void initialize(std::string name,
                  tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  bool isGoalReached() override;

private:
  static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  static inline double dist2D(const geometry_msgs::PoseStamped& a,
                              const geometry_msgs::PoseStamped& b) {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }

  static inline double getYaw(const geometry_msgs::PoseStamped& p) {
    return tf2::getYaw(p.pose.orientation);
  }

  geometry_msgs::PoseStamped getLookaheadTarget(const geometry_msgs::PoseStamped& robot_pose) const;
  bool getRobotPose(geometry_msgs::PoseStamped& pose_out) const;

  void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg);

  struct OtherAgent {
    std::string name;
    hrvo::Vector2 pos;
    hrvo::Vector2 vel;
    float radius;
  };
  std::vector<OtherAgent> buildOtherAgents() const;

  void resetSimulator(const geometry_msgs::PoseStamped& robot_pose,
                      const geometry_msgs::PoseStamped& target);

  void publishLocalPlanRollout(const geometry_msgs::PoseStamped& robot_pose,
                             double vx, double vy) const;

  bool isCmdSafe(const geometry_msgs::PoseStamped& robot_pose,
                 const geometry_msgs::Twist& cmd) const;
  double distanceToWall(double x, double y) const;
  geometry_msgs::Twist fallbackCmd(const geometry_msgs::PoseStamped& robot_pose,
                                   const geometry_msgs::PoseStamped& target) const;

  // ROS / move_base
  bool initialized_{false};
  tf2_ros::Buffer* tf_{nullptr};
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  costmap_2d::Costmap2D* costmap_{nullptr};

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped goal_;

  ros::Publisher local_plan_pub_;

  // gazebo model states
  ros::Subscriber model_states_sub_;
  gazebo_msgs::ModelStates last_model_states_;
  bool have_model_states_{false};
  ros::Time last_model_states_stamp_;

  // filtering
  std::string robot_model_name_;
  std::string other_model_prefix_;
  bool use_gazebo_agents_{true};
  bool include_humans_{true};

  // move_base style params
  double max_vel_x_{0.8};
  double max_vel_theta_{1.0};
  double acc_lim_x_{1.0};
  double acc_lim_theta_{2.0};

  double xy_goal_tol_{0.2};
  double yaw_goal_tol_{0.2};
  double lookahead_dist_{0.8};

  // HRVO params
  double neighbor_dist_{6.0};
  int    max_neighbors_{25};
  double robot_radius_{0.30};
  double goal_radius_{0.20};
  double pref_speed_{0.4};
  double max_speed_{0.6};
  double uncertainty_offset_{0.10};
  double max_accel_{1.0};
  double time_step_{0.1};

  // diff-drive mapping
  double heading_kp_{2.0};
  bool slowdown_cos_{true};
  double stop_yaw_error_{1.2};

  // runtime
  ros::Time last_time_;
  geometry_msgs::Twist last_cmd_;

  // HRVO sim state
  bool sim_initialized_{false};
  std::unique_ptr<hrvo::Simulator> sim_;
  std::size_t robot_agent_id_{0};
  std::size_t robot_goal_id_{0};
  hrvo::Vector2 last_goal_pos_{0.f, 0.f};

  std::unordered_map<std::string, std::size_t> other_agent_ids_;

  int goal_reset_every_{200};
  int step_count_{0};

  // extras
  std::string gazebo_frame_{"world"};
  std::string human_prefix_{"human"};
  double human_radius_{0.45};

  bool use_costmap_safety_{true};
  int obstacle_cost_threshold_{254};  // default: lethal only
  double safety_rollout_time_{1.0};
  double safety_dt_{0.1};

  double desired_wall_clearance_{0.50};
  double min_wall_clearance_{0.05};

  double goal_update_dist_{0.35};
  double min_vnorm_dir_{1e-3};
};

}  // namespace hrvo_local_planner
