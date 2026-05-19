#pragma once

#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/buffer.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <people_msgs/People.h>
#include <hrvo_local_planner/HRVOInput.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>

#include <hrvo/Simulator.h>
#include <hrvo/Vector2.h>

namespace hrvo_local_planner {

class HRVOLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  HRVOLocalPlanner();

  void initialize(std::string name,
                  tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  struct OtherAgent
  {
    std::string name;
    hrvo::Vector2 pos;
    hrvo::Vector2 vel;
    float radius{0.35f};
  };

  struct TrackedPersonState
  {
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    ros::Time stamp;
    double radius{0.40};
  };

  struct RolloutEval
  {
    bool safe{true};
    double avg_cost_norm{0.0};
    double max_cost_norm{0.0};
    double min_clearance{10.0};
    double x_end{0.0};
    double y_end{0.0};
    double yaw_end{0.0};
  };

  // callbacks
  void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg);
  void trackedPeopleCb(const people_msgs::People::ConstPtr& msg);
  void hrvoInputCb(const hrvo_local_planner::HRVOInput::ConstPtr& msg);

  // helpers
  bool getRobotPose(geometry_msgs::PoseStamped& pose_out) const;
  geometry_msgs::PoseStamped getLookaheadTarget(
      const geometry_msgs::PoseStamped& robot_pose) const;

  std::vector<OtherAgent> buildOtherAgents() const;
  void prioritizeOtherAgents(std::vector<OtherAgent>& agents,
                               double robot_x, double robot_y) const;
  std::vector<OtherAgent> buildOtherAgentsFromHRVOInput() const;
  std::vector<OtherAgent> buildOtherAgentsFromGazebo() const;
  std::vector<OtherAgent> buildOtherAgentsFromTrackedPeople() const;

  /** True if candidate is within duplicate_radius (m) of any agent already in out. */
  static bool nearExistingAgent(const OtherAgent& candidate,
                                const std::vector<OtherAgent>& out,
                                double duplicate_radius);

  void resetSimulator(const geometry_msgs::PoseStamped& robot_pose,
                      const geometry_msgs::PoseStamped& target);

  bool isStateSafe(double x, double y) const;
  bool isCmdSafe(const geometry_msgs::PoseStamped& robot_pose,
                 const geometry_msgs::Twist& cmd) const;

  double pointCostNorm(double x, double y) const;
  double estimateObstacleDistance(double x, double y, double max_probe) const;
  RolloutEval evaluateRollout(const geometry_msgs::PoseStamped& robot_pose,
                              double v_cmd, double w_cmd, double horizon) const;

  geometry_msgs::Twist fallbackCmd(const geometry_msgs::PoseStamped& robot_pose,
                                   const geometry_msgs::PoseStamped& target) const;

  void publishLocalPlanRollout(const geometry_msgs::PoseStamped& robot_pose,
                               double vx, double vy) const;

private:
  // ROS/core
  bool initialized_{false};
  bool sim_initialized_{false};

  tf2_ros::Buffer* tf_{nullptr};
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  costmap_2d::Costmap2D* costmap_{nullptr};

  ros::Subscriber model_states_sub_;
  ros::Subscriber tracked_people_sub_;
  ros::Subscriber hrvo_input_sub_;
  ros::Publisher local_plan_pub_;

  // global plan
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped goal_;

  // simulator
  std::unique_ptr<hrvo::Simulator> sim_;
  std::size_t robot_agent_id_{0};
  std::size_t robot_goal_id_{0};
  hrvo::Vector2 last_goal_pos_{0.0f, 0.0f};
  std::unordered_map<std::string, std::size_t> other_agent_ids_;

  // timing/cmd
  ros::Time last_time_;
  geometry_msgs::Twist last_cmd_;
  int step_count_{0};

  // gazebo agent cache
  bool have_model_states_{false};
  gazebo_msgs::ModelStates last_model_states_;
  ros::Time last_model_states_stamp_;

  // tracked people cache
  std::unordered_map<std::string, TrackedPersonState> tracked_people_;
  ros::Time last_people_stamp_;

  // fused HRVO input cache
  bool have_hrvo_input_{false};
  hrvo_local_planner::HRVOInput last_hrvo_input_;
  ros::Time last_hrvo_input_stamp_;

    // limits
  double max_vel_x_;
  double max_vel_y_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_y_;
  double acc_lim_theta_;

  // goal / tracking
  double xy_goal_tol_;
  double yaw_goal_tol_;
  double lookahead_dist_;
  int goal_reset_every_;
  double goal_update_dist_;
  double min_vnorm_dir_;

  // hrvo params
  double neighbor_dist_;
  int max_neighbors_;
  int max_other_agents_for_hrvo_;
  double static_agent_speed_thresh_;
  double robot_radius_;
  double goal_radius_;
  double pref_speed_;
  double max_speed_;
  double uncertainty_offset_;
  double max_accel_;
  double time_step_;

  // command mapping
  bool holonomic_mode_{false};
  double heading_kp_;
  double heading_brake_angle_;
  double min_speed_turning_scale_;
  bool slowdown_cos_;
  double stop_yaw_error_;
  double rotate_enter_error_;
  double rotate_exit_error_;
  bool rotate_in_place_mode_{true};
  bool allow_backward_{true};
  double max_vel_x_backwards_{-1.0};  // <0: dùng max_vel_x_

  double maxVelXBackwards() const {
    return (max_vel_x_backwards_ > 0.0) ? max_vel_x_backwards_ : max_vel_x_;
  }

  // gazebo agents
  bool use_gazebo_agents_;
  std::string robot_model_name_;
  std::string other_model_prefix_;
  bool include_humans_;
  std::string gazebo_frame_;
  std::string human_prefix_;
  double human_radius_;

  // tracked people
  bool use_tracked_people_;
  std::string tracked_people_topic_;
  double tracked_people_timeout_;
  double tracked_person_radius_;
  double people_velocity_alpha_;
  double min_people_speed_for_hrvo_;

  // unified input
  bool use_hrvo_input_topic_;
  std::string hrvo_input_topic_;
  double hrvo_input_timeout_;
  bool use_hrvo_input_robot_state_;

  /**
   * Dynamic obstacle fusion for HRVO other-agents.
   * - merge: HRVOInput agents first, then Gazebo / tracked people not within
   *   agent_duplicate_radius_ of an already-added agent (meters).
   * - hrvo_input_only: only /hrvo/input (or configured topic) agents.
   */
  std::string agent_fusion_policy_;
  double agent_duplicate_radius_;
  std::string gazebo_model_states_topic_;

  // robustness
  bool allow_world_as_global_when_tf_fails_;
  double model_states_timeout_;

  // safety / costmap
  bool use_costmap_safety_;
  int obstacle_cost_threshold_;
  double safety_rollout_time_;
  double safety_dt_;
  double safety_radius_padding_;
  double clearance_probe_dist_;
  double clearance_weight_;
  double obstacle_cost_weight_;
  double jerk_weight_;
  double hrvo_align_weight_;
  double goal_align_weight_;
  double speed_weight_;
  double turn_weight_;
  double spin_weight_;
};

} // namespace hrvo_local_planner
