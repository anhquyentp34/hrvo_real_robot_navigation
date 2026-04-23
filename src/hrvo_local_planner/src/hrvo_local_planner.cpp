#include <hrvo/hrvo_local_planner.h>

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_set>
#include <vector>

namespace hrvo_local_planner {

static inline bool isFinite(double x) { return std::isfinite(x); }

namespace {
std::mutex g_model_states_mtx;
std::mutex g_people_mtx;
}

static inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

static inline double poseYaw(const geometry_msgs::PoseStamped& p)
{
  return tf2::getYaw(p.pose.orientation);
}

static inline double dist2DPoses(const geometry_msgs::PoseStamped& a,
                                 const geometry_msgs::PoseStamped& b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

static inline bool transformPose2D(tf2_ros::Buffer* tf,
                                   const std::string& from_frame,
                                   const std::string& to_frame,
                                   const geometry_msgs::Pose& in_pose,
                                   geometry_msgs::Pose& out_pose)
{
  if (!tf || from_frame == to_frame || from_frame.empty() || to_frame.empty()) {
    out_pose = in_pose;
    return true;
  }

  geometry_msgs::PoseStamped ps_in, ps_out;
  ps_in.header.frame_id = from_frame;
  ps_in.header.stamp = ros::Time(0);
  ps_in.pose = in_pose;

  try {
    ps_out = tf->transform(ps_in, to_frame, ros::Duration(0.05));
    out_pose = ps_out.pose;
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

static inline bool transformTwist2D(tf2_ros::Buffer* tf,
                                    const std::string& from_frame,
                                    const std::string& to_frame,
                                    const geometry_msgs::Twist& in_tw,
                                    geometry_msgs::Twist& out_tw)
{
  out_tw = in_tw;
  if (!tf || from_frame == to_frame || from_frame.empty() || to_frame.empty()) {
    return true;
  }

  try {
    const geometry_msgs::TransformStamped T =
        tf->lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(0.05));

    const double yaw = tf2::getYaw(T.transform.rotation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    const double vx = in_tw.linear.x;
    const double vy = in_tw.linear.y;

    out_tw.linear.x = c * vx - s * vy;
    out_tw.linear.y = s * vx + c * vy;
    out_tw.angular.z = in_tw.angular.z;
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

HRVOLocalPlanner::HRVOLocalPlanner()
{
  last_time_ = ros::Time::now();
  last_cmd_ = geometry_msgs::Twist();
  sim_initialized_ = false;
  initialized_ = false;
  have_model_states_ = false;
  step_count_ = 0;
}

void HRVOLocalPlanner::modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(g_model_states_mtx);
  last_model_states_ = *msg;
  have_model_states_ = true;
  last_model_states_stamp_ = ros::Time::now();
}

void HRVOLocalPlanner::trackedPeopleCb(const people_msgs::People::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(g_people_mtx);

  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  const std::string people_frame = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  const std::string global_frame = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : "map";

  std::unordered_map<std::string, TrackedPersonState> new_people;

  for (std::size_t i = 0; i < msg->people.size(); ++i) {
    const auto& p = msg->people[i];

    std::string id = p.name.empty() ? ("person_" + std::to_string(i)) : p.name;

    geometry_msgs::Pose pose_src;
    pose_src.position = p.position;
    pose_src.orientation.w = 1.0;

    geometry_msgs::Pose pose_dst;
    if (!transformPose2D(tf_, people_frame, global_frame, pose_src, pose_dst)) {
      if (!allow_world_as_global_when_tf_fails_) continue;
      pose_dst = pose_src;
    }

    TrackedPersonState state;
    state.pose = pose_dst;
    state.stamp = stamp;
    state.radius = tracked_person_radius_;
    state.twist.linear.x = 0.0;
    state.twist.linear.y = 0.0;
    state.twist.angular.z = 0.0;

    auto it_old = tracked_people_.find(id);
    if (it_old != tracked_people_.end()) {
      const double dt = std::max(1e-3, (stamp - it_old->second.stamp).toSec());
      const double raw_vx = (pose_dst.position.x - it_old->second.pose.position.x) / dt;
      const double raw_vy = (pose_dst.position.y - it_old->second.pose.position.y) / dt;

      state.twist.linear.x =
          people_velocity_alpha_ * raw_vx +
          (1.0 - people_velocity_alpha_) * it_old->second.twist.linear.x;

      state.twist.linear.y =
          people_velocity_alpha_ * raw_vy +
          (1.0 - people_velocity_alpha_) * it_old->second.twist.linear.y;
    }

    new_people[id] = state;
  }

  tracked_people_.swap(new_people);
  last_people_stamp_ = stamp;
}

void HRVOLocalPlanner::initialize(std::string name,
                                  tf2_ros::Buffer* tf,
                                  costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) return;

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;

  ros::NodeHandle nh("~/" + name);

  nh.param("max_vel_x", max_vel_x_, 0.8);
  nh.param("max_vel_theta", max_vel_theta_, 1.0);
  nh.param("acc_lim_x", acc_lim_x_, 1.0);
  nh.param("acc_lim_theta", acc_lim_theta_, 2.0);

  nh.param("xy_goal_tolerance", xy_goal_tol_, 0.20);
  nh.param("yaw_goal_tolerance", yaw_goal_tol_, 0.20);
  nh.param("lookahead_dist", lookahead_dist_, 0.80);
  nh.param("goal_reset_every", goal_reset_every_, 20);
  nh.param("goal_update_dist", goal_update_dist_, 0.35);
  nh.param("min_vnorm_dir", min_vnorm_dir_, 1e-3);

  nh.param("neighbor_dist", neighbor_dist_, 8.0);
  nh.param("max_neighbors", max_neighbors_, 30);
  nh.param("robot_radius", robot_radius_, 0.30);
  nh.param("goal_radius", goal_radius_, 0.20);
  nh.param("pref_speed", pref_speed_, 0.55);
  nh.param("max_speed", max_speed_, 0.80);
  nh.param("uncertainty_offset", uncertainty_offset_, 0.15);
  nh.param("max_accel", max_accel_, 1.2);
  nh.param("time_step", time_step_, 0.10);

  nh.param("heading_kp", heading_kp_, 2.5);
  nh.param("slowdown_cos", slowdown_cos_, true);
  nh.param("stop_yaw_error", stop_yaw_error_, 1.0);

  nh.param("use_gazebo_agents", use_gazebo_agents_, true);
  nh.param("robot_model_name", robot_model_name_, std::string(""));
  nh.param("other_model_prefix", other_model_prefix_, std::string(""));
  nh.param("include_humans", include_humans_, true);
  nh.param("gazebo_frame", gazebo_frame_, std::string("world"));
  nh.param("human_prefix", human_prefix_, std::string("human"));
  nh.param("human_radius", human_radius_, 0.45);

  nh.param("use_tracked_people", use_tracked_people_, false);
  nh.param("tracked_people_topic", tracked_people_topic_, std::string("/people"));
  nh.param("tracked_people_timeout", tracked_people_timeout_, 0.6);
  nh.param("tracked_person_radius", tracked_person_radius_, 0.42);
  nh.param("people_velocity_alpha", people_velocity_alpha_, 0.6);
  nh.param("min_people_speed_for_hrvo", min_people_speed_for_hrvo_, 0.05);

  nh.param("allow_world_as_global_when_tf_fails", allow_world_as_global_when_tf_fails_, true);
  nh.param("model_states_timeout", model_states_timeout_, 0.5);

  nh.param("use_costmap_safety", use_costmap_safety_, true);
  nh.param("obstacle_cost_threshold", obstacle_cost_threshold_, 253);
  nh.param("safety_rollout_time", safety_rollout_time_, 1.0);
  nh.param("safety_dt", safety_dt_, 0.10);
  nh.param("safety_radius_padding", safety_radius_padding_, 0.03);

  nh.param("clearance_probe_dist", clearance_probe_dist_, 0.80);
  nh.param("clearance_weight", clearance_weight_, 1.8);
  nh.param("obstacle_cost_weight", obstacle_cost_weight_, 1.2);
  nh.param("jerk_weight", jerk_weight_, 0.25);
  nh.param("hrvo_align_weight", hrvo_align_weight_, 2.5);
  nh.param("goal_align_weight", goal_align_weight_, 1.5);
  nh.param("speed_weight", speed_weight_, 0.8);
  nh.param("turn_weight", turn_weight_, 0.25);
  nh.param("spin_weight", spin_weight_, 0.05);

  last_time_ = ros::Time::now();
  last_cmd_ = geometry_msgs::Twist();

  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);

  if (use_gazebo_agents_) {
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 1,
                                     &HRVOLocalPlanner::modelStatesCb, this);
  }

  if (use_tracked_people_) {
    tracked_people_sub_ = nh.subscribe(tracked_people_topic_, 1,
                                       &HRVOLocalPlanner::trackedPeopleCb, this);
  }

  initialized_ = true;
  ROS_INFO("[HRVO] initialized.");
}

bool HRVOLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_) return false;

  global_plan_ = orig_global_plan;
  if (!global_plan_.empty()) {
    goal_ = global_plan_.back();
  }

  sim_initialized_ = false;
  other_agent_ids_.clear();
  step_count_ = 0;
  return true;
}

bool HRVOLocalPlanner::getRobotPose(geometry_msgs::PoseStamped& pose_out) const
{
  return costmap_ros_ && costmap_ros_->getRobotPose(pose_out);
}

geometry_msgs::PoseStamped HRVOLocalPlanner::getLookaheadTarget(
    const geometry_msgs::PoseStamped& robot_pose) const
{
  if (global_plan_.empty()) return robot_pose;
  if (global_plan_.size() == 1) return global_plan_.front();

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  std::size_t nearest_idx = 0;
  double best_d = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < global_plan_.size(); ++i) {
    const double dx = global_plan_[i].pose.position.x - rx;
    const double dy = global_plan_[i].pose.position.y - ry;
    const double d = dx * dx + dy * dy;
    if (d < best_d) {
      best_d = d;
      nearest_idx = i;
    }
  }

  for (std::size_t i = nearest_idx; i < global_plan_.size(); ++i) {
    const double dx = global_plan_[i].pose.position.x - rx;
    const double dy = global_plan_[i].pose.position.y - ry;
    if (std::hypot(dx, dy) >= lookahead_dist_) {
      return global_plan_[i];
    }
  }

  return global_plan_.back();
}

std::vector<HRVOLocalPlanner::OtherAgent> HRVOLocalPlanner::buildOtherAgentsFromGazebo() const
{
  std::vector<OtherAgent> out;
  if (!use_gazebo_agents_) return out;

  gazebo_msgs::ModelStates snap;
  ros::Time stamp;
  bool ok = false;

  {
    std::lock_guard<std::mutex> lk(g_model_states_mtx);
    if (have_model_states_) {
      snap = last_model_states_;
      stamp = last_model_states_stamp_;
      ok = true;
    }
  }

  if (!ok) return out;
  if ((ros::Time::now() - stamp).toSec() > model_states_timeout_) return out;

  const std::string global_frame = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : "map";
  const std::string gazebo_frame = gazebo_frame_.empty() ? "world" : gazebo_frame_;
  const std::string human_prefix = human_prefix_.empty() ? "human" : human_prefix_;

  const float human_r = static_cast<float>(std::max(0.05, human_radius_));
  const float other_r = static_cast<float>(std::max(0.05, robot_radius_));

  for (std::size_t i = 0; i < snap.name.size(); ++i) {
    const std::string& name = snap.name[i];

    if (!robot_model_name_.empty() && name == robot_model_name_) continue;
    if (name == "ground_plane" || name == "sun") continue;

    const bool is_human =
        (!human_prefix.empty() && name.rfind(human_prefix, 0) == 0) ||
        (name.find("human")  != std::string::npos) ||
        (name.find("person") != std::string::npos) ||
        (name.find("actor")  != std::string::npos) ||
        (name.find("ped")    != std::string::npos);

    if (is_human) {
      if (!include_humans_) continue;
    } else {
      if (other_model_prefix_.empty()) continue;
      if (name.rfind(other_model_prefix_, 0) != 0) continue;
    }

    geometry_msgs::Pose pose_src = snap.pose[i];
    geometry_msgs::Twist tw_src  = snap.twist[i];

    geometry_msgs::Pose pose_dst;
    geometry_msgs::Twist tw_dst;

    bool ok_pose = transformPose2D(tf_, gazebo_frame, global_frame, pose_src, pose_dst);
    bool ok_tw   = transformTwist2D(tf_, gazebo_frame, global_frame, tw_src, tw_dst);

    if ((!ok_pose || !ok_tw) && allow_world_as_global_when_tf_fails_) {
      pose_dst = pose_src;
      tw_dst   = tw_src;
      ok_pose = ok_tw = true;
    }

    if (!ok_pose || !ok_tw) continue;
    if (!isFinite(pose_dst.position.x) || !isFinite(pose_dst.position.y)) continue;
    if (!isFinite(tw_dst.linear.x) || !isFinite(tw_dst.linear.y)) continue;

    OtherAgent a;
    a.name = name;
    a.pos = hrvo::Vector2(static_cast<float>(pose_dst.position.x),
                          static_cast<float>(pose_dst.position.y));
    a.vel = hrvo::Vector2(static_cast<float>(tw_dst.linear.x),
                          static_cast<float>(tw_dst.linear.y));
    a.radius = is_human ? human_r : other_r;
    out.push_back(a);
  }

  return out;
}

std::vector<HRVOLocalPlanner::OtherAgent> HRVOLocalPlanner::buildOtherAgentsFromTrackedPeople() const
{
  std::vector<OtherAgent> out;
  if (!use_tracked_people_) return out;

  std::unordered_map<std::string, TrackedPersonState> snap;
  ros::Time stamp;

  {
    std::lock_guard<std::mutex> lk(g_people_mtx);
    snap = tracked_people_;
    stamp = last_people_stamp_;
  }

  if ((ros::Time::now() - stamp).toSec() > tracked_people_timeout_) return out;

  for (const auto& kv : snap) {
    const auto& id = kv.first;
    const auto& p  = kv.second;

    if (!isFinite(p.pose.position.x) || !isFinite(p.pose.position.y)) continue;

    OtherAgent a;
    a.name = id;
    a.pos = hrvo::Vector2(static_cast<float>(p.pose.position.x),
                          static_cast<float>(p.pose.position.y));
    a.vel = hrvo::Vector2(static_cast<float>(p.twist.linear.x),
                          static_cast<float>(p.twist.linear.y));
    a.radius = static_cast<float>(std::max(0.05, p.radius));
    out.push_back(a);
  }

  return out;
}

std::vector<HRVOLocalPlanner::OtherAgent> HRVOLocalPlanner::buildOtherAgents() const
{
  std::vector<OtherAgent> out;
  std::unordered_map<std::string, std::size_t> seen;

  auto push_unique = [&](const std::vector<OtherAgent>& src) {
    for (const auto& a : src) {
      if (seen.find(a.name) != seen.end()) continue;
      seen[a.name] = out.size();
      out.push_back(a);
    }
  };

  if (use_tracked_people_) push_unique(buildOtherAgentsFromTrackedPeople());
  if (use_gazebo_agents_)  push_unique(buildOtherAgentsFromGazebo());

  return out;
}

void HRVOLocalPlanner::resetSimulator(const geometry_msgs::PoseStamped& robot_pose,
                                      const geometry_msgs::PoseStamped& target)
{
  sim_ = std::make_unique<hrvo::Simulator>();
  sim_->setTimeStep(static_cast<float>(time_step_));

  const float rx  = static_cast<float>(robot_pose.pose.position.x);
  const float ry  = static_cast<float>(robot_pose.pose.position.y);
  const float yaw = static_cast<float>(poseYaw(robot_pose));

  const float tx = static_cast<float>(target.pose.position.x);
  const float ty = static_cast<float>(target.pose.position.y);

  last_goal_pos_ = hrvo::Vector2(tx, ty);
  robot_goal_id_ = sim_->addGoal(last_goal_pos_);

  const float rvx = static_cast<float>(last_cmd_.linear.x * std::cos(yaw));
  const float rvy = static_cast<float>(last_cmd_.linear.x * std::sin(yaw));

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
      hrvo::Vector2(rvx, rvy),
      yaw);

  other_agent_ids_.clear();
  sim_initialized_ = true;
  step_count_ = 0;
}

bool HRVOLocalPlanner::isStateSafe(double x, double y) const
{
  if (!costmap_) return true;

  const double r = robot_radius_ + safety_radius_padding_;
  constexpr int NUM = 16;

  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) return false;
    if (static_cast<int>(costmap_->getCost(mx, my)) >= obstacle_cost_threshold_) return false;
  }

  for (int i = 0; i < NUM; ++i) {
    const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(NUM);
    const double px = x + r * std::cos(ang);
    const double py = y + r * std::sin(ang);

    unsigned int mx, my;
    if (!costmap_->worldToMap(px, py, mx, my)) return false;
    if (static_cast<int>(costmap_->getCost(mx, my)) >= obstacle_cost_threshold_) return false;
  }

  return true;
}

bool HRVOLocalPlanner::isCmdSafe(const geometry_msgs::PoseStamped& robot_pose,
                                 const geometry_msgs::Twist& cmd) const
{
  if (!use_costmap_safety_ || !costmap_) return true;

  const double dt = std::max(1e-2, safety_dt_);
  const int N = static_cast<int>(std::ceil(std::max(0.1, safety_rollout_time_) / dt));

  double x = robot_pose.pose.position.x;
  double y = robot_pose.pose.position.y;
  double yaw = poseYaw(robot_pose);

  for (int i = 0; i < N; ++i) {
    x += cmd.linear.x * std::cos(yaw) * dt;
    y += cmd.linear.x * std::sin(yaw) * dt;
    yaw += cmd.angular.z * dt;

    if (!isStateSafe(x, y)) return false;
  }

  return true;
}

double HRVOLocalPlanner::pointCostNorm(double x, double y) const
{
  if (!costmap_) return 0.0;

  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return 1.0;

  const unsigned char c = costmap_->getCost(mx, my);
  if (c >= static_cast<unsigned char>(obstacle_cost_threshold_)) return 1.0;
  return static_cast<double>(c) / 252.0;
}

double HRVOLocalPlanner::estimateObstacleDistance(double x, double y, double max_probe) const
{
  if (!costmap_) return max_probe;

  const double r0 = robot_radius_ + safety_radius_padding_;
  const double dr = 0.05;
  const int nang = 24;

  for (double probe = r0; probe <= max_probe; probe += dr) {
    for (int i = 0; i < nang; ++i) {
      const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(nang);
      const double px = x + probe * std::cos(ang);
      const double py = y + probe * std::sin(ang);

      unsigned int mx, my;
      if (!costmap_->worldToMap(px, py, mx, my)) {
        return probe;
      }

      if (static_cast<int>(costmap_->getCost(mx, my)) >= obstacle_cost_threshold_) {
        return probe;
      }
    }
  }

  return max_probe;
}

HRVOLocalPlanner::RolloutEval
HRVOLocalPlanner::evaluateRollout(const geometry_msgs::PoseStamped& robot_pose,
                                  double v_cmd, double w_cmd, double horizon) const
{
  RolloutEval ev;
  if (!costmap_ || !use_costmap_safety_) {
    ev.safe = true;
    ev.min_clearance = clearance_probe_dist_;
    return ev;
  }

  const double dt = std::max(1e-2, safety_dt_);
  const int N = static_cast<int>(std::ceil(std::max(0.1, horizon) / dt));

  double x = robot_pose.pose.position.x;
  double y = robot_pose.pose.position.y;
  double yaw = poseYaw(robot_pose);

  double cost_sum = 0.0;
  double cost_max = 0.0;
  double clear_min = clearance_probe_dist_;

  for (int i = 0; i < N; ++i) {
    x += v_cmd * std::cos(yaw) * dt;
    y += v_cmd * std::sin(yaw) * dt;
    yaw += w_cmd * dt;

    if (!isStateSafe(x, y)) {
      ev.safe = false;
      ev.avg_cost_norm = 1.0;
      ev.max_cost_norm = 1.0;
      ev.min_clearance = 0.0;
      ev.x_end = x;
      ev.y_end = y;
      ev.yaw_end = yaw;
      return ev;
    }

    const double c = pointCostNorm(x, y);
    cost_sum += c;
    cost_max = std::max(cost_max, c);
    clear_min = std::min(clear_min, estimateObstacleDistance(x, y, clearance_probe_dist_));
  }

  ev.safe = true;
  ev.avg_cost_norm = (N > 0) ? cost_sum / static_cast<double>(N) : 0.0;
  ev.max_cost_norm = cost_max;
  ev.min_clearance = clear_min;
  ev.x_end = x;
  ev.y_end = y;
  ev.yaw_end = yaw;
  return ev;
}

geometry_msgs::Twist HRVOLocalPlanner::fallbackCmd(const geometry_msgs::PoseStamped& robot_pose,
                                                   const geometry_msgs::PoseStamped& target) const
{
  const double yaw = poseYaw(robot_pose);
  const double dir = std::atan2(target.pose.position.y - robot_pose.pose.position.y,
                                target.pose.position.x - robot_pose.pose.position.x);
  const double dyaw = angles::shortest_angular_distance(yaw, dir);

  geometry_msgs::Twist rot;
  rot.linear.x = 0.0;
  rot.angular.z = clampd(heading_kp_ * dyaw, -max_vel_theta_, max_vel_theta_);
  if (isCmdSafe(robot_pose, rot)) return rot;

  geometry_msgs::Twist creep;
  creep.linear.x = std::min(0.08, max_vel_x_);
  creep.angular.z = clampd(0.5 * heading_kp_ * dyaw,
                           -0.5 * max_vel_theta_, 0.5 * max_vel_theta_);
  if (isCmdSafe(robot_pose, creep)) return creep;

  return geometry_msgs::Twist();
}

void HRVOLocalPlanner::publishLocalPlanRollout(const geometry_msgs::PoseStamped& robot_pose,
                                               double vx, double vy) const
{
  if (!local_plan_pub_ || !costmap_ros_) return;

  nav_msgs::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped p0, p1;
  p0.header = path.header;
  p0.pose = robot_pose.pose;

  const double vnorm = std::hypot(vx, vy);
  const double scale = std::max(1.0, std::min(2.5, 1.5 + vnorm * 2.0));

  p1.header = path.header;
  p1.pose.position.x = robot_pose.pose.position.x + scale * vx;
  p1.pose.position.y = robot_pose.pose.position.y + scale * vy;
  p1.pose.position.z = robot_pose.pose.position.z;

  const double yaw = (vnorm > 1e-6) ? std::atan2(vy, vx) : poseYaw(robot_pose);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  p1.pose.orientation = tf2::toMsg(q);

  path.poses.push_back(p0);
  path.poses.push_back(p1);
  local_plan_pub_.publish(path);
}

bool HRVOLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_ || global_plan_.empty()) return false;

  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) return false;

  const double yaw = poseYaw(robot_pose);
  const double d_goal = dist2DPoses(robot_pose, goal_);
  const double goal_yaw = poseYaw(goal_);
  const double dyaw_goal = angles::shortest_angular_distance(yaw, goal_yaw);

  if (d_goal <= xy_goal_tol_) {
    geometry_msgs::Twist out;
    if (std::fabs(dyaw_goal) <= yaw_goal_tol_) {
      out.linear.x = 0.0;
      out.angular.z = 0.0;
    } else {
      out.linear.x = 0.0;
      out.angular.z = clampd(heading_kp_ * dyaw_goal, -max_vel_theta_, max_vel_theta_);
    }

    cmd_vel = out;
    last_cmd_ = cmd_vel;
    last_time_ = ros::Time::now();
    publishLocalPlanRollout(robot_pose, 0.0, 0.0);
    return true;
  }

  const geometry_msgs::PoseStamped target = getLookaheadTarget(robot_pose);

  const hrvo::Vector2 new_goal(static_cast<float>(target.pose.position.x),
                               static_cast<float>(target.pose.position.y));

  bool require_reset = false;
  if (!sim_initialized_) require_reset = true;

  const float gdx = new_goal.getX() - last_goal_pos_.getX();
  const float gdy = new_goal.getY() - last_goal_pos_.getY();
  if (std::hypot(gdx, gdy) > static_cast<float>(goal_update_dist_)) {
    require_reset = true;
  }

  if (goal_reset_every_ > 0 && step_count_ >= goal_reset_every_) {
    require_reset = true;
  }

  const auto others = buildOtherAgents();
  std::unordered_set<std::string> seen;
  seen.reserve(others.size());

  for (const auto& a : others) seen.insert(a.name);

  if (sim_initialized_) {
    for (const auto& kv : other_agent_ids_) {
      if (seen.find(kv.first) == seen.end()) {
        require_reset = true;
        break;
      }
    }
    for (const auto& a : others) {
      if (other_agent_ids_.find(a.name) == other_agent_ids_.end()) {
        require_reset = true;
        break;
      }
    }
  }

  if (require_reset) {
    resetSimulator(robot_pose, target);

    for (const auto& a : others) {
      const double avx = static_cast<double>(a.vel.getX());
      const double avy = static_cast<double>(a.vel.getY());
      const double aspeed = std::hypot(avx, avy);

      hrvo::Vector2 shadow_goal = a.pos;
      float shadow_pref_speed = 0.0f;
      float shadow_yaw = 0.0f;

      if (aspeed > min_people_speed_for_hrvo_) {
        const double proj_dist = std::max(0.8, std::min(3.0, aspeed * 2.0));
        const double ux = avx / aspeed;
        const double uy = avy / aspeed;

        shadow_goal = hrvo::Vector2(
            static_cast<float>(a.pos.getX() + proj_dist * ux),
            static_cast<float>(a.pos.getY() + proj_dist * uy));

        shadow_pref_speed = static_cast<float>(std::min(aspeed, max_speed_));
        shadow_yaw = static_cast<float>(std::atan2(avy, avx));
      }

      const std::size_t g = sim_->addGoal(shadow_goal);
      const std::size_t id = sim_->addAgent(
          a.pos,
          g,
          static_cast<float>(neighbor_dist_),
          static_cast<std::size_t>(std::max(1, max_neighbors_)),
          static_cast<float>(a.radius),
          static_cast<float>(goal_radius_),
          shadow_pref_speed,
          static_cast<float>(max_speed_),
          static_cast<float>(uncertainty_offset_),
          static_cast<float>(max_accel_),
          a.vel,
          shadow_yaw);

      other_agent_ids_[a.name] = id;
      sim_->setAgentPosition(id, a.pos);
      sim_->setAgentVelocity(id, a.vel);
      sim_->setAgentOrientation(id, shadow_yaw);
    }
  } else {
    sim_->setAgentPosition(robot_agent_id_,
                           hrvo::Vector2(static_cast<float>(robot_pose.pose.position.x),
                                         static_cast<float>(robot_pose.pose.position.y)));
    sim_->setAgentOrientation(robot_agent_id_, static_cast<float>(yaw));

    const float rvx = static_cast<float>(last_cmd_.linear.x * std::cos(yaw));
    const float rvy = static_cast<float>(last_cmd_.linear.x * std::sin(yaw));
    sim_->setAgentVelocity(robot_agent_id_, hrvo::Vector2(rvx, rvy));

    for (const auto& a : others) {
      auto it = other_agent_ids_.find(a.name);
      if (it == other_agent_ids_.end()) continue;

      const std::size_t id = it->second;
      sim_->setAgentPosition(id, a.pos);
      sim_->setAgentVelocity(id, a.vel);

      const double avx = static_cast<double>(a.vel.getX());
      const double avy = static_cast<double>(a.vel.getY());
      if (std::hypot(avx, avy) > min_people_speed_for_hrvo_) {
        sim_->setAgentOrientation(id, static_cast<float>(std::atan2(avy, avx)));
      }
    }
  }

  sim_->setAgentPosition(robot_agent_id_,
                         hrvo::Vector2(static_cast<float>(robot_pose.pose.position.x),
                                       static_cast<float>(robot_pose.pose.position.y)));
  sim_->setAgentOrientation(robot_agent_id_, static_cast<float>(yaw));
  {
    const float rvx = static_cast<float>(last_cmd_.linear.x * std::cos(yaw));
    const float rvy = static_cast<float>(last_cmd_.linear.x * std::sin(yaw));
    sim_->setAgentVelocity(robot_agent_id_, hrvo::Vector2(rvx, rvy));
  }

  sim_->doStep();
  ++step_count_;

  const hrvo::Vector2 v_hrvo = sim_->getAgentVelocity(robot_agent_id_);
  const double vx_hrvo = static_cast<double>(v_hrvo.getX());
  const double vy_hrvo = static_cast<double>(v_hrvo.getY());
  const double vnorm_hrvo = std::hypot(vx_hrvo, vy_hrvo);

  const double tx = target.pose.position.x;
  const double ty = target.pose.position.y;
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  const double desired_dir =
      (vnorm_hrvo >= min_vnorm_dir_) ? std::atan2(vy_hrvo, vx_hrvo)
                                     : std::atan2(ty - ry, tx - rx);

  double desired_speed = std::min(vnorm_hrvo, max_vel_x_);
  const double heading_err = std::fabs(angles::shortest_angular_distance(yaw, desired_dir));

  if (slowdown_cos_) {
    desired_speed *= std::max(0.0, std::cos(std::min(heading_err, M_PI_2)));
  }

  const double desired_w = clampd(heading_kp_ *
                                  angles::shortest_angular_distance(yaw, desired_dir),
                                  -max_vel_theta_, max_vel_theta_);

  geometry_msgs::Twist best_cmd;
  double best_score = -std::numeric_limits<double>::infinity();
  bool found_safe = false;

  auto scoreCandidate = [&](double v_cmd, double w_cmd) -> double
  {
    const double H = std::max(0.4, std::min(1.0, safety_rollout_time_));
    const RolloutEval ev = evaluateRollout(robot_pose, v_cmd, w_cmd, H);
    if (!ev.safe) return -1e9;

    const double yaw_mid = yaw + 0.5 * w_cmd * H;
    const double hrvo_align = std::cos(angles::shortest_angular_distance(yaw_mid, desired_dir));

    const double to_goal_dir = std::atan2(ty - ev.y_end, tx - ev.x_end);
    const double goal_align = std::cos(angles::shortest_angular_distance(ev.yaw_end, to_goal_dir));

    const double speed_term = -std::fabs(v_cmd - desired_speed);
    const double turn_term  = -std::fabs(w_cmd - desired_w);
    const double spin_pen   = -std::fabs(w_cmd);
    const double jerk_pen   = -std::fabs(v_cmd - last_cmd_.linear.x)
                              - 0.3 * std::fabs(w_cmd - last_cmd_.angular.z);

    const double clearance_norm =
        clampd(ev.min_clearance / std::max(0.15, clearance_probe_dist_), 0.0, 1.0);

    const double obstacle_pen = -(0.7 * ev.avg_cost_norm + 0.3 * ev.max_cost_norm);

    double near_goal_bonus = 0.0;
    if (d_goal < 1.0) {
      near_goal_bonus = -0.2 * std::fabs(w_cmd);
    }

    return hrvo_align_weight_ * hrvo_align +
           goal_align_weight_ * goal_align +
           speed_weight_ * speed_term -
           turn_weight_ * std::fabs(turn_term) -
           spin_weight_ * std::fabs(spin_pen) +
           clearance_weight_ * clearance_norm +
           obstacle_cost_weight_ * obstacle_pen +
           jerk_weight_ * jerk_pen +
           near_goal_bonus;
  };

  std::vector<double> v_samples;
  std::vector<double> w_samples;

  v_samples.push_back(0.0);
  v_samples.push_back(std::min(max_vel_x_, desired_speed));
  v_samples.push_back(std::min(max_vel_x_, last_cmd_.linear.x));
  for (int i = 1; i <= 8; ++i) {
    v_samples.push_back(max_vel_x_ * static_cast<double>(i) / 8.0);
  }

  w_samples.push_back(desired_w);
  w_samples.push_back(last_cmd_.angular.z);
  for (int i = -8; i <= 8; ++i) {
    w_samples.push_back(max_vel_theta_ * static_cast<double>(i) / 8.0);
  }

  for (double v_cmd : v_samples) {
    v_cmd = clampd(v_cmd, 0.0, max_vel_x_);

    for (double w_cmd : w_samples) {
      w_cmd = clampd(w_cmd, -max_vel_theta_, max_vel_theta_);

      geometry_msgs::Twist cand;
      cand.linear.x = v_cmd;
      cand.angular.z = w_cmd;

      if (d_goal < 1.0) {
        const double scale = clampd(d_goal / 1.0, 0.20, 1.0);
        cand.linear.x *= scale;
      }

      if (std::fabs(angles::shortest_angular_distance(yaw, desired_dir)) > stop_yaw_error_) {
        cand.linear.x = std::min(cand.linear.x, 0.06);
      }

      if (!isCmdSafe(robot_pose, cand)) continue;

      const double score = scoreCandidate(cand.linear.x, cand.angular.z);
      if (score > best_score) {
        best_score = score;
        best_cmd = cand;
        found_safe = true;
      }
    }
  }

  geometry_msgs::Twist out;
  if (found_safe) {
    out = best_cmd;
  } else {
    out = fallbackCmd(robot_pose, target);
  }

  const ros::Time now = ros::Time::now();
  const double dt = std::max(1e-3, (now - last_time_).toSec());
  last_time_ = now;

  const double dv_max = acc_lim_x_ * dt;
  const double dw_max = acc_lim_theta_ * dt;

  out.linear.x = clampd(out.linear.x,
                        last_cmd_.linear.x - dv_max,
                        last_cmd_.linear.x + dv_max);

  out.angular.z = clampd(out.angular.z,
                         last_cmd_.angular.z - dw_max,
                         last_cmd_.angular.z + dw_max);

  out.linear.x = clampd(out.linear.x, 0.0, max_vel_x_);
  out.angular.z = clampd(out.angular.z, -max_vel_theta_, max_vel_theta_);

  if (!isCmdSafe(robot_pose, out)) {
    out = fallbackCmd(robot_pose, target);
  }

  cmd_vel = out;
  last_cmd_ = cmd_vel;

  publishLocalPlanRollout(robot_pose, vx_hrvo, vy_hrvo);
  return true;
}

bool HRVOLocalPlanner::isGoalReached()
{
  if (!initialized_ || global_plan_.empty()) return false;

  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) return false;

  const double d = dist2DPoses(robot_pose, goal_);
  const double yaw = poseYaw(robot_pose);
  const double goal_yaw = poseYaw(goal_);
  const double dyaw = angles::shortest_angular_distance(yaw, goal_yaw);

  return (d <= xy_goal_tol_) && (std::fabs(dyaw) <= yaw_goal_tol_);
}

} // namespace hrvo_local_planner

PLUGINLIB_EXPORT_CLASS(hrvo_local_planner::HRVOLocalPlanner, nav_core::BaseLocalPlanner)