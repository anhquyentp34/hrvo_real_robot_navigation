#include <hrvo/hrvo_local_planner.h>

#include <pluginlib/class_list_macros.h>
#include <memory>
#include <mutex>
#include <unordered_set>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hrvo_local_planner {

static inline bool isFinite(double x) {
  return std::isfinite(x);
}

HRVOLocalPlanner::HRVOLocalPlanner()
{
  last_time_ = ros::Time::now();
  last_cmd_ = geometry_msgs::Twist();
  sim_initialized_ = false;
}

// -------------------- Gazebo model states (thread-safe cache) --------------------
namespace {
std::mutex g_model_states_mtx;
}

void HRVOLocalPlanner::modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(g_model_states_mtx);
  last_model_states_ = *msg;
  have_model_states_ = true;
  last_model_states_stamp_ = ros::Time::now();  // model_states has no header
}

// Transform helper: gazebo/world -> global_frame (2D)
static inline bool transformPose2D(tf2_ros::Buffer* tf,
                                  const std::string& from_frame,
                                  const std::string& to_frame,
                                  const geometry_msgs::Pose& in_pose,
                                  geometry_msgs::Pose& out_pose)
{
  if (!tf || from_frame == to_frame) {
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
  } catch (...) {
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
  if (!tf || from_frame == to_frame) return true;

  // TwistStamped transform is not always available; do a planar rotate using TF yaw only
  try {
    geometry_msgs::TransformStamped T =
        tf->lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(0.05));

    const double yaw = tf2::getYaw(T.transform.rotation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    // rotate linear velocity
    const double vx = in_tw.linear.x;
    const double vy = in_tw.linear.y;
    out_tw.linear.x = c * vx - s * vy;
    out_tw.linear.y = s * vx + c * vy;

    // angular z is same in planar if frames share z axis
    out_tw.angular.z = in_tw.angular.z;
    return true;
  } catch (...) {
    return false;
  }
}

std::vector<HRVOLocalPlanner::OtherAgent> HRVOLocalPlanner::buildOtherAgents() const
{
  std::vector<OtherAgent> out;
  if (!use_gazebo_agents_) return out;

  gazebo_msgs::ModelStates snap;
  bool ok = false;

  {
    std::lock_guard<std::mutex> lk(g_model_states_mtx);
    if (have_model_states_) {
      snap = last_model_states_;
      ok = true;
    }
  }
  if (!ok) return out;

  const std::string global_frame = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : "map";
  // You can set these as ROS params if needed; keep simple defaults.
  const std::string gazebo_frame = "world";  // common in Gazebo + ROS

  for (std::size_t i = 0; i < snap.name.size(); ++i) {
    const std::string& name = snap.name[i];
    if (!robot_model_name_.empty() && name == robot_model_name_) continue;

    if (!other_model_prefix_.empty()) {
      if (name.rfind(other_model_prefix_, 0) != 0) continue;  // prefix mismatch
    }

    // Optional: filter humans if you want
    if (!include_humans_) {
      // heuristic: ignore models containing "human" if not included
      if (name.find("human") != std::string::npos || name.find("person") != std::string::npos)
        continue;
    }

    geometry_msgs::Pose pose_g = snap.pose[i];
    geometry_msgs::Twist tw_g  = snap.twist[i];

    geometry_msgs::Pose pose;
    geometry_msgs::Twist tw;
    if (!transformPose2D(tf_, gazebo_frame, global_frame, pose_g, pose)) continue;
    if (!transformTwist2D(tf_, gazebo_frame, global_frame, tw_g, tw)) continue;

    if (!isFinite(pose.position.x) || !isFinite(pose.position.y)) continue;

    OtherAgent a;
    a.name = name;
    a.pos = hrvo::Vector2(static_cast<float>(pose.position.x),
                          static_cast<float>(pose.position.y));
    a.vel = hrvo::Vector2(static_cast<float>(tw.linear.x),
                          static_cast<float>(tw.linear.y));

    // Radius per-agent: you can also make a map param later; keep one radius for now
    a.radius = static_cast<float>(robot_radius_);
    out.push_back(a);
  }

  return out;
}

// -------------------- ROS / move_base --------------------
void HRVOLocalPlanner::initialize(std::string name,
                                 tf2_ros::Buffer* tf,
                                 costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) return;

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;

  ros::NodeHandle nh("~/" + name);

  // move_base style params
  nh.param("max_vel_x", max_vel_x_, 0.8);
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

  // gazebo agent params
  nh.param("use_gazebo_agents", use_gazebo_agents_, true);
  nh.param("robot_model_name", robot_model_name_, std::string(""));
  nh.param("other_model_prefix", other_model_prefix_, std::string(""));
  nh.param("include_humans", include_humans_, false);

  last_time_ = ros::Time::now();
  last_cmd_ = geometry_msgs::Twist();

  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);

  if (use_gazebo_agents_) {
    // Note: you can remap this topic in launch
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 1, &HRVOLocalPlanner::modelStatesCb, this);
  }

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

// -------------------- HRVO sim management --------------------
void HRVOLocalPlanner::resetSimulator(const geometry_msgs::PoseStamped& robot_pose,
                                      const geometry_msgs::PoseStamped& target)
{
  sim_ = std::make_unique<hrvo::Simulator>();
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

  other_agent_ids_.clear();
  sim_initialized_ = true;
  step_count_ = 0;
}

void HRVOLocalPlanner::publishLocalPlanRollout(const geometry_msgs::PoseStamped& robot_pose,
                                              const geometry_msgs::Twist& cmd_vel) const
{
  if (!local_plan_pub_) return;
  if (!costmap_ros_) return;

  nav_msgs::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = ros::Time::now();

  // forward integrate for ~2 seconds
  const double dt = std::max(1e-2, time_step_);
  const int N = static_cast<int>(std::ceil(2.0 / dt));

  double x = robot_pose.pose.position.x;
  double y = robot_pose.pose.position.y;
  double yaw = getYaw(robot_pose);

  geometry_msgs::PoseStamped p;
  p.header = path.header;
  p.pose = robot_pose.pose;
  path.poses.push_back(p);

  for (int i = 0; i < N; ++i) {
    yaw += cmd_vel.angular.z * dt;
    x += cmd_vel.linear.x * std::cos(yaw) * dt;
    y += cmd_vel.linear.x * std::sin(yaw) * dt;

    geometry_msgs::PoseStamped pi;
    pi.header = path.header;
    pi.pose.position.x = x;
    pi.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pi.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pi);
  }

  local_plan_pub_.publish(path);
}

// -------------------- Core loop --------------------
bool HRVOLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_ || global_plan_.empty()) return false;

  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) return false;

  const double d_goal = dist2D(robot_pose, goal_);
  const double yaw = getYaw(robot_pose);
  const double goal_yaw = getYaw(goal_);
  const double dyaw_goal = angles::shortest_angular_distance(yaw, goal_yaw);

  // Goal reached: align yaw
  if (d_goal <= xy_goal_tol_) {
    geometry_msgs::Twist out;
    if (std::fabs(dyaw_goal) <= yaw_goal_tol_) {
      cmd_vel = geometry_msgs::Twist();
      last_cmd_ = cmd_vel;
      publishLocalPlanRollout(robot_pose, cmd_vel);
      return true;
    }
    out.linear.x = 0.0;
    out.angular.z = clamp(dyaw_goal, -max_vel_theta_, max_vel_theta_);
    cmd_vel = out;
    last_cmd_ = cmd_vel;
    publishLocalPlanRollout(robot_pose, cmd_vel);
    return true;
  }

  // Pick local target
  const geometry_msgs::PoseStamped target = getLookaheadTarget(robot_pose);

  // Init/reset sim
  if (!sim_initialized_) {
    resetSimulator(robot_pose, target);
  }
  if (goal_reset_every_ > 0 && step_count_ >= goal_reset_every_) {
    resetSimulator(robot_pose, target);
  }

  // Update robot state in sim
  sim_->setAgentPosition(robot_agent_id_,
                         hrvo::Vector2(static_cast<float>(robot_pose.pose.position.x),
                                       static_cast<float>(robot_pose.pose.position.y)));
  sim_->setAgentOrientation(robot_agent_id_, static_cast<float>(yaw));

  // Update goal (library lacks "setGoalPosition", so we add new when moved enough)
  const hrvo::Vector2 new_goal(static_cast<float>(target.pose.position.x),
                               static_cast<float>(target.pose.position.y));

  const float gdx = new_goal.getX() - last_goal_pos_.getX();
  const float gdy = new_goal.getY() - last_goal_pos_.getY();
  const float gdist = std::hypot(gdx, gdy);

  if (gdist > 0.15f) {
    last_goal_pos_ = new_goal;
    robot_goal_id_ = sim_->addGoal(last_goal_pos_);
    sim_->setAgentGoal(robot_agent_id_, robot_goal_id_);
  }

  // ---- Add/update neighbor agents (Gazebo) ----
  const auto others = buildOtherAgents();
  std::unordered_set<std::string> seen;
  seen.reserve(others.size());

  for (const auto& a : others) {
    seen.insert(a.name);

    auto it = other_agent_ids_.find(a.name);
    if (it == other_agent_ids_.end()) {
      // Add new agent with NO goal (or self goal). Strategy: create a "dummy goal" at its current pos.
      const std::size_t g = sim_->addGoal(a.pos);
      const std::size_t id = sim_->addAgent(
        a.pos,
        g,
        static_cast<float>(neighbor_dist_),
        static_cast<std::size_t>(std::max(1, max_neighbors_)),
        static_cast<float>(a.radius),
        static_cast<float>(goal_radius_),
        static_cast<float>(pref_speed_),   // not too important for neighbors if we set velocity
        static_cast<float>(max_speed_),
        static_cast<float>(uncertainty_offset_),
        static_cast<float>(max_accel_),
        a.vel,
        0.0f
      );
      other_agent_ids_[a.name] = id;
      it = other_agent_ids_.find(a.name);
    }

    const std::size_t id = it->second;
    sim_->setAgentPosition(id, a.pos);
    sim_->setAgentVelocity(id, a.vel);

    // keep its goal at its current pos to avoid it "seeking" somewhere else inside sim
    // (acts like a moving agent with measured velocity)
    // If your HRVO lib allows setGoalPosition, use it; otherwise: periodically reset sim to avoid goal growth.
  }

  // Remove disappeared agents (best effort, depending on HRVO library support)
  // If hrvo::Simulator has no removeAgent API, we can't truly delete; we can "park" them far away.
  for (auto it = other_agent_ids_.begin(); it != other_agent_ids_.end(); ) {
    if (seen.find(it->first) == seen.end()) {
      const std::size_t id = it->second;
      // park it far away + zero vel
      sim_->setAgentPosition(id, hrvo::Vector2(1e6f, 1e6f));
      sim_->setAgentVelocity(id, hrvo::Vector2(0.f, 0.f));
      it = other_agent_ids_.erase(it);
    } else {
      ++it;
    }
  }

  // HRVO step
  sim_->doStep();
  ++step_count_;

  // HRVO output: holonomic velocity (vx, vy)
  const hrvo::Vector2 v = sim_->getAgentVelocity(robot_agent_id_);
  const double vx = static_cast<double>(v.getX());
  const double vy = static_cast<double>(v.getY());

  // Map to diff-drive (v, w)
  const double vnorm = std::hypot(vx, vy);
  const double vdir  = std::atan2(vy, vx);
  const double dyaw  = angles::shortest_angular_distance(yaw, vdir);

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

  // Accel clamp (move_base style)
  const ros::Time now = ros::Time::now();
  const double dt = std::max(1e-3, (now - last_time_).toSec());
  last_time_ = now;

  const double dv_max = acc_lim_x_ * dt;
  const double dw_max = acc_lim_theta_ * dt;

  out.linear.x  = clamp(out.linear.x,  last_cmd_.linear.x  - dv_max, last_cmd_.linear.x  + dv_max);
  out.angular.z = clamp(out.angular.z, last_cmd_.angular.z - dw_max, last_cmd_.angular.z + dw_max);

  // Final clamp
  out.linear.x  = clamp(out.linear.x,  -max_vel_x_,     max_vel_x_);
  out.angular.z = clamp(out.angular.z, -max_vel_theta_, max_vel_theta_);

  cmd_vel = out;
  last_cmd_ = cmd_vel;

  publishLocalPlanRollout(robot_pose, cmd_vel);
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