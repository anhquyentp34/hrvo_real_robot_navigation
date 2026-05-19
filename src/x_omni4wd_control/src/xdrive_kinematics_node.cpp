#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

class XDriveKinematicsNode
{
public:
  XDriveKinematicsNode() : nh_("~"), last_cmd_time_(0.0), cmd_timeout_(0.2)
  {
    // Parameters
    nh_.param("wheel_radius", wheel_radius_, 0.0635);
    nh_.param("L_HALF", L_HALF_, 0.176776695); // half length (x)
    nh_.param("W_HALF", W_HALF_, 0.176776695); // half width (y)
    k_ = L_HALF_ + W_HALF_;                    // rotation factor
    nh_.param("odom_frame", odom_frame_, std::string("odom"));
    nh_.param("base_frame", base_frame_, std::string("base_link"));
    nh_.param("publish_tf", publish_tf_, true);
    nh_.param("cmd_timeout", cmd_timeout_, 0.2);

    // Robot namespace
    std::string robot_ns;
    nh_.param("robot_ns", robot_ns, std::string("/x_omni4wd"));
    if (robot_ns[0] != '/')
      robot_ns = "/" + robot_ns;

    // Wheel positions (x, y) in base_link frame
    // FL: (+0.13601, +0.13601), RL: (-0.13601, +0.13601)
    // RR: (-0.13601, -0.13601), FR: (+0.13601, -0.13601)
    wheel_positions_.resize(4);
    wheel_positions_[0] = std::make_pair(0.13601, 0.13601);   // FL
    wheel_positions_[1] = std::make_pair(-0.13601, 0.13601);  // RL
    wheel_positions_[2] = std::make_pair(-0.13601, -0.13601); // RR
    wheel_positions_[3] = std::make_pair(0.13601, -0.13601);  // FR

    // Wheel alpha angles (effective rolling direction) from joint rpy yaw
    // FL: -45°, RL: +45°, RR: -45°, FR: +45°
    wheel_alpha_.resize(4);
    wheel_alpha_[0] = -M_PI / 4.0; // FL: -45°
    wheel_alpha_[1] = M_PI / 4.0;  // RL: +45°
    wheel_alpha_[2] = -M_PI / 4.0; // RR: -45°
    wheel_alpha_[3] = M_PI / 4.0;  // FR: +45°

    // Sign calibration per wheel (default +1, can be overridden by params)
    sign_per_wheel_.resize(4, 1.0);
    nh_.param("sign_fl", sign_per_wheel_[0], 1.0);
    nh_.param("sign_rl", sign_per_wheel_[1], 1.0);
    nh_.param("sign_rr", sign_per_wheel_[2], 1.0);
    nh_.param("sign_fr", sign_per_wheel_[3], 1.0);

    // Joint names
    joint_names_.resize(4);
    joint_names_[0] = "base_link_front_left_wheel_joint";
    joint_names_[1] = "base_link_rear_left_wheel_joint";
    joint_names_[2] = "base_link_rear_right_wheel_joint";
    joint_names_[3] = "base_link_front_right_wheel_joint";

    // Publisher for forward_velocity_controller (using Float64MultiArray like omni_navigation)
    // Note: JointGroupVelocityController uses /command topic (not /commands)
    wheel_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(robot_ns + "/forward_velocity_controller/command", 10);

    // Subscribers
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &XDriveKinematicsNode::cmdVelCallback, this);
    joint_states_sub_ = nh_.subscribe(robot_ns + "/joint_states", 10, &XDriveKinematicsNode::jointStatesCallback, this);

    // Publishers
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

    // TF broadcaster
    if (publish_tf_)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    }

    // Initialize odometry
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    last_time_ = ros::Time::now();

    ROS_INFO("X-Drive Kinematics Node initialized");
    ROS_INFO("  Wheel radius: %.3f m", wheel_radius_);
    ROS_INFO("  Signs: FL=%.1f, RL=%.1f, RR=%.1f, FR=%.1f",
             sign_per_wheel_[0], sign_per_wheel_[1], sign_per_wheel_[2], sign_per_wheel_[3]);
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    last_cmd_time_ = ros::Time::now().toSec();

    // Quy ước REP-103 cho cmd_vel:
    // linear.x > 0  => robot tiến ( +X )
    // linear.y > 0  => robot sang trái ( +Y )
    // angular.z > 0 => quay CCW (dương)
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;

    // Inverse kinematics X-drive (Jacobian giống script cmd_vel_to_omni_wheels.py)
    // Dạng chuẩn đối xứng 4 bánh:
    //   w_FL = (-vx + vy + k*omega) / r
    //   w_FR = ( vx + vy + k*omega) / r
    //   w_RL = (-vx - vy + k*omega) / r
    //   w_RR = ( vx - vy + k*omega) / r
    std::vector<double> wheel_velocities(4);

    double w_fl = (-vx + vy + k_ * omega) / wheel_radius_;
    double w_fr = (vx + vy + k_ * omega) / wheel_radius_;
    double w_rl = (-vx - vy + k_ * omega) / wheel_radius_;
    double w_rr = (vx - vy + k_ * omega) / wheel_radius_;

    // Áp dụng hệ số hiệu chỉnh dấu từng bánh (nếu cần)
    wheel_velocities[0] = sign_per_wheel_[0] * w_fl; // FL
    wheel_velocities[1] = sign_per_wheel_[1] * w_rl; // RL
    wheel_velocities[2] = sign_per_wheel_[2] * w_rr; // RR
    wheel_velocities[3] = sign_per_wheel_[3] * w_fr; // FR

    // Publish wheel velocity commands using Float64MultiArray (like omni_navigation)
    std_msgs::Float64MultiArray cmd_array;
    cmd_array.data.resize(4);
    for (int i = 0; i < 4; i++)
    {
      cmd_array.data[i] = wheel_velocities[i];
    }
    wheel_cmd_pub_.publish(cmd_array);
  }

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    // Extract wheel velocities from joint_states
    std::vector<double> wheel_velocities(4, 0.0);
    bool found_all = true;

    for (int i = 0; i < 4; i++)
    {
      bool found = false;
      for (size_t j = 0; j < msg->name.size(); j++)
      {
        if (msg->name[j] == joint_names_[i])
        {
          if (j < msg->velocity.size())
          {
            wheel_velocities[i] = msg->velocity[j];
            found = true;
            break;
          }
        }
      }
      if (!found)
        found_all = false;
    }

    if (!found_all)
    {
      ROS_WARN_THROTTLE(1.0, "Not all wheel joints found in joint_states");
      return;
    }

    // Compute forward kinematics: wheel velocities -> robot velocity
    // Using inverse of the inverse kinematics
    // For X-drive: we need to solve the system
    // This is a simplified approach - in practice, you'd use the inverse matrix

    // Forward kinematics: wheel velocities -> robot velocity
    // Solve the inverse of: wheel_vel_i = (1/r) * d_i^T * (v_robot + ω * [-y_i, x_i]^T)
    // This is an overdetermined system (4 equations, 3 unknowns)
    // Use least squares solution

    // Build the system: A * [vx, vy, omega]^T = b
    // where A is 4x3 matrix and b is wheel velocities * r
    Eigen::MatrixXd A(4, 3);
    Eigen::VectorXd b(4);

    for (int i = 0; i < 4; i++)
    {
      double dx = std::cos(wheel_alpha_[i]);
      double dy = std::sin(wheel_alpha_[i]);
      double xi = wheel_positions_[i].first;
      double yi = wheel_positions_[i].second;

      // A[i] = [d_i^T, d_i^T * [-y_i, x_i]^T]
      A(i, 0) = dx;                 // coefficient for vx
      A(i, 1) = dy;                 // coefficient for vy
      A(i, 2) = -dx * yi + dy * xi; // coefficient for omega

      // b[i] = wheel_velocity[i] * r (convert to linear velocity)
      b(i) = wheel_velocities[i] * wheel_radius_;
    }

    // Solve using pseudo-inverse: x = (A^T * A)^(-1) * A^T * b
    Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::VectorXd Atb = A.transpose() * b;
    Eigen::VectorXd x = AtA.ldlt().solve(Atb);

    double vx = x(0);
    double vy = x(1);
    double omega = x(2);

    // Update odometry
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();

    if (dt > 0.0 && dt < 1.0) // Sanity check
    {
      // Integrate velocity to get position
      double dx = vx * std::cos(yaw_) - vy * std::sin(yaw_);
      double dy = vx * std::sin(yaw_) + vy * std::cos(yaw_);

      x_ += dx * dt;
      y_ += dy * dt;
      yaw_ += omega * dt;

      // Normalize yaw to [-pi, pi]
      while (yaw_ > M_PI)
        yaw_ -= 2.0 * M_PI;
      while (yaw_ < -M_PI)
        yaw_ += 2.0 * M_PI;
    }

    last_time_ = current_time;

    // Publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = omega;

    // Covariance (placeholder values)
    odom.pose.covariance[0] = 0.1;  // x
    odom.pose.covariance[7] = 0.1;  // y
    odom.pose.covariance[35] = 0.1; // yaw
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[7] = 0.1;
    odom.twist.covariance[35] = 0.1;

    odom_pub_.publish(odom);

    // Publish TF
    if (publish_tf_ && tf_broadcaster_)
    {
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = current_time;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;

      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = 0.0;

      transform.transform.rotation.x = q.x();
      transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z();
      transform.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(transform);
    }
  }

  void update()
  {
    // Safety: stop wheels if no command received for cmd_timeout
    double current_time = ros::Time::now().toSec();
    if (current_time - last_cmd_time_ > cmd_timeout_)
    {
      // Publish zero commands using Float64MultiArray
      std_msgs::Float64MultiArray zero_cmd;
      zero_cmd.data.resize(4, 0.0);
      wheel_cmd_pub_.publish(zero_cmd);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher wheel_cmd_pub_; // Single publisher for Float64MultiArray
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  double wheel_radius_;
  double L_HALF_;
  double W_HALF_;
  double k_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  double cmd_timeout_;

  // Wheel configuration
  std::vector<std::pair<double, double>> wheel_positions_; // (x, y) in base_link
  std::vector<double> wheel_alpha_;                        // Effective rolling direction angles
  std::vector<double> sign_per_wheel_;                     // Sign calibration per wheel
  std::vector<std::string> joint_names_;

  // Odometry state
  double x_, y_, yaw_;
  ros::Time last_time_;
  double last_cmd_time_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xdrive_kinematics_node");

  XDriveKinematicsNode node;

  ros::Rate rate(50); // 50 Hz

  while (ros::ok())
  {
    ros::spinOnce();
    node.update();
    rate.sleep();
  }

  return 0;
}
