#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

static bool finiteTwist(const geometry_msgs::Twist& t) {
  return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) &&
         std::isfinite(t.angular.z);
}

static double safeYaw(const geometry_msgs::Quaternion& q) {
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-6) {
    return 0.0;
  }
  const double yaw = tf::getYaw(q);
  return std::isfinite(yaw) ? yaw : 0.0;
}

/**
 * Ép cmd_vel holonomic (vx, vy, omega) sang diff-drive: linear.y = 0.
 * Dùng với HRVOLocalPlanner holonomic_mode hoặc nguồn cmd_vel có linear.y != 0.
 */
class DiffDriveTwistAdapter {
 public:
  DiffDriveTwistAdapter() {
    ros::NodeHandle pnh("~");
    pnh.param("project_to_forward", project_to_forward_, false);
    // true: cmd_vel_hrvo.linear.x đã là v tiến theo base_link (HRVOLocalPlanner holonomic_mode=false)
    pnh.param("input_in_body_frame", input_in_body_frame_, false);
    pnh.param("use_odom_topic", use_odom_topic_, true);
    pnh.param("odom_topic", odom_topic_, std::string("/odom"));
    pnh.param("input_topic", input_topic_, std::string("cmd_vel_hrvo"));
    pnh.param("output_topic", output_topic_, std::string("cmd_vel"));
    pnh.param("pose_topic", pose_topic_, std::string("return_pose_robot"));

    sub_twist_ = nh_.subscribe(input_topic_, 5, &DiffDriveTwistAdapter::onTwist, this);
    if (use_odom_topic_) {
      sub_odom_ = nh_.subscribe(odom_topic_, 5, &DiffDriveTwistAdapter::onOdom, this);
    } else {
      sub_pose_ = nh_.subscribe(pose_topic_, 5, &DiffDriveTwistAdapter::onPose, this);
    }
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 5);
  }

 private:
  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_theta_ = safeYaw(msg->pose.pose.orientation);
    has_pose_ = true;
  }

  void onPose(const geometry_msgs::Pose::ConstPtr& msg) {
    robot_theta_ = safeYaw(msg->orientation);
    has_pose_ = true;
  }

  void onTwist(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!finiteTwist(*msg)) {
      ROS_WARN_THROTTLE(1.0,
                        "diff_drive_twist_adapter: bỏ qua cmd không hợp lệ (NaN/Inf)");
      return;
    }

    geometry_msgs::Twist out;
    double v = msg->linear.x;

    if (input_in_body_frame_) {
      v = msg->linear.x;
    } else if (project_to_forward_) {
      const double theta = has_pose_ ? robot_theta_ : 0.0;
      v = msg->linear.x * std::cos(theta) + msg->linear.y * std::sin(theta);
    }

    if (!std::isfinite(v)) {
      v = 0.0;
    }

    out.linear.x = v;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = std::isfinite(msg->angular.z) ? msg->angular.z : 0.0;
    pub_twist_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_twist_;
  bool project_to_forward_{false};
  bool input_in_body_frame_{false};
  bool use_odom_topic_{true};
  bool has_pose_{false};
  double robot_theta_{0.0};
  std::string input_topic_;
  std::string output_topic_;
  std::string pose_topic_;
  std::string odom_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "diff_drive_twist_adapter");
  DiffDriveTwistAdapter node;
  ros::spin();
  return 0;
}
