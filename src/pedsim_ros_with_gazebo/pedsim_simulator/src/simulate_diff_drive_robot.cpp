#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double g_updateRate, g_simulationFactor;
std::string g_worldFrame, g_robotFrame;
std::string g_odomTopic;
geometry_msgs::Twist g_currentTwist;
geometry_msgs::Pose g_returnPose;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
ros::Publisher g_returnPosePub;
ros::Publisher g_odomPub;
boost::mutex mutex;
bool g_publishReturnPose = true;
bool g_publishOdom = true;

static double sanitize(double value) {
  return std::isfinite(value) ? value : 0.0;
}

/// Diff-drive: v = linear.x, omega = angular.z; x += cos(theta)*v*dt
void updateLoop() {
  ros::Rate rate(g_updateRate);
  const double dt = g_simulationFactor / g_updateRate;

  while (true) {
    double x = g_currentPose.getOrigin().x();
    double y = g_currentPose.getOrigin().y();
    double theta = sanitize(tf::getYaw(g_currentPose.getRotation()));

    double v = 0.0;
    double omega = 0.0;
    {
      boost::mutex::scoped_lock lock(mutex);
      v = sanitize(g_currentTwist.linear.x);
      omega = sanitize(g_currentTwist.angular.z);
    }

    x += cos(theta) * v * dt;
    y += sin(theta) * v * dt;
    theta = sanitize(theta + omega * dt);

    g_currentPose.getOrigin().setX(x);
    g_currentPose.getOrigin().setY(y);
    g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));

    if (g_publishReturnPose) {
      g_returnPose.position.x = x;
      g_returnPose.position.y = y;
      g_returnPose.position.z = 0.0;
      tf::quaternionTFToMsg(g_currentPose.getRotation(), g_returnPose.orientation);
      g_returnPosePub.publish(g_returnPose);
    }

    if (g_publishOdom) {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = g_worldFrame;
      odom.child_frame_id = g_robotFrame;
      odom.pose.pose = g_returnPose;
      odom.twist.twist.linear.x = v;
      odom.twist.twist.angular.z = omega;
      g_odomPub.publish(odom);
    }

    g_transformBroadcaster->sendTransform(tf::StampedTransform(
        g_currentPose, ros::Time::now(), g_worldFrame, g_robotFrame));

    rate.sleep();
  }
}

void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist) {
  boost::mutex::scoped_lock lock(mutex);
  g_currentTwist = *twist;
  g_currentTwist.linear.x = sanitize(g_currentTwist.linear.x);
  g_currentTwist.linear.y = sanitize(g_currentTwist.linear.y);
  g_currentTwist.angular.z = sanitize(g_currentTwist.angular.z);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulate_diff_drive_robot");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");

  privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
  privateHandle.param<std::string>("robot_frame", g_robotFrame, "base_footprint");
  privateHandle.param<std::string>("odom_topic", g_odomTopic, "odom");
  privateHandle.param<double>("/pedsim_simulator/simulation_factor", g_simulationFactor, 1.0);
  privateHandle.param<double>("/pedsim_simulator/update_rate", g_updateRate, 25.0);

  double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
  privateHandle.param<double>("pose_initial_x", initialX, 0.0);
  privateHandle.param<double>("pose_initial_y", initialY, 0.0);
  privateHandle.param<double>("pose_initial_theta", initialTheta, 0.0);
  privateHandle.param<bool>("publish_return_pose", g_publishReturnPose, true);
  privateHandle.param<bool>("publish_odom", g_publishOdom, true);

  g_currentPose.getOrigin().setX(initialX);
  g_currentPose.getOrigin().setY(initialY);
  g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, sanitize(initialTheta)));
  g_returnPose.orientation.w = 1.0;

  g_transformBroadcaster.reset(new tf::TransformBroadcaster());
  if (g_publishReturnPose) {
    g_returnPosePub = nodeHandle.advertise<geometry_msgs::Pose>("return_pose_robot", 1);
  }
  if (g_publishOdom) {
    g_odomPub = nodeHandle.advertise<nav_msgs::Odometry>(g_odomTopic, 1);
  }

  ros::Subscriber twistSubscriber =
      nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 3, onTwistReceived);

  boost::thread updateThread(updateLoop);
  ros::spin();
}
