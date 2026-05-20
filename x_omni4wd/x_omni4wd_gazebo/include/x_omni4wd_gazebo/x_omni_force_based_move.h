/*
 * Gazebo plugin for 4-wheel omni-directional robot with X configuration
 * Based on nexus_ros_force_based_move plugin
 * Thứ tự bánh: (1,2,3,4) = (FL,RL,RR,FR)
 */

#ifndef GAZEBO_ROS_X_OMNI_FORCE_BASED_MOVE_HH
#define GAZEBO_ROS_X_OMNI_FORCE_BASED_MOVE_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>
#include <algorithm>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

  class GazeboRosXOmniForceBasedMove : public ModelPlugin {

    public: 
      GazeboRosXOmniForceBasedMove();
      ~GazeboRosXOmniForceBasedMove();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);
      void publishJointStates();
      tf::Transform getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const;

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      physics::LinkPtr link_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher odometry_pub_;
      ros::Publisher joint_state_pub_;
      ros::Subscriber vel_sub_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::Odometry odom_;
      sensor_msgs::JointState joint_state_;
      std::string tf_prefix_;

      tf::Transform odom_transform_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      double odometry_rate_;
      double cmd_vel_time_out_;
      bool publish_odometry_tf_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
      common::Time last_cmd_vel_time_;

      double x_;
      double y_;
      double rot_;
      bool alive_;
      common::Time last_odom_publish_time_;
#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Pose3d last_odom_pose_;
#else
      math::Pose last_odom_pose_;
#endif
      
      // Control gains
      double torque_yaw_velocity_p_gain_;
      double force_x_velocity_p_gain_;
      double force_y_velocity_p_gain_;

      double max_x_velocity;
      double max_y_velocity;
      double max_yaw_velocity;

      // Wheel separation for kinematics
      double wheel_separation_x_;
      double wheel_separation_y_;

      // Joint pointers for 4 wheels - Thứ tự: (1,2,3,4) = (FL,RL,RR,FR)
      physics::JointPtr front_left_wheel_joint_;
      physics::JointPtr rear_left_wheel_joint_;
      physics::JointPtr rear_right_wheel_joint_;
      physics::JointPtr front_right_wheel_joint_;

      // Inverse kinematics: từ vận tốc robot → vận tốc bánh
      void calculateWheelVelocities(double v_x, double v_y, double omega_z,
                                    double& v_fl, double& v_rl, double& v_rr, double& v_fr);
  };

}

#endif /* GAZEBO_ROS_X_OMNI_FORCE_BASED_MOVE_HH */

