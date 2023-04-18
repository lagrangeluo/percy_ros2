/*
 * percy_messenger.hpp
 *
 * Created on: Jun 14, 2022 16:37
 * Description:
 *
 * Copyright (c) 2022 tx (tx)
 */

#ifndef TRACER_MESSENGER_HPP
#define TRACER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "percy_msgs/msg/percy_status.hpp"
#include "percy_msgs/msg/percy_light_cmd.hpp"

#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace agilexrobotics {
template <typename PercyType>
class PercyMessenger {
 public:
  PercyMessenger(std::shared_ptr<PercyType> percy, rclcpp::Node *node)
      : percy_(percy), node_(node) {}

  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }

  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }

  void SetupSubscription() {
    // odometry publisher
    odom_pub_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
    status_pub_ = node_->create_publisher<percy_msgs::msg::PercyStatus>(
        "/percy_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 5,
        std::bind(&PercyMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));
    light_cmd_sub_ = node_->create_subscription<percy_msgs::msg::PercyLightCmd>(
        "/light_control", 5,
        std::bind(&PercyMessenger::LightCmdCallback, this,
                  std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    auto state = percy_->GetRobotCoreStateMsgGroup();

    // publish percy state message
    percy_msgs::msg::PercyStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.motion_state.linear_velocity;
    status_msg.angular_velocity = state.motion_state.angular_velocity;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    //status_msg.battery_voltage = state.system_state.battery_voltage;

    auto actuator = percy_->GetActuatorStateMsgGroup();

    for (int i = 0; i < 2; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }

    status_msg.light_control_enabled = state.light_state.lightctl_enable;
    status_msg.front_light_state.mode = state.light_state.front_illu_mode;
    //status_msg.front_light_state.custom_value = state.light_state.front_light.custom_value;
    status_pub_->publish(status_msg);

    MotionStateMessage motion;
    motion.angular_velocity = state.motion_state.angular_velocity;
    motion.linear_velocity = state.motion_state.linear_velocity;
    // publish odometry and tf
    PublishOdometryToROS(motion, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

 private:
  std::shared_ptr<PercyType> percy_;
  rclcpp::Node *node_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<percy_msgs::msg::PercyStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<percy_msgs::msg::PercyLightCmd>::SharedPtr
      light_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!simulated_robot_) {
      SetPercyMotionCommand(msg);
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }
  void SetPercyMotionCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    percy_->SendMotionCommand(msg->linear.x, msg->angular.z);
  }

  void LightCmdCallback(const percy_msgs::msg::PercyLightCmd::SharedPtr msg) {
    if (!simulated_robot_) {

        PercyLightCmdMessage cmd;

        switch (msg->cmd_ctrl_allowed)
        {
          case percy_msgs::msg::PercyLightCmd::LIGHT_CTL_DISABLE:
          {
            cmd.control_enable = AgxPercyLightctlMode::AUTO_CONTROL;
            break;
          }
          case percy_msgs::msg::PercyLightCmd::LIGHT_CTL_ENABLE:
          {
            cmd.control_enable = AgxPercyLightctlMode::CONTROL_ENABLE;
            break;
          }
        }

        switch (msg->front_illumination_mode)
        {
          case percy_msgs::msg::PercyLightCmd::LIGHT_CTL_DISABLE:
          {
            cmd.front_illumination_mode = AgxPercyLightMode::TURN_OFF;
            break;
          }
          case percy_msgs::msg::PercyLightCmd::LIGHT_CTL_ENABLE:
          {
            cmd.front_illumination_mode = AgxPercyLightMode::TURN_ON;
            break;
          }
        }
        switch (msg->back_illumination_mode)
        {
          case percy_msgs::msg::PercyLightCmd::LIGHT_CTL_DISABLE:
          {
          cmd.back_illumination_mode = AgxPercyLightMode::TURN_OFF;
              break;
          }
          case percy_msgs::msg::PercyLightCmd::LIGHT_CTL_ENABLE:
          {
          cmd.back_illumination_mode = AgxPercyLightMode::TURN_ON;
              break;
          }
        }
        percy_->SendLightCommand(cmd);
      } else {
      std::cout << "simulated robot received light control cmd" << std::endl;
    }
  }
  

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    // perform numerical integration to get an estimation of pose
    double linear_speed = msg.linear_velocity;
    double angular_speed = msg.angular_velocity;

    // if (std::is_base_of<PercyMiniOmniRobot, PercyType>::value) {
    //   lateral_speed = msg.lateral_velocity;
    // } else {
    //   lateral_speed = 0;
    // }

    double d_x = linear_speed * std::cos(theta_) * dt;
    double d_y = linear_speed * std::sin(theta_) * dt;
    double d_theta = angular_speed * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed;

    odom_pub_->publish(odom_msg);
  }
};
}  // namespace agilexrobotics

#endif /* SCOUT_MESSENGER_HPP */
