/*
 * percy_base_ros.cpp
 *
 * Created on: 3 2, 2022 16:38
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#include "percy_base/percy_base_ros.hpp"

#include "percy_base/percy_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace agilexrobotics {
PercyBaseRos::PercyBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter<std::string>("port_name");   //声明参数

  this->declare_parameter<std::string>("odom_frame");
  this->declare_parameter<std::string>("base_frame");
  this->declare_parameter<std::string>("odom_topic_name");

  this->declare_parameter<bool>("is_percy_mini");
  this->declare_parameter<bool>("simulated_robot");
  this->declare_parameter<int>("control_rate");

  LoadParameters();
}

void PercyBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");//获取参数

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");
  this->get_parameter_or<bool>("is_percy_mini", is_percy_mini_, false);
  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- is percy mini: " << std::boolalpha << is_percy_mini_
            << std::endl;


  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool PercyBaseRos::Initialize() {
  if (is_percy_mini_) {
    std::cout << "Robot base: Percy Mini" << std::endl;
  } else {
    std::cout << "Robot base: Percy" << std::endl;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
  if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
    
        robot_ = std::unique_ptr<PercyRobot>();
        std::cout << "Creating interface for Percy with AGX_V2 Protocol"
                  << std::endl;  
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void PercyBaseRos::Stop() { keep_running_ = false; }

void PercyBaseRos::Run() {
  robot_ = std::make_shared<PercyRobot>();
  // instantiate a ROS messenger
  // PercyMessenger messenger(robot_.get(),this);
  std::unique_ptr<PercyMessenger<PercyRobot>> messenger =
      std::unique_ptr<PercyMessenger<PercyRobot>>(new PercyMessenger<PercyRobot>(robot_,this));

  messenger->SetOdometryFrame(odom_frame_);
  messenger->SetBaseFrame(base_frame_);
  messenger->SetOdometryTopicName(odom_topic_name_);
  if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

  // connect to robot and setup ROS subscription
  if (port_name_.find("can") != std::string::npos) {
    if (robot_->Connect(port_name_)) {
      robot_->EnableCommandedMode();
      std::cout << "Using CAN bus to talk with the robot" << std::endl;
    } else {
      std::cout << "Failed to connect to the robot CAN bus" << std::endl;
      return;
    }
  } else {
    std::cout << "Please check the specified port name is a CAN port"
              << std::endl;
    return;
  }

  // publish robot state at 50Hz while listening to twist commands
  messenger->SetupSubscription();
  keep_running_ = true;
  rclcpp::Rate rate(50);
  while (keep_running_) {
    messenger->PublishStateToROS();
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
  
}
}  // namespace agilexrobotics