#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <regex>
#include <map>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "robot_basic_tools/Extrinsic.h"
#include "calibrator_listener/sensor.hpp"

class CalibratorListener {
 public:
  explicit CalibratorListener(ros::NodeHandle& nh) : nh_(nh) {}
  ~CalibratorListener() = default;
  void run();

 private:
  bool read_params();
  bool read_nav_params();
  bool read_extrinsic(const std::string& path);

  bool update_nav_params();
  bool write_nav_params();

  void broadcast_tf();
  void tf_static_callback(const tf2_msgs::TFMessage& msg);
  bool extrinsic_callback(robot_basic_tools::Extrinsic::Request& req, robot_basic_tools::Extrinsic::Response& res);

 private:
  bool offline_;
  double tf_freq_;
  std::string extrinsic_service_name_;
  std::string nav_filepath_;
  std::string output_filepath_;

  std::mutex mtx_;
  std::vector<float> raw_transform_;

  ros::NodeHandle nh_;
  ros::Subscriber tf_static_sub_;
  ros::Subscriber tf_sub_;
  ros::Publisher ext_publisher_;

  std::vector<geometry_msgs::TransformStamped> all_transforms_;
  std::set<std::string> all_frames_set_;
  std::vector<std::string> all_frames_vector_;
  std::vector<std::string> major_frames_;

  std::string parent_true_;
  std::string parent_true_naked_;
  std::string child_true_;
  std::string child_true_naked_;

  std::string parent_selected_;
  std::string parent_selected_naked_;
  std::string child_selected_;
  std::string child_selected_naked_;

  tf::Transform transform_cs2base_;
  tf::Transform transform_pt2ct_;
  tf::Transform transform_ps2pt_;
  tf::Transform transform_ct2cs_;
  tf::Transform transform_ps2base_;

  std::thread br_thread_;
  tf2_ros::TransformBroadcaster br_;

  std::vector<Sensor> nav_params_;
  std::vector<nlohmann::json> js_nav_;
  nlohmann::json js_robot_;
  nlohmann::json js_input_whole_;
  nlohmann::json js_output_whole_;


  std::regex reg_num_pattern_ = std::regex(R"(^[\d]+[\.]?[\d+]?$)");
};

