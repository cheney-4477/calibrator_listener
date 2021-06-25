#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>

class CalibratorListener {
 public:
  explicit CalibratorListener(ros::NodeHandle& nh) : nh_(nh) {}
  ~CalibratorListener() = default;
  void run();

 private:
  void readParams();
  bool read_extrinsic(const std::string& path);
  void publish_extrinsic();

 private:
  ros::NodeHandle nh_;
  std::mutex mtx_;

  ros::Publisher ext_publisher_;

  std::string filepath_;
  std::vector<float> raw_transform_;

};

