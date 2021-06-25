#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nlohmann/json.hpp>

#include "calibrator_listener/calibrator_listener.h"

void CalibratorListener::publish_extrinsic() {

  std::lock_guard<std::mutex> lock(mtx_);

  ext_publisher_ = nh_.advertise<geometry_msgs::TransformStamped>("extrinsic", 1);
  ros::Rate loop_rate(1);
  geometry_msgs::TransformStamped ext_msg;

  ext_msg.header.stamp = ros::Time::now();
  ext_msg.header.frame_id = "from";
  ext_msg.child_frame_id = "to";

  ext_msg.transform.translation.x = raw_transform_[0];
  ext_msg.transform.translation.y = raw_transform_[1];
  ext_msg.transform.translation.z = raw_transform_[2];

  tf2::Quaternion q;
  q.setRPY(raw_transform_[3], raw_transform_[4], raw_transform_[5]);

  ext_msg.transform.rotation.x = q.x();
  ext_msg.transform.rotation.y = q.y();
  ext_msg.transform.rotation.z = q.z();
  ext_msg.transform.rotation.w = q.w();

  while(ros::ok()) {
    ext_publisher_.publish(ext_msg);
    loop_rate.sleep();
  }
}

bool CalibratorListener::read_extrinsic(const std::string& path){
  std::ifstream ifs(path, std::ios::in);
  nlohmann::json js_file;

  if (ifs.is_open()) {
    ROS_INFO("Loading extrinsic from %s", path.c_str());
    ifs >> js_file;
    ifs.close();
  } else {
    ROS_ERROR("Unable to load extrinsic from %s", path.c_str());
    return false;
  }

  if(!js_file.contains("type") || js_file["type"] != "two_cameras_extrinsic") {
    ROS_ERROR("Wrong file type (%s)", path.c_str());
    return false;
  }

  raw_transform_.clear();
  raw_transform_.resize(6);

  for (const auto& data : js_file["data"].items()) {
    if (std::strcmp(data.key().c_str(), "tx") == 0) {
      raw_transform_[0] = data.value();
    } else if (std::strcmp(data.key().c_str(), "ty") == 0) {
      raw_transform_[1] = data.value();
    } else if (std::strcmp(data.key().c_str(), "tz") == 0) {
      raw_transform_[2] = data.value();
    } else if (std::strcmp(data.key().c_str(), "roll") == 0) {
      raw_transform_[3] = data.value();
    } else if (std::strcmp(data.key().c_str(), "pitch") == 0) {
      raw_transform_[4] = data.value();
    } else if (std::strcmp(data.key().c_str(), "yaw") == 0) {
      raw_transform_[5] = data.value();
    }
  }
  printf("----- CalibratorListener::extrinsic_reader() ..... raw_transform_ :\n");
  for(auto elem : raw_transform_){
    std::cout << elem << std::endl;
  }

  return true;
}

void CalibratorListener::run(){
  readParams();
  if (!read_extrinsic(filepath_)) {
    ROS_ERROR("CalibratorListener::extrinsic_reader() FAILED");
  }
  publish_extrinsic();
}

void CalibratorListener::readParams(){
  bool flag_filepath_ = nh_.getParam("file", filepath_);

  if(!flag_filepath_){
    ROS_WARN("flag_filepath_ read FAILURE\n");
    exit(1);
  } else {
    printf("----- CalibratorListener::readParams() ..... filepath_ = %s\n", filepath_.c_str());
  }
}

// ---------------------------------------------------------------------------------------------------------------------

int main(int argc, char** argv){

  ros::init(argc, argv, "calibrator_listener");
  ros::NodeHandle nh_("~");
  std::shared_ptr<CalibratorListener> listener(new CalibratorListener(nh_));
  ros::AsyncSpinner async_spinner(4);

  async_spinner.start();
  listener->run();
  async_spinner.stop();

  return 0;
}