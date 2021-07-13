#include <algorithm>
#include <thread>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include "calibrator_listener/calibrator_listener.hpp"
#include "calibrator_listener/calibrator_print.hpp"

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
  debug_printf("----- CalibratorListener::extrinsic_reader() ..... raw_transform_ :\n");
  for(auto elem : raw_transform_){
    std::cout << elem << std::endl;
  }
  return true;
}


bool CalibratorListener::read_params(){
  bool flag_offline = nh_.getParam("offline", offline_);
  bool flag_mode = nh_.getParam("mode", mode_);
  bool flag_service = nh_.getParam("service_name", extrinsic_service_name_);
  bool flag_extrinsic_filepath = nh_.getParam("extrinsic_file", extrinsic_filepath_);
  bool flag_nav_filepath = nh_.getParam("nav_param_file", nav_filepath_);
  bool flag_output_filepath = nh_.getParam("output_file", output_filepath_);
  bool flag_tf_freq = nh_.getParam("tf_frequency", tf_freq_);

  if (!flag_offline || !flag_mode || !flag_service || !flag_extrinsic_filepath || !flag_nav_filepath || !flag_output_filepath || !flag_tf_freq){
    ROS_ERROR("offline_ || mode_ || extrinsic_service_name_ || extrinsic_filepath_ || nav_filepath_ || output_filepath_ || tf_freq_ read FAILURE\n");
    return false;
  } else {
    debug_printf("----- CalibratorListener::readParams() ..... offline_ = %d\n", offline_);
    debug_printf("----- CalibratorListener::readParams() ..... mode_ = %d\n", mode_);
    debug_printf("----- CalibratorListener::readParams() ..... extrinsic_service_name_ = %s\n", extrinsic_service_name_.c_str());
    debug_printf("----- CalibratorListener::readParams() ..... extrinsic_filepath_ = %s\n", extrinsic_filepath_.c_str());
    debug_printf("----- CalibratorListener::readParams() ..... nav_filepath_ = %s\n", nav_filepath_.c_str());
    debug_printf("----- CalibratorListener::readParams() ..... output_filepath_ = %s\n", output_filepath_.c_str());
    debug_printf("----- CalibratorListener::readParams() ..... tf_freq_ = %f\n", tf_freq_);
    return true;
  }
}


void CalibratorListener::tf_static_callback(const tf2_msgs::TFMessage& msg) {
  debug_printf("----- CalibratorListener::tf_static_callback() ..... calling\n");

  geometry_msgs::TransformStamped current_tf;
  current_tf.header = msg.transforms[0].header;
  current_tf.child_frame_id = msg.transforms[0].child_frame_id;
  current_tf.transform = msg.transforms[0].transform;

  all_transforms_.push_back(current_tf);

  std::pair<std::set<std::string>::iterator, bool> feedback_parent = all_frames_set_.insert(msg.transforms[0].header.frame_id);
  std::pair<std::set<std::string>::iterator, bool> feedback_child = all_frames_set_.insert(msg.transforms[0].child_frame_id);

  if (feedback_parent.second) {
    all_frames_vector_.push_back(msg.transforms[0].header.frame_id);
    if (msg.transforms[0].header.frame_id.find("/base") != std::string::npos) {
      major_frames_.push_back(msg.transforms[0].header.frame_id);
    }
  }

  if (feedback_child.second) {
    all_frames_vector_.push_back(msg.transforms[0].child_frame_id);
    if (msg.transforms[0].child_frame_id.find("/base") != std::string::npos) {
      major_frames_.push_back(msg.transforms[0].child_frame_id);
    }
  }

  debug_printf("----- CalibratorListener::tf_static_callback() ..... %lu transforms being captured so far.\n", all_transforms_.size());
  debug_printf("----- CalibratorListener::tf_static_callback() ..... ALL FRAMES -----\n");
  int count = 0;
  for (auto& elem : all_frames_vector_) {
    debug_printf("----- ----- %d : %s\n", count, elem.c_str());
    count++;
  }
  debug_printf("----- CalibratorListener::tf_static_callback() ..... MAJOR FRAMES -----\n");
  count = 0;
  for (auto& elem : major_frames_) {
    debug_printf("----- ----- %d : %s\n", count, elem.c_str());
    count++;
  }
}


bool CalibratorListener::extrinsic_callback(robot_basic_tools::Extrinsic::Request& req, robot_basic_tools::Extrinsic::Response& res) {
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... calling\n");

  parent_true_ = req.tfs.header.frame_id;
  parent_true_naked_ = std::string(parent_true_, 1);
  child_true_ = req.tfs.child_frame_id;
  child_true_naked_ = std::string(child_true_, 1);
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_true_ = %s \t child_true_ = %s\n", parent_true_.c_str(), child_true_.c_str());
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_true_naked_ = %s \t child_true_naked_ = %s\n", parent_true_naked_.c_str(), child_true_naked_.c_str());

  parent_selected_ = req.parent_selected;
  parent_selected_naked_ = std::string(parent_selected_, 1);
  child_selected_ = req.child_selected;
  child_selected_naked_ = std::string(child_selected_, 1);
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_selected_ = %s \t child_selected_ = %s\n", parent_selected_.c_str(), child_selected_.c_str());
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_selected_naked_ = %s \t child_selected_naked_ = %s\n", parent_selected_naked_.c_str(), child_selected_naked_.c_str());

  tf::StampedTransform tmp_stf;
  tf::transformStampedMsgToTF(req.tfs, tmp_stf);
  transform_pt2ct_ = tf::Transform(tmp_stf.getRotation(), tmp_stf.getOrigin());
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... transform_pt2ct_.q: [%f, %f, %f, %f]\n",
               transform_pt2ct_.getRotation().x(), transform_pt2ct_.getRotation().y(), transform_pt2ct_.getRotation().z(), transform_pt2ct_.getRotation().w());
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... transform_pt2ct_.t: [%f, %f, %f]\n",
               transform_pt2ct_.getOrigin().x(), transform_pt2ct_.getOrigin().y(), transform_pt2ct_.getOrigin().z());


  if (!update_nav_params()) {
    ROS_ERROR("extrinsic_callback() : Error updating received extrinsic into navigation params");
    return false;
  }

  if (!write_nav_params()) {
    ROS_ERROR("extrinsic_callback() : Error writing updated parameters into json format");
    return false;
  }

  return true;
}

bool CalibratorListener::update_nav_params() {
  debug_printf("----- CalibratorListener::update_nav_params() ..... start updating nav params\n");

  bool found_cs2base = false;
  for (const auto& iter : nav_params_) {
    if (std::strcmp(iter.frame_.c_str(), child_selected_naked_.c_str()) == 0) {
      transform_cs2base_ = iter.transform_sensor2base_;
      found_cs2base = true;
      break;
    }
  }
  if (!found_cs2base) {
    ROS_ERROR("Unable to find %s in navigation platform parameters", child_selected_.c_str());
    return false;
  }

  if (std::strcmp(parent_true_.c_str(), parent_selected_.c_str()) != 0) {
    debug_printf("----- CalibratorListener::update_nav_params() ..... Looking for intermediate transform from %s to %s\n", parent_selected_naked_.c_str(), parent_true_naked_.c_str());

    tf2_ros::Buffer tmp_buff;
    tf2_ros::TransformListener tfl(tmp_buff);
    tf::StampedTransform tmp_stf;
    try {
      tf::transformStampedMsgToTF(tmp_buff.lookupTransform(parent_selected_naked_, parent_true_naked_, ros::Time(0), ros::Duration(1.0)), tmp_stf);
      transform_ps2pt_ = tf::Transform(tmp_stf.getRotation(), tmp_stf.getOrigin());
      debug_printf("Found intermediate transform 1 !!\n");
      debug_printf("parent: %s\n", tmp_stf.frame_id_.c_str());
      debug_printf("child : %s\n", tmp_stf.child_frame_id_.c_str());
      debug_printf("q: [%f, %f, %f, %f]\n", transform_ps2pt_.getRotation().x(), transform_ps2pt_.getRotation().y(), transform_ps2pt_.getRotation().z(), transform_ps2pt_.getRotation().w());
      debug_printf("t: [%f, %f, %f]\n", transform_ps2pt_.getOrigin().x(), transform_ps2pt_.getOrigin().y(), transform_ps2pt_.getOrigin().z());
    } catch (tf2::TransformException& e) {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
    }
  } else {
    transform_ps2pt_ = tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(0, 0, 0));
  }

  if (std::strcmp(child_true_.c_str(), child_selected_.c_str()) != 0) {
    debug_printf("----- CalibratorListener::update_nav_params() ..... Looking for intermediate transform from %s to %s\n", child_true_naked_.c_str(), child_selected_naked_.c_str());

    tf2_ros::Buffer tmp_buff;
    tf2_ros::TransformListener tfl(tmp_buff);
    tf::StampedTransform tmp_stf;
    try {
      tf::transformStampedMsgToTF(tmp_buff.lookupTransform(child_true_naked_, child_selected_naked_, ros::Time(0), ros::Duration(1.0)), tmp_stf);
      transform_ct2cs_ = tf::Transform(tmp_stf.getRotation(), tmp_stf.getOrigin());
      debug_printf("Found intermediate transform 2 !!\n");
      debug_printf("parent: %s\n", tmp_stf.frame_id_.c_str());
      debug_printf("child : %s\n", tmp_stf.child_frame_id_.c_str());
      debug_printf("q: [%f, %f, %f, %f]\n", transform_ct2cs_.getRotation().x(), transform_ct2cs_.getRotation().y(), transform_ct2cs_.getRotation().z(), transform_ct2cs_.getRotation().w());
      debug_printf("t: [%f, %f, %f]\n", transform_ct2cs_.getOrigin().x(), transform_ct2cs_.getOrigin().y(), transform_ct2cs_.getOrigin().z());
    } catch (tf2::TransformException& e) {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
    }
  } else {
    transform_ct2cs_ = tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(0, 0, 0));
  }

  tf::Transform transform_ps2ct = transform_ps2pt_ * transform_pt2ct_;
  tf::Transform transform_ps2cs = transform_ps2ct * transform_ct2cs_;
  transform_ps2base_ = transform_ps2cs * transform_cs2base_;
  debug_printf("----- CalibratorListener::update_nav_params() ..... calculated Transform:\n");
  debug_printf("q: [%f, %f, %f, %f]\n", transform_ps2base_.getRotation().x(), transform_ps2base_.getRotation().y(), transform_ps2base_.getRotation().z(), transform_ps2base_.getRotation().w());
  debug_printf("t: [%f, %f, %f]\n", transform_ps2base_.getOrigin().x(), transform_ps2base_.getOrigin().y(), transform_ps2base_.getOrigin().z());

  bool found_ps2base = false;
  for (auto& iter : nav_params_) {
    if (std::strcmp(iter.frame_.c_str(), parent_selected_naked_.c_str()) == 0) {
      found_ps2base = true;
      iter.update_pos(transform_ps2base_);
      break;
    }
  }
  if (!found_ps2base) {
    ROS_ERROR("Unable to find %s in navigation platform parameters", parent_selected_.c_str());
    return false;
  }

  return true;
}


bool CalibratorListener::read_nav_params() {
  std::ifstream ifs(nav_filepath_, std::ios::in);
  nlohmann::json js_whole;

  if (ifs.is_open()) {
    ROS_INFO("Loading nav_params from file success");
    ifs >> js_whole;
    ifs.close();
  } else {
    ROS_ERROR("Failed loading nav_params");
    return false;
  }

  if (!js_whole.contains("sensorsettings")) {
    ROS_ERROR("----- CalibratorListener::read_nav_params() ..... Possibly wrong json file for nav params, no sensorsettings");
    return false;
  }

  for (const auto& data : js_whole["sensorsettings"].items()) {
    std::string current_frame_id;
    std::string current_type = data.value().at("type");
    std::string current_id= static_cast<std::string>(data.value().at("id").dump());

    if (std::strcmp(current_type.c_str(), "laser") == 0) {
      current_frame_id = "base_laser_link";
    } else {
      for (std::string::iterator iter = current_id.begin(); iter < current_id.end(); ++iter) {
        if (*iter == *const_cast<char*>("\"")) {
          current_id.erase(iter);
          iter--;
        }
      }
      current_frame_id = "base_" + current_type + "_link_" + current_id;
    }

    float current_rx = static_cast<float>(data.value().at("pos").at("rx"));
    float current_ry = static_cast<float>(data.value().at("pos").at("ry"));
    float current_rz = static_cast<float>(data.value().at("pos").at("rz"));
    float current_tx = static_cast<float>(data.value().at("pos").at("tx"));
    float current_ty = static_cast<float>(data.value().at("pos").at("ty"));
    float current_tz = static_cast<float>(data.value().at("pos").at("tz"));

    Sensor current_sensor(current_type, current_id, current_frame_id, {current_rx, current_ry, current_rz, current_tx, current_ty, current_tz});
    nav_params_.push_back(current_sensor);
  }

  debug_printf("----- CalibratorListener::read_nav_params() ..... Loaded sensor list length = %lu\n", nav_params_.size());
  for (const auto& item : nav_params_) {
    item.print_sensor();
  }

  if (!js_whole.contains("robotsettings")) {
    ROS_ERROR("----- CalibratorListener::read_nav_params() ..... Possibly wrong json file for nav params, no robotsettings");
    return false;
  }

  for (const auto& data : js_whole["robotsettings"].items()) {
    js_robot_.emplace_back(data);
  }
//  output_whole_["robotsettings"] = js_robot;
//  output_whole_ = {{"robotsettings", js_robot}};

  return true;
}


bool CalibratorListener::write_nav_params() {
  debug_printf("----- CalibratorListener::write_json() ..... calling\n");

  std::vector<nlohmann::json> js_nav;
  for (const auto& data : nav_params_) {
    nlohmann::json js_pos;
    js_pos["rx"] = data.pos_[0];
    js_pos["ry"] = data.pos_[1];
    js_pos["rz"] = data.pos_[2];
    js_pos["tx"] = data.pos_[3];
    js_pos["ty"] = data.pos_[4];
    js_pos["tz"] = data.pos_[5];
//    js_pos["rx"] = round(data.pos_[0] * 100) / 100;
//    js_pos["ry"] = round(data.pos_[1] * 100) / 100;
//    js_pos["rz"] = round(data.pos_[2] * 100) / 100;
//    js_pos["tx"] = round(data.pos_[3] * 100) / 100;
//    js_pos["ty"] = round(data.pos_[4] * 100) / 100;
//    js_pos["tz"] = round(data.pos_[5] * 100) / 100;
    nlohmann::json js = {{"type", data.type_},
                         {"id", data.id_},
                         {"pos", js_pos}};
    js_nav.push_back(js);
  }
  output_whole_["robotsettings"] = js_robot_;
  output_whole_["sensorsettings"] = js_nav;

  std::ofstream ofs(output_filepath_, std::ios::out);
  if (ofs.is_open()) {
    std::cout << "save data to " << output_filepath_.c_str() << std::endl;
    ofs << std::setw(2) << output_whole_ << std::endl;
    ofs.close();
    return true;
  } else {
    std::cout << "cannot create a file at " << output_filepath_.c_str() << std::endl;
    return false;
  }
}


void CalibratorListener::broadcast_tf() {
  ros::Rate rate(0.2);
  while (nh_.ok()){
    debug_printf("----- CalibratorListener::broadcast_tf() ..... broadcasting\n");
    for (const auto& iter : nav_params_) {
      geometry_msgs::TransformStamped msg;
      tf::transformStampedTFToMsg(tf::StampedTransform(iter.transform_sensor2base_, ros::Time::now(), "/base_link", "/" + iter.frame_), msg);
      br_.sendTransform(msg);
    }
    rate.sleep();
  }
}

// ---------------------------------------------------------------------------------------------------------------------

void CalibratorListener::run() {

  if(!read_params()) {
    ROS_ERROR("Failed reading from parameter centre");
    exit(1);
  }
  debug_printf("\n");

  if (!read_nav_params()) {
    ROS_ERROR("Failed reading navigation platform parameters");
    exit(1);
  }
  debug_printf("\n");

  tf_static_sub_ = nh_.subscribe("/tf_static", 1, &CalibratorListener::tf_static_callback, this);
  debug_printf("\n");

  br_thread_ = std::thread(&CalibratorListener::broadcast_tf, this);

  if (mode_ == 0) {
    debug_printf("----- CalibratorListener::run() ..... MODE 0: Expecting extrinsic from service\n");
    ros::ServiceServer service = nh_.advertiseService(extrinsic_service_name_, &CalibratorListener::extrinsic_callback, this);
    ROS_INFO("Server ready. Waiting for extrinsic service request ......");
    ros::spin();
  } else if (mode_ == 1) {
    debug_printf("----- CalibratorListener::run() ..... MODE 1: Read extrinsic from file\n");
    if (!read_extrinsic(extrinsic_filepath_)) {
      ROS_ERROR("Failed reading extrinsic file");
      exit(1);
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------

int main(int argc, char** argv){

  ros::init(argc, argv, "calibrator_listener");
  ros::NodeHandle nh_("~");
  std::shared_ptr<CalibratorListener> listener(new CalibratorListener(nh_));

  listener->run();

  return 0;
}

// ---------------------------------------------------------------------------------------------------------------------

//bool CalibratorListener::update_nav_params() {
//  debug_printf("----- CalibratorListener::update_nav_params() ..... start updating nav params\n");
//
//  bool found_parent = false;
//  bool found_child = false;
//  Eigen::Quaterniond parent_to_baselink_rotation;
//  Eigen::Vector3d parent_to_baselink_translation;
//
//  for (const auto& iter : nav_params_) {
//    if (std::strcmp(iter.frame_.c_str(), parent_frame_.c_str()) == 0) {
//      found_parent = true;
//      parent_to_baselink_translation = Eigen::Vector3d(iter.pos_[3], iter.pos_[4], iter.pos_[5]);
//
//      Eigen::Vector3d parent_to_baselink_euler(iter.pos_[2], iter.pos_[1], iter.pos_[0]);
//      parent_to_baselink_rotation = Eigen::AngleAxisd(parent_to_baselink_euler[0], Eigen::Vector3d::UnitZ()) *
//                                    Eigen::AngleAxisd(parent_to_baselink_euler[1], Eigen::Vector3d::UnitY()) *
//                                    Eigen::AngleAxisd(parent_to_baselink_euler[2], Eigen::Vector3d::UnitX());
//      break;
//    }
//  }
//
//  if (!found_parent) {
//    ROS_ERROR("Unable to find parent frame %s in navigation platform parameters", parent_frame_.c_str());
//    return false;
//  }
//
//  Eigen::Quaterniond child_to_baselink_rotation = parent_to_baselink_rotation * ext_rotation_;
//  Eigen::Vector3d child_to_baselink_euler = child_to_baselink_rotation.matrix().eulerAngles(2, 1, 0);
//  Eigen::Vector3d child_to_baselink_translation = parent_to_baselink_translation + parent_to_baselink_rotation.toRotationMatrix() * ext_translation_;
//
//  for (auto& iter : nav_params_) {
//    if (std::strcmp(iter.frame_.c_str(), child_frame_.c_str()) == 0) {
//      found_child = true;
//      std::vector<double> child_tf = {child_to_baselink_euler[2], child_to_baselink_euler[1], child_to_baselink_euler[0],
//                                      child_to_baselink_translation[0], child_to_baselink_translation[1], child_to_baselink_translation[2]};
//
//      iter.update_pos(child_tf);
//      break;
//    }
//  }
//
//  if(!found_child) {
//    ROS_ERROR("Unable to find child frame %s in navigation platform parameters", child_frame_.c_str());
//    return false;
//  }
//
//  debug_printf("----- CalibratorListener::update_nav_params() ..... updated nav params:\n");
//  for (const auto& item : nav_params_) {
//    item.print_sensor();
//  }
//  return true;
//}

//    tf::TransformListener lis;
//    ros::Rate rate(10.0);
//    while (nh_.ok()) {
//      tf::StampedTransform transform;
//      try {
//        lis.lookupTransform(selected_parent_frame_, parent_frame_, all_transforms_[0].header.stamp, transform);
//      } catch (tf::TransformException& ex) {
//        ROS_ERROR("%s", ex.what());
//        ros::Duration(1.0).sleep();
//        continue;
//      }
//    }
// Need to find intermediate transforms between true and selected frames only when they are different, otherwise use selected
////  if (std::strcmp(parent_frame_.c_str(), selected_parent_frame_.c_str()) != 0) {
////    debug_printf("----- CalibratorListener::extrinsic_callback() ..... Looking for intermediate transform from %s to %s\n", parent_frame_.c_str(), selected_parent_frame_.c_str());
////
////    std::string clean_true = parent_frame_.erase(parent_frame_.find('/'), 1);
////    std::string clean_selected = selected_parent_frame_.erase(selected_parent_frame_.find('/'), 1);
////    debug_printf("----- CalibratorListener::extrinsic_callback() ..... from %s to %s\n", clean_true.c_str(), clean_selected.c_str());
////
////    tf2_ros::Buffer tmp_buff;
////    tf2_ros::TransformListener tfl(tmp_buff);
////    ros::Time time = ros::Time(1.0);
////    ros::Duration timeout(1.0);
////    geometry_msgs::TransformStamped tmp_tfs;
////
////    try {
////      tmp_tfs = tmp_buff.lookupTransform(clean_selected, clean_true, time, timeout);
////      debug_printf("Found intermediate transform 1 !!\n");
////      debug_printf("source frame = %s\n", tmp_tfs.header.frame_id.c_str());
////      debug_printf("target frame = %s\n", tmp_tfs.child_frame_id.c_str());
////      debug_printf("rotation = [%f, %f, %f, %f]\n", tmp_tfs.transform.rotation.x, tmp_tfs.transform.rotation.y, tmp_tfs.transform.rotation.z, tmp_tfs.transform.rotation.w); /      debug_printf("translation = [%f, %f, %f]\n", tmp_tfs.transform.translation.x, tmp_tfs.transform.translation.y, tmp_tfs.transform.translation.z);
////
////      Eigen::Matrix4d transform_sp_tp = Eigen::Matrix4d::Zero();
////      transform_sp_tp(3, 3) = 1;
////
////    } catch (tf2::TransformException& e) {
////      ROS_ERROR("%s", e.what());
////      ros::Duration(1.0).sleep();
////    }
////  }
////  // Need to find intermediate transforms between true and selected frames only when they are different, otherwise use selected /  if (std::strcmp(child_frame_.c_str(), selected_child_frame_.c_str()) != 0) { /    debug_printf("----- CalibratorListener::extrinsic_callback() ..... Looking for intermediate transform between %s and %s\n", child_frame_.c_str(), selected_child_frame_.c_str());
////
////    std::string clean_true = child_frame_.erase(child_frame_.find('/'), 1);
////    std::string clean_selected = selected_child_frame_.erase(selected_child_frame_.find('/'), 1);
////    debug_printf("----- CalibratorListener::extrinsic_callback() ..... clean_true = %s, clean_selected = %s\n", clean_true.c_str(), clean_selected.c_str());
////
////    tf2_ros::Buffer tmp_buff;
////    tf2_ros::TransformListener tfl(tmp_buff);
////    ros::Time time = ros::Time(1.0);
////    ros::Duration timeout(1.0);
////    geometry_msgs::TransformStamped tmp_tfs;
////    try {
////      tmp_tfs = tmp_buff.lookupTransform(clean_selected, clean_true, time, timeout);
////      debug_printf("Found intermediate transform 2 !!\n");
////      debug_printf("source frame = %s\n", tmp_tfs.header.frame_id.c_str());
////      debug_printf("target frame = %s\n", tmp_tfs.child_frame_id.c_str());
////      debug_printf("rotation = [%f, %f, %f, %f]\n", tmp_tfs.transform.rotation.x, tmp_tfs.transform.rotation.y, tmp_tfs.transform.rotation.z, tmp_tfs.transform.rotation.w); /      debug_printf("translation = [%f, %f, %f]\n", tmp_tfs.transform.translation.x, tmp_tfs.transform.translation.y, tmp_tfs.transform.translation.z); /    } catch (tf2::TransformException& e) { /      ROS_ERROR("%s", e.what()); /      ros::Duration(1.0).sleep(); /    } /  }
////