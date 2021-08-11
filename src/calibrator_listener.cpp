#include "calibrator_listener/calibrator_listener.hpp"
#include "calibrator_listener/calibrator_print.hpp"

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
}


bool CalibratorListener::extrinsic_callback(robot_basic_tools::Extrinsic::Request& req, robot_basic_tools::Extrinsic::Response& res) {
  debug_printf("\n----- CalibratorListener::extrinsic_callback() ..... calling\n");

  parent_true_ = req.tfs.header.frame_id;
  child_true_ = req.tfs.child_frame_id;
  if (std::strcmp(parent_true_.substr(0, 1).c_str(), "/") == 0) {
    parent_true_naked_ = std::string(parent_true_, 1);
  } else {
    parent_true_naked_ = parent_true_;
  }
  if (std::strcmp(child_true_.substr(0, 1).c_str(), "/") == 0) {
    child_true_naked_ = std::string(child_true_, 1);
  } else {
    child_true_naked_ = child_true_;
  }
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_true_ = %s \t child_true_ = %s\n", parent_true_.c_str(), child_true_.c_str());
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_true_naked_ = %s \t child_true_naked_ = %s\n", parent_true_naked_.c_str(), child_true_naked_.c_str());

  parent_selected_ = req.parent_selected;
  child_selected_ = req.child_selected;
  if (std::strcmp(parent_selected_.substr(0, 1).c_str(), "/") == 0) {
    parent_selected_naked_ = std::string(parent_selected_, 1);
  } else {
    parent_selected_naked_ = parent_selected_;
  }
  if (std::strcmp(child_selected_.substr(0, 1).c_str(), "/") == 0) {
    child_selected_naked_ = std::string(child_selected_, 1);
  } else {
    child_selected_naked_ = child_selected_;
  }
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_selected_ = %s \t child_selected_ = %s\n", parent_selected_.c_str(), child_selected_.c_str());
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... parent_selected_naked_ = %s \t child_selected_naked_ = %s\n", parent_selected_naked_.c_str(), child_selected_naked_.c_str());

  tf::StampedTransform tmp_stf;
  tf::transformStampedMsgToTF(req.tfs, tmp_stf);
  transform_pt2ct_ = tf::Transform(tmp_stf.getRotation(), tmp_stf.getOrigin());

  double r, p, y;
  tf::Matrix3x3(transform_pt2ct_.getRotation()).getRPY(r, p, y);
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... transform_pt2ct_.q: [%f, %f, %f, %f]\n",
               transform_pt2ct_.getRotation().x(), transform_pt2ct_.getRotation().y(), transform_pt2ct_.getRotation().z(), transform_pt2ct_.getRotation().w());
  debug_printf("----- CalibratorListener::extrinsic_callback() ..... roll = %f, pitch = %f, yaw = %f\n", r, p, y);
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
  debug_printf("\n");
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
    ROS_ERROR("Unable to find child selected (%s) in navigation platform parameters", child_selected_.c_str());
    return false;
  }

  if (std::strcmp(parent_true_naked_.c_str(), parent_selected_naked_.c_str()) != 0) {
    debug_printf("----- CalibratorListener::update_nav_params() ..... Looking for intermediate transform from %s to %s\n", parent_selected_naked_.c_str(), parent_true_naked_.c_str());
    tf2_ros::Buffer tmp_buff;
    tf2_ros::TransformListener tfl(tmp_buff);
    tf::StampedTransform tmp_stf;
    try {
      tf::transformStampedMsgToTF(tmp_buff.lookupTransform(parent_selected_naked_, parent_true_naked_, ros::Time(0), ros::Duration(1.0)), tmp_stf);
      transform_ps2pt_ = tf::Transform(tmp_stf.getRotation(), tmp_stf.getOrigin());

      double r, p, y;
      tf::Matrix3x3(transform_ps2pt_.getRotation()).getRPY(r, p, y);
      debug_printf("Found intermediate transform 1 !!\n");
      debug_printf("parent: %s\n", tmp_stf.frame_id_.c_str());
      debug_printf("child : %s\n", tmp_stf.child_frame_id_.c_str());
      debug_printf("roll = %f, pitch = %f, yaw = %f\n", r, p, y);
      debug_printf("q: [%f, %f, %f, %f]\n", transform_ps2pt_.getRotation().x(), transform_ps2pt_.getRotation().y(), transform_ps2pt_.getRotation().z(), transform_ps2pt_.getRotation().w());
      debug_printf("t: [%f, %f, %f]\n", transform_ps2pt_.getOrigin().x(), transform_ps2pt_.getOrigin().y(), transform_ps2pt_.getOrigin().z());
    } catch (tf2::TransformException& e) {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
    }
  } else {
    transform_ps2pt_ = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
  }

  if (std::strcmp(child_true_naked_.c_str(), child_selected_naked_.c_str()) != 0) {
    debug_printf("----- CalibratorListener::update_nav_params() ..... Looking for intermediate transform from %s to %s\n", child_true_naked_.c_str(), child_selected_naked_.c_str());
    tf2_ros::Buffer tmp_buff;
    tf2_ros::TransformListener tfl(tmp_buff);
    tf::StampedTransform tmp_stf;
    try {
      tf::transformStampedMsgToTF(tmp_buff.lookupTransform(child_true_naked_, child_selected_naked_, ros::Time(0), ros::Duration(1.0)), tmp_stf);
      transform_ct2cs_ = tf::Transform(tmp_stf.getRotation(), tmp_stf.getOrigin());

      double r, p, y;
      tf::Matrix3x3(transform_ps2pt_.getRotation()).getRPY(r, p, y);
      debug_printf("Found intermediate transform 2 !!\n");
      debug_printf("parent: %s\n", tmp_stf.frame_id_.c_str());
      debug_printf("child : %s\n", tmp_stf.child_frame_id_.c_str());
      debug_printf("roll = %f, pitch = %f, yaw = %f\n", r, p, y);
      debug_printf("q: [%f, %f, %f, %f]\n", transform_ct2cs_.getRotation().x(), transform_ct2cs_.getRotation().y(), transform_ct2cs_.getRotation().z(), transform_ct2cs_.getRotation().w());
      debug_printf("t: [%f, %f, %f]\n", transform_ct2cs_.getOrigin().x(), transform_ct2cs_.getOrigin().y(), transform_ct2cs_.getOrigin().z());
    } catch (tf2::TransformException& e) {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
    }
  } else {
    transform_ct2cs_ = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
  }

  tf::Transform transform_ct2base = transform_cs2base_ * transform_ct2cs_.inverse();
  tf::Transform transform_pt2base = transform_ct2base * transform_pt2ct_;
  transform_ps2base_ = transform_pt2base * transform_ps2pt_.inverse();
  debug_printf("----- CalibratorListener::update_nav_params() ..... calculated Transform as below ----- -----\n");
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
    ROS_INFO("Unable to find parent selected (%s) in navigation platform parameters", parent_selected_.c_str());
    return false;
//    Sensor current_sensor(current_type, current_id, current_frame_id, {current_rx, current_ry, current_rz, current_tx, current_ty, current_tz});
  }

  return true;
}


bool CalibratorListener::read_nav_params() {
  std::ifstream ifs(nav_filepath_, std::ios::in);

  if (ifs.is_open()) {
    ROS_INFO("----- CalibratorListener::read_nav_params() ..... Loading nav_params from file success");
    ifs >> js_input_whole_;
    ifs.close();
  } else {
    ROS_ERROR("----- CalibratorListener::read_nav_params() ..... Failed loading nav_params");
    return false;
  }

  if (!js_input_whole_.contains("sensorsettings")) {
    ROS_ERROR("----- CalibratorListener::read_nav_params() ..... Possibly wrong json file for nav params, no sensorsettings");
    return false;
  }

  for (const auto& data : js_input_whole_["sensorsettings"].items()) {
    std::string current_frame_id;
    std::string current_type = data.value().at("type");
    std::string current_id= static_cast<std::string>(data.value().at("id").dump());

    if (std::strcmp(current_type.c_str(), "laser") == 0) {
      current_frame_id = "base_laser_link";
      current_id = "0";
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

  if (!js_input_whole_.contains("robotsettings")) {
    ROS_ERROR("----- CalibratorListener::read_nav_params() ..... Possibly wrong json file for nav params, no robotsettings");
    return false;
  }

  for (const auto& data : js_input_whole_["robotsettings"].items()) {
    js_robot_[data.key()] = data.value();
  }

  return true;
}


bool CalibratorListener::write_nav_params() {
  debug_printf("----- CalibratorListener::write_nav_params() ..... calling\n");

  for (const auto& data : nav_params_) {
    nlohmann::json js_pos;
//    js_pos["rx"] = data.pos_[0];
//    js_pos["ry"] = data.pos_[1];
//    js_pos["rz"] = data.pos_[2];
//    js_pos["tx"] = data.pos_[3];
//    js_pos["ty"] = data.pos_[4];
    js_pos["tz"] = data.pos_[5];
    js_pos["rx"] = round(data.pos_[0] * 1000) / 1000;
    js_pos["ry"] = round(data.pos_[1] * 1000) / 1000;
    js_pos["rz"] = round(data.pos_[2] * 1000) / 1000;
    js_pos["tx"] = round(data.pos_[3] * 1000) / 1000;
    js_pos["ty"] = round(data.pos_[4] * 1000) / 1000;
    js_pos["tz"] = round(data.pos_[5] * 1000) / 1000;
    int id_num;
    nlohmann::json js;
    if (regex_match(data.id_, reg_num_pattern_)) {
      id_num = static_cast<int>(std::strtol(data.id_.c_str(), nullptr, 10));
      js = {{"type", data.type_}, {"id", id_num}, {"pos", js_pos}};
    } else {
      js = {{"type", data.type_}, {"id", data.id_}, {"pos", js_pos}};
    }
    js_nav_.push_back(js);
  }
  js_output_whole_["robotsettings"] = js_robot_;
  js_output_whole_["sensorsettings"] = js_nav_;

  std::ofstream ofs(output_filepath_, std::ios::out);
  if (ofs.is_open()) {
    ROS_INFO("----- CalibratorListener::write_nav_params() ..... Saving data to %s", output_filepath_.c_str());
    ofs << std::setw(4) << js_output_whole_ << std::endl;
    ofs.close();
    return true;
  } else {
    ROS_ERROR("----- CalibratorListener::write_nav_params() ..... Cannot create a file at %s", output_filepath_.c_str());
    return false;
  }
}


void CalibratorListener::broadcast_tf() {
  ros::Rate rate(tf_freq_);
  while (nh_.ok()){
//    debug_printf("----- CalibratorListener::broadcast_tf() ..... broadcasting\n");
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

//  br_thread_ = std::thread(&CalibratorListener::broadcast_tf, this);

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