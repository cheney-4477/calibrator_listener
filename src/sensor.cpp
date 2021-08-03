#include "calibrator_listener/sensor.hpp"

void Sensor::print_sensor() const {
  printf("----- ----- type: %s \t id: %s \t frame: %s\n", type_.c_str(), id_.c_str(), frame_.c_str());
  printf("----- ----- ----- pos_: [%f, %f, %f, %f, %f, %f]\n", pos_[0], pos_[1], pos_[2], pos_[3], pos_[4], pos_[5]);
  printf("----- ----- ----- q: [%f, %f, %f, %f]\n",
         transform_sensor2base_.getRotation().x(), transform_sensor2base_.getRotation().y(),
         transform_sensor2base_.getRotation().z(), transform_sensor2base_.getRotation().w());
  printf("----- ----- ----- t: [%f, %f, %f]\n",
         transform_sensor2base_.getOrigin().x(), transform_sensor2base_.getOrigin().y(), transform_sensor2base_.getOrigin().z());
}

void Sensor::update_pos(tf::Transform incoming_tf) {
  transform_sensor2base_ = incoming_tf;

  pos_.clear();
  pos_[3] = incoming_tf.getOrigin().x();
  pos_[4] = incoming_tf.getOrigin().y();
  pos_[5] = incoming_tf.getOrigin().z();
  tf::Matrix3x3(incoming_tf.getRotation()).getRPY(pos_[0], pos_[1], pos_[2]);
}