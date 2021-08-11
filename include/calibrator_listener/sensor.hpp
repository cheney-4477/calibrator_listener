#pragma once

#include <iostream>
#include <utility>
#include <vector>

#include <tf/transform_datatypes.h>

class Sensor {
 public:
  Sensor(std::string type, std::string id, std::string frame, std::vector<double> pos) : type_(std::move(type)), id_(std::move(id)),
                                                                                         frame_(std::move(frame)), pos_(std::move(pos)) {
    tf::Quaternion q = tf::createQuaternionFromRPY(pos_[0], pos_[1], pos_[2]);
    tf::Vector3 t = tf::Vector3(pos_[3], pos_[4], pos_[5]);
    transform_sensor2base_ = tf::Transform(q, t);
  }

  Sensor(std::string type, std::string id, std::string frame, tf::Transform tf) : type_(std::move(type)), id_(std::move(id)),
                                                                                  frame_(std::move(frame)), transform_sensor2base_(tf) {
    pos_[3] = tf.getOrigin().x();
    pos_[4] = tf.getOrigin().y();
    pos_[5] = tf.getOrigin().z();
    tf::Matrix3x3(tf.getRotation()).getRPY(pos_[0], pos_[1], pos_[2]);
  }

  void print_sensor() const;
  void update_pos(tf::Transform incoming_tf);

 public:
  std::string type_;
  std::string id_;
  std::string frame_;
  std::vector<double> pos_;
  tf::Transform transform_sensor2base_;
};