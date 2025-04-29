#pragma once

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <string>

class QoSHelper
{
public:
  explicit QoSHelper(const std::string& package_name, const std::string& yaml_relative_path);

  rclcpp::QoS getQoSForTopic(const std::string &topic_name) const;

private:
  std::unordered_map<std::string, rclcpp::QoS> qos_map_;

  static rclcpp::QoS parseQoS(const YAML::Node& node);
};
