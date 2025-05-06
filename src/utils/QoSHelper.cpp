#include "utils/QoSHelper.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>

QoSHelper::QoSHelper(const std::string& package_name, const std::string& yaml_relative_path)
{
  std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
  std::string yaml_file_path = package_share_dir + "/" + yaml_relative_path;

  YAML::Node config = YAML::LoadFile(yaml_file_path);
  if (!config["topics"]) {
    throw std::runtime_error("QoSHelper: No 'topics' field in YAML file: " + yaml_file_path);
  }

  std::cout << "[QoSHelper] Loading QoS config from: " << yaml_file_path << std::endl;

  for (const auto& topic : config["topics"]) {
    const std::string topic_name = topic.first.as<std::string>();
    const YAML::Node qos_node = topic.second;
    std::cout << "  - Topic: " << topic_name
    << ", Reliability: " << (qos_node["reliability"] ? qos_node["reliability"].as<std::string>() : "best_effort")
    << ", History: " << (qos_node["history"] ? qos_node["history"].as<std::string>() : "keep_all")
    << ", Durability: " << (qos_node["durability"] ? qos_node["durability"].as<std::string>() : "volatile")
    << ", Depth: " << (qos_node["depth"] ? qos_node["depth"].as<int>() : 10)
    << std::endl;
    qos_map_.insert({topic_name, parseQoS(qos_node)});
  }
}

rclcpp::QoS QoSHelper::getQoSForTopic(const std::string &topic_name) const
{
  auto it = qos_map_.find(topic_name);
  if (it != qos_map_.end()) {
    return it->second;
  }
  return rclcpp::QoS(rclcpp::KeepAll()).reliability(rclcpp::ReliabilityPolicy::BestEffort);
}

rclcpp::QoS QoSHelper::parseQoS(const YAML::Node& node)
{
  std::string reliability_str = node["reliability"] ? node["reliability"].as<std::string>() : "best_effort";
  std::string history_str = node["history"] ? node["history"].as<std::string>() : "keep_all";
  std::string durability_str = node["durability"] ? node["durability"].as<std::string>() : "volatile";
  int depth = node["depth"] ? node["depth"].as<int>() : 10;
  
  rclcpp::QoS qos = (history_str == "keep_all") ?
  rclcpp::QoS(rclcpp::KeepAll()) :
  rclcpp::QoS(rclcpp::KeepLast(depth));
  
  if (reliability_str == "best_effort") {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  }
  if (durability_str == "transient_local") {
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  } else {
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
  }

  return qos;
}
