cmake_minimum_required(VERSION 3.10)
project(heuristic_planners)

# Required for message generation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenMP REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Enable C++17
add_compile_options(-std=c++17 -Wall -Wextra -pedantic)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-fsanitize=address,undefined -g3)
  add_link_options(-fsanitize=address,undefined)
endif()

# =================== Include ===================
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# =================== Libraries ===================
set(UTILS_SOURCES
  src/utils/geometry_utils.cpp
  src/utils/LineOfSight.cpp
)

add_library(PlanEnv
  src/plan_env/raycast.cpp
  src/plan_env/grid_map.cpp
)
ament_target_dependencies(PlanEnv rclcpp PCL nav_msgs geometry_msgs visualization_msgs sensor_msgs pcl_conversions cv_bridge)

add_library(AlgorithmBase src/Planners/AlgorithmBase.cpp ${UTILS_SOURCES})
target_link_libraries(AlgorithmBase PlanEnv)
ament_target_dependencies(AlgorithmBase rclcpp PCL)

add_library(AStar src/Planners/AStar.cpp src/Planners/AlgorithmBase.cpp ${UTILS_SOURCES})
target_link_libraries(AStar AlgorithmBase PlanEnv)
ament_target_dependencies(AStar rclcpp PCL)

add_library(ThetaStar src/Planners/ThetaStar.cpp)
target_link_libraries(ThetaStar AlgorithmBase PlanEnv)
ament_target_dependencies(ThetaStar rclcpp PCL)

add_library(ThetaStarAGR src/Planners/ThetaStarAGR.cpp)
target_link_libraries(ThetaStarAGR AlgorithmBase PlanEnv)
ament_target_dependencies(ThetaStarAGR rclcpp PCL)

add_library(ThetaStarAGRpp src/Planners/ThetaStarAGRpp.cpp)
target_link_libraries(ThetaStarAGRpp AlgorithmBase PlanEnv)
ament_target_dependencies(ThetaStarAGRpp rclcpp PCL)

# =================== Executables ===================
add_executable(planner_ros_node src/planner_ros_node.cpp)
target_link_libraries(planner_ros_node
  AlgorithmBase AStar ThetaStar ThetaStarAGR ThetaStarAGRpp PlanEnv
)
ament_target_dependencies(planner_ros_node rclcpp PCL cv_bridge rosidl_typesupport_cpp)
target_include_directories(planner_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# =================== Install ===================
install(TARGETS
  AlgorithmBase AStar ThetaStar ThetaStarAGR ThetaStarAGRpp PlanEnv planner_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Optional Python scripts
install(PROGRAMS
  scripts/test_algorithms.py
  scripts/test_algorithms_compare.py
  scripts/test_algorithms_performance.py
  scripts/test_algorithms_pseudo_random_paths.py
  scripts/compare_trajectories_rviz.py
  scripts/plan_caller.py
  DESTINATION lib/${PROJECT_NAME}
)

# =================== Messages & Services ===================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPath.srv"
  "srv/SetAlgorithm.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

rosidl_target_interfaces(planner_ros_node
${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
