cmake_minimum_required(VERSION 3.10.2)
project(heuristic_planners)

add_compile_options(-std=c++17 -Wall -Wextra -pedantic)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-fsanitize=address,undefined -g3)
  add_link_options(-fsanitize=address,undefined)
endif()

# =================== Dependencies ===================
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
  roscpp
  pcl_conversions
  pcl_ros
  costmap_2d
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_python_setup()

# =================== Messages & Services ===================
add_service_files(
  FILES
    GetPath.srv
    SetAlgorithm.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES AlgorithmBase AStar ThetaStar ThetaStarAGR ThetaStarAGRpp
  CATKIN_DEPENDS std_msgs visualization_msgs geometry_msgs nav_msgs roscpp message_runtime costmap_2d cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plan_env
)

set(UTILS_SOURCES
  src/utils/geometry_utils.cpp
  src/utils/heuristic.cpp
  src/utils/LineOfSight.cpp
  src/utils/metrics.cpp
)

# =================== Libraries ===================
add_library(AlgorithmBase src/Planners/AlgorithmBase.cpp ${UTILS_SOURCES})
add_library(AStar         src/Planners/AStar.cpp src/Planners/AlgorithmBase.cpp ${UTILS_SOURCES})
add_library(ThetaStar     src/Planners/ThetaStar.cpp)
add_library(ThetaStarAGR  src/Planners/ThetaStarAGR.cpp)
add_library(ThetaStarAGRpp src/Planners/ThetaStarAGRpp.cpp)

add_library(PlanEnv
  src/plan_env/obj_predictor.cpp
  src/plan_env/raycast.cpp
  src/plan_env/grid_map.cpp
)
target_link_libraries(PlanEnv ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# =================== Link & Dependency ===================
target_link_libraries(AlgorithmBase ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(AStar        ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ThetaStar    ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ThetaStarAGR ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(ThetaStarAGRpp ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(OpenMP_CXX_FOUND)
  message(STATUS "✅ OpenMP Found: ${OpenMP_CXX_FLAGS}")
  add_definitions(-DOPENMP_ENABLED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  target_link_libraries(AlgorithmBase OpenMP::OpenMP_CXX)
  target_link_libraries(AStar OpenMP::OpenMP_CXX)
  target_link_libraries(ThetaStar OpenMP::OpenMP_CXX)
  target_link_libraries(ThetaStarAGR OpenMP::OpenMP_CXX)
  target_link_libraries(ThetaStarAGRpp OpenMP::OpenMP_CXX)
endif()

add_dependencies(AlgorithmBase     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(AStar             ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ThetaStar         ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ThetaStarAGR      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ThetaStarAGRpp    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(PlanEnv           ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# =================== Executable ===================
add_executable(planner_ros_node src/planner_ros_node.cpp)
target_link_libraries(planner_ros_node
  AlgorithmBase AStar ThetaStar ThetaStarAGR ThetaStarAGRpp PlanEnv
  ${catkin_LIBRARIES} Eigen3::Eigen -lstdc++fs -lcrypto -lssl
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(planner_ros_node OpenMP::OpenMP_CXX)
endif()
add_dependencies(planner_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# =================== Install ===================
install(TARGETS
  AlgorithmBase AStar ThetaStar ThetaStarAGR ThetaStarAGRpp PlanEnv
  planner_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/test_algorithms.py
  scripts/test_algorithms_compare.py
  scripts/test_algorithms_performance.py
  scripts/test_algorithms_pseudo_random_paths.py
  scripts/compare_trajectories_rviz.py
  scripts/plan_caller.py
  setup.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch/ config/ rviz/ resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
