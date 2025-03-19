cmake_minimum_required(VERSION 3.10.2)
project(heuristic_planners)

add_compile_options(-std=c++17 -Wall -Wextra -pedantic)

option(OPTIMIZE_FLAG        "Enable Compiler Optimization for Runtime Performance" ON)

if(OPTIMIZE_FLAG)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  set(CMAKE_BUILD_TYPE Release)
else()
  add_compile_options(-fsanitize=address,undefined -static-libasan -g3)
  set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(Boost)
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "✅ OpenMP Found: ${OpenMP_CXX_FLAGS}")
    add_definitions(-DOPENMP_ENABLED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
  roscpp
  pcl_conversions
  pcl_ros
  octomap_ros
  costmap_2d
  cv_bridge
)
find_package (Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_python_setup()

add_service_files(
  FILES
  GetPath.srv
  SetAlgorithm.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES AlgorithmBase AStar ThetaStar ThetaStarAGR
 CATKIN_DEPENDS std_msgs visualization_msgs geometry_msgs nav_msgs roscpp message_runtime costmap_2d cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/plan_env
)

##This is True by default but it's good to have it here as a reminder
set(BUILD_SHARED_LIBS TRUE)
list(APPEND ${PROJECT_NAME}_UTILS_SOURCES src/utils/geometry_utils.cpp
                                          src/utils/heuristic.cpp
                                          src/utils/LineOfSight.cpp
                                          src/utils/metrics.cpp       
                                          )

add_library(AlgorithmBase                   src/Planners/AlgorithmBase.cpp 
                                            ${${PROJECT_NAME}_UTILS_SOURCES})

add_library(AStar                  src/Planners/AStar.cpp
                                            src/Planners/AlgorithmBase.cpp 
                                            ${${PROJECT_NAME}_UTILS_SOURCES}
                                            )

add_library(ThetaStar          src/Planners/ThetaStar.cpp )
add_library(ThetaStarAGR        src/Planners/ThetaStarAGR.cpp )

add_library(PlanEnv            src/plan_env/edt_environment.cpp
                               src/plan_env/obj_predictor.cpp
                               src/plan_env/raycast.cpp
                               src/plan_env/sdf_map.cpp)
target_link_libraries(PlanEnv
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

list(APPEND ${PROJECT_NAME}_LIBRARIES AlgorithmBase AStar ThetaStar ThetaStarAGR PlanEnv)  
target_link_libraries(${${PROJECT_NAME}_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
if (OpenMP_CXX_FOUND)
  target_link_libraries(${${PROJECT_NAME}_LIBRARIES} OpenMP::OpenMP_CXX)
endif()
add_dependencies( ${${PROJECT_NAME}_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_TARGETS ${${PROJECT_NAME}_LIBRARIES})

add_executable(planner_ros_node src/planner_ros_node.cpp )
add_dependencies(planner_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_LIBRARIES})
target_link_libraries(planner_ros_node ${catkin_LIBRARIES} ${${PROJECT_NAME}_LIBRARIES} PlanEnv -lstdc++fs Eigen3::Eigen -lcrypto -lssl)
if(OpenMP_CXX_FOUND)
  target_link_libraries(planner_ros_node OpenMP::OpenMP_CXX)
endif()
list(APPEND ${PROJECT_NAME}_TARGETS planner_ros_node)



install(TARGETS ${${PROJECT_NAME}_TARGETS}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS  scripts/test_algorithms.py
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

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
install(DIRECTORY resources/ 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)
