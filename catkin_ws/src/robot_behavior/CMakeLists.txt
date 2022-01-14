cmake_minimum_required(VERSION 2.8.3)
project(robot_behavior)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 roscpp
 geometry_msgs
 sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS
)

add_compile_options(-std=c++11) 

include_directories(
 include/robot_behavior_ros
 include/robot_behavior_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/robot_behavior_ros/RobotBehaviorNode.cpp src/robot_behavior_algorithms/RobotBehaviorAlgorithmTeleop.cpp src/robot_behavior_algorithms/RobotBehaviorAlgorithmWander.cpp src/robot_behavior_algorithms/RobotMoveActions.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
