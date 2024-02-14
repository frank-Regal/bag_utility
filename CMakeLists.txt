cmake_minimum_required(VERSION 3.0.2)
project(bag_utility)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  audio_common_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs rosbag audio_common_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare ROS executable
add_executable(save_bags src/save_bags.cpp)
target_link_libraries(save_bags ${catkin_LIBRARIES})