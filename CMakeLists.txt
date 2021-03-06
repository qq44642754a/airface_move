cmake_minimum_required(VERSION 2.8.3)
project(learn_action)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
  airface_drive_msgs
  tf
  rostime
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
  compressed_image_transport
  compressed_depth_image_transport
  geometry_msgs
)


find_package(Boost REQUIRED)

add_action_files(
  FILES
  TurtleMove.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
 
catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
 
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${boost_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
# ${TinyXML_INCLUDE_DIRS}
 "/usr/include/eigen3"
)

add_executable(TurtleMove_client src/TurtleMove_client.cpp)
target_link_libraries(TurtleMove_client ${catkin_LIBRARIES} ${boost_LIBRARIES})
add_dependencies(TurtleMove_client ${PROJECT_NAME}_gencpp)
 
add_executable(TurtleMove_server src/TurtleMove_server.cpp)
target_link_libraries(TurtleMove_server ${catkin_LIBRARIES} ${boost_LIBRARIES})
add_dependencies(TurtleMove_server ${PROJECT_NAME}_gencpp)

add_executable(move_test src/move_test.cpp)
target_link_libraries(move_test ${catkin_LIBRARIES} ${boost_LIBRARIES})
add_dependencies(move_test ${PROJECT_NAME}_gencpp)

add_executable(move_object src/move_object.cpp)
target_link_libraries(move_object
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


