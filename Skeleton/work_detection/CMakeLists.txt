cmake_minimum_required(VERSION 2.8.3)
project(work_detection)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
  std_msgs
  geometry_msgs
)

add_message_files( 
  FILES
  TargetArea.msg
  WorkDetectionResult.msg
  WorkDetectionResultList.msg
)

add_service_files(
  FILES
  WorkDetection.srv
)

generate_messages( 
  DEPENDENCIES 
  std_msgs
  geometry_msgs
)
