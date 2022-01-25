cmake_minimum_required(VERSION 2.8.3)
project(csi_rover_description)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  joint_state_controller
  robot_state_publisher
)

catkin_package()

install(DIRECTORY launch sdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


