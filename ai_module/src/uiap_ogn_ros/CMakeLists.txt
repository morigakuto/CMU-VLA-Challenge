cmake_minimum_required(VERSION 3.0.2)
project(uiap_ogn_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  message_filters
  tf2_ros
  cv_bridge
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
    message_filters
    tf2_ros
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/policy_node.py
  scripts/question_router.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/launch_vlm_servers_nohup.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
