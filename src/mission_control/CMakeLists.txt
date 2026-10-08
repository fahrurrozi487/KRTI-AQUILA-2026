cmake_minimum_required(VERSION 3.0.2)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  message_generation
)

add_message_files(
  FILES
  ArucoMarker.msg
  ArucoMarkers.msg
  Gate.msg
  Line.msg
  WpMarkerResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/mavros_helper.py
  scripts/mission_node.py
  scripts/record_waypoint.py
  scripts/aruco_detector.py
  scripts/aruco_node.py
  scripts/gate_detector.py
  scripts/gate_node.py
  scripts/gate_multi_detector.py
  scripts/gate_multi_node.py
  scripts/image_convert.py
  scripts/mission_d.py
  scripts/sim_markers.py
  scripts/wp_marker_detector.py
  scripts/wp_marker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
