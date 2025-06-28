cmake_minimum_required(VERSION 3.15)
project(multi_target_tracking)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
rospy
std_msgs
message_generation
geometry_msgs
model_distance_from_height
)

# Uncomment this for Python packages in ROS1
catkin_python_setup()

# Add message files
add_message_files(
DIRECTORY msg
FILES Target.msg TargetArray.msg
)

# Generate messages
generate_messages(
DEPENDENCIES
std_msgs
geometry_msgs
)

# Set up catkin package
catkin_package(
# INCLUDE_DIRS include
# LIBRARIES NNClient
CATKIN_DEPENDS rospy message_runtime std_msgs geometry_msgs model_distance_from_height
#  DEPENDS system_lib
)

# Install Python scripts
install(PROGRAMS
  nodes/JPDA_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files (if they exist)
install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  OPTIONAL)
