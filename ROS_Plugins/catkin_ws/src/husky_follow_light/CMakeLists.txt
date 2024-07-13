cmake_minimum_required(VERSION 3.0.2)
project(husky_follow_light)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

# add_service_files(
#   FILES
#   PerformAction.srv
# )

# add_message_files(
#   FILES
#   Action.msg
# )


catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)









