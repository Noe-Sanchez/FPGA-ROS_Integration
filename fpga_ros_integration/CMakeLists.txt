cmake_minimum_required(VERSION 3.0.2)
project(fpga_ros_integration)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
)

add_message_files(FILES
  fpga_data.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(control_node src/uav_control.cpp)
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node ${catkin_LIBRARIES})

add_executable(logic_node src/action_manager.cpp)
add_dependencies(logic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(logic_node ${catkin_LIBRARIES})
