cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_move_it_config)

find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  pcl_conversions
  pcl_ros
  pluginlib
  rosbag
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  message_generation
  std_msgs
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
  FILES 
  SetAttachedObject.srv
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)





###########
## Build ##
###########

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(vertical src/vertical.cpp)
target_link_libraries(vertical ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS vertical DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(packaging src/packaging.cpp)
target_link_libraries(packaging ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS packaging DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(add_obstacle src/add_obstacle.cpp)
target_link_libraries(add_obstacle ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS add_obstacle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(attached_service_server src/attached_service_server.cpp)
target_link_libraries(attached_service_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS attached_service_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})