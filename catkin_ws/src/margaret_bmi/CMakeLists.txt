cmake_minimum_required(VERSION 2.8.3)
project(margaret_bmi)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  roscpp
  rospy
  roslaunch
  laser_geometry
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

roslaunch_add_file_check(launch)

###########
## Build ##
###########

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(bno055RawToImu_node src/bno055RawToImu_node.cpp)
add_executable(xv11RawToLaserScan_node src/xv11RawToLaserScan_node.cpp)

add_dependencies(bno055RawToImu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(xv11RawToLaserScan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bno055RawToImu_node
  ${catkin_LIBRARIES}
)
target_link_libraries(xv11RawToLaserScan_node
  ${catkin_LIBRARIES}
)

# Install C++ and Python Executables
install(TARGETS 
  bno055RawToImu_node
  xv11RawToLaserScan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#catkin_install_python(PROGRAMS bin/adaptive_dither_dqn_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)