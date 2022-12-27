cmake_minimum_required(VERSION 2.8.3)
project(spot_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
)
find_package(
  PCL REQUIRED
)

catkin_package()

catkin_python_setup()

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(velodyne_filter src/spot_driver/velodyne_point_filter.cpp)
target_link_libraries(velodyne_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pointcloud_fusion src/spot_driver/pointcloud_fusion.cpp)
target_link_libraries(pointcloud_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS
    scripts/spot_ros
    scripts/spot_controller
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY doc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
