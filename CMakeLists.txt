cmake_minimum_required(VERSION 3.0.2)
project(pcmap_fuser)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  tf
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


find_package(PCL REQUIRED COMPONENTS common io)



catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
    "${PROJECT_SOURCE_DIR}/include"
)

add_executable(pcmap_fuser_node src/pcmap_fuser.cpp src/pcmap_fuser_node.cpp)
target_link_libraries(pcmap_fuser_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIRS})






install(
  TARGETS
  pcmap_fuser_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)