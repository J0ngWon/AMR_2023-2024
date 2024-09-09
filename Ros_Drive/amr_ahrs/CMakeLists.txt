cmake_minimum_required(VERSION 2.8.3)
project(amr_ahrs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  sensor_msgs
  std_srvs
  serial
  realtime_tools
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(mw_ahrs_driver_node
  src/mw_ahrs_driver.cpp
)
add_dependencies(mw_ahrs_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mw_ahrs_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  DIRECTORY launch rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS mw_ahrs_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
