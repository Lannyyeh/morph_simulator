cmake_minimum_required(VERSION 2.8.3)
project(morph_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES smc_position_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(smc_position_controller
  src/library/smc_position_controller.cpp
)

target_link_libraries(smc_position_controller ${catkin_LIBRARIES})
add_dependencies(smc_position_controller ${catkin_EXPORTED_TARGETS})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
add_executable(publish_position_node src/nodes/publish_position_node.cpp)

add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(position_controller_node
  smc_position_controller ${catkin_LIBRARIES})

target_link_libraries(publish_position_node ${catkin_LIBRARIES})


install(TARGETS smc_position_controller 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS position_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
