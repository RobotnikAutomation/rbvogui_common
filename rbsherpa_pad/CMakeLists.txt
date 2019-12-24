cmake_minimum_required(VERSION 2.8.3)
project(rbsherpa_pad)

find_package(catkin REQUIRED COMPONENTS 
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    diagnostic_updater
    )
    
#generate_messages(
#   DEPENDENCIES
#   robotnik_msgs sensor_msgs rbsherpa_pad
# )

catkin_package(
  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

add_executable(rbsherpa_pad src/rbsherpa_pad.cpp)

target_link_libraries(rbsherpa_pad
   ${catkin_LIBRARIES}
 )

