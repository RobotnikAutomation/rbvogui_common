cmake_minimum_required(VERSION 2.8.3)
project(rbsherpa_hl_pad)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    diagnostic_updater
    )
    
#generate_messages(
#   DEPENDENCIES
#   robotnik_msgs sensor_msgs rbsherpa_hl_pad
# )

catkin_package(
  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

add_executable(rbsherpa_hl_pad src/rbsherpa_hl_pad.cpp)

add_dependencies(rbsherpa_hl_pad rbsherpa_hl_pad_generate_messages_cpp)

target_link_libraries(rbsherpa_hl_pad
   ${catkin_LIBRARIES}
 )

