cmake_minimum_required(VERSION 2.8.3)
project(rbvogui_pad)

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
#   robotnik_msgs sensor_msgs rbvogui_pad
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

add_executable(rbvogui_pad src/rbvogui_pad.cpp)

target_link_libraries(rbvogui_pad
   ${catkin_LIBRARIES}
 )

