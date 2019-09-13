cmake_minimum_required(VERSION 2.8.3)
project(ft_calibration)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interfaces
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# find_library(TIMER_LIB TimerLinux HINTS /usr/local/lib/RobotUtilities)
find_library(RUT Utilities HINTS /usr/local/lib/RobotUtilities)

catkin_package()

###########
## Build ##
###########
add_executable(ft_calibration_node src/calibration.cpp)
target_link_libraries(ft_calibration_node
    ${catkin_LIBRARIES}
    ${RUT}
)
