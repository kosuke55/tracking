cmake_minimum_required(VERSION 2.8.3)
project(tracking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pedantic -O3 -Wno-long-long -fno-omit-frame-pointer -fpic")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  nodelet
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(${PROJECT_NAME} SHARED
  src/kcf_tracker.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# add_library(kcf_tracker
#   src/kcf_tracker.cpp
# )

# # add_executable(kcf_tracker src/kcf_tracker.cpp)
# target_link_libraries(kcf_tracker
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
