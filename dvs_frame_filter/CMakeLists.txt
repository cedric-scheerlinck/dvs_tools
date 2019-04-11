cmake_minimum_required(VERSION 2.8.3)
project(dvs_frame_filter)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# make the executable
cs_add_executable(dvs_frame_filter
  src/frame_filter.cpp
)

# link the executable to the necesarry libs
target_link_libraries(dvs_frame_filter
   ${catkin_LIBRARIES}
)

cs_install()

