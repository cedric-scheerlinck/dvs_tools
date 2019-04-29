cmake_minimum_required(VERSION 2.8.3)
project(dvs_crop_msg)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# make the executable
cs_add_executable(dvs_crop_msg
  src/crop_msg.cpp
)

# link the executable to the necesarry libs
target_link_libraries(dvs_crop_msg
   ${catkin_LIBRARIES}
)

cs_install()

