cmake_minimum_required(VERSION 2.8.3)
project(dvs_reverse_events)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

cs_add_library(${PROJECT_NAME} src/utils.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(reverse_events src/reverse_events.cpp)
target_link_libraries(reverse_events ${PROJECT_NAME})

cs_install()
