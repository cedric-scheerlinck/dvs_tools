cmake_minimum_required(VERSION 2.8.3)
project(dvs_hot_pixel_filter)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

cs_add_library(${PROJECT_NAME} src/utils.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(hot_pixel_filter src/hot_pixel_filter.cpp)
target_link_libraries(hot_pixel_filter ${PROJECT_NAME})

cs_install()
