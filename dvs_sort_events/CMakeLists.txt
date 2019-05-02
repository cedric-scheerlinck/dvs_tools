cmake_minimum_required(VERSION 2.8.3)
project(dvs_sort_events)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

cs_add_library(${PROJECT_NAME} src/utils.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# make the executable
cs_add_executable(sort src/sort_events.cpp)
# link the executable to the project lib
target_link_libraries(sort ${PROJECT_NAME})

cs_install()

