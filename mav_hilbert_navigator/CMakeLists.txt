cmake_minimum_required(VERSION 2.8.3)
project(mav_hilbert_navigator)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############
cs_add_executable(mav_hilbert_navigator
        src/mav_hilbert_navigator.cpp src/mav_hilbert_navigator_node.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
##########