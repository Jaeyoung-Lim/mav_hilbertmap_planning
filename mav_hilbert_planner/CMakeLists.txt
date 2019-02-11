cmake_minimum_required(VERSION 2.8.3)
project(mav_hilbert_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/mav_hilbert_planner.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(mav_hilbert_planner_node
  src/mav_hilbert_planner_node.cpp
)
target_link_libraries(mav_hilbert_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
