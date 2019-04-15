cmake_minimum_required(VERSION 2.8.3)
project(hilbert_planning_benchmark)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# LIBRARIES #
############
cs_add_library(${PROJECT_NAME}
        src/trajectory_plotter.cpp
        src/local_hilbert_planning_benchmark.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
############
# BINARIES #
############
cs_add_executable(trajectory_plotter_node
        src/trajectory_plotter_node.cpp
        )
target_link_libraries(trajectory_plotter_node ${PROJECT_NAME})

cs_add_executable(local_hilbert_planning_benchmark_node
        src/local_hilbert_planning_benchmark_node.cpp
        )
target_link_libraries(local_hilbert_planning_benchmark_node ${PROJECT_NAME})

##########
cs_install()
cs_export()