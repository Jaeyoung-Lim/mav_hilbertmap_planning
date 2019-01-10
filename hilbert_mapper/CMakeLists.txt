cmake_minimum_required(VERSION 2.8.3)
project(hilbert_mapper)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        pcl_ros
)
#catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
        include
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

############
# BINARIES #
############
add_executable(hilbert_mapper
        src/hilbert_mapper.cpp src/hilbert_mapper_node.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)
##########
# EXPORT #
##########
#cs_install()
#cs_export()