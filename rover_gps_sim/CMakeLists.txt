cmake_minimum_required(VERSION 2.8.3)
project(rover_gps_sim)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/DRNoiseGen.cfg
)

catkin_package()

add_executable(gps_sim_node
    src/gps_sim_node/Main.cpp)
target_link_libraries(gps_sim_node ${catkin_LIBRARIES})
add_dependencies(gps_sim_node ${rover_gps_sim}_gencfg)
