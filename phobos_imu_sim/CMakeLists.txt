cmake_minimum_required(VERSION 2.8.3)
project(phobos_imu_sim)

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

add_executable(imu_sim_node
    src/imu_sim_node/Main.cpp)
target_link_libraries(imu_sim_node ${catkin_LIBRARIES})
add_dependencies(imu_sim_node ${phobos_imu_sim}_gencfg)
