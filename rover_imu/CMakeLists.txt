cmake_minimum_required(VERSION 2.8.3)
project(rover_imu)

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

add_executable(imu_node
    src/imu_node/Main.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES})
add_dependencies(imu_node ${rover_imu}_gencfg)
