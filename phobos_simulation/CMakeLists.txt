cmake_minimum_required(VERSION 2.8.3)
project(phobos_simulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    dynamic_reconfigure
    tf
    phobos_shared
)

generate_dynamic_reconfigure_options(
  cfg/ControllerServer.cfg
  cfg/ArmJointDRC.cfg
  cfg/ImuNoiseGen.cfg
)

catkin_package()

add_executable(controller_server
    src/nodes/controller_server/ControllerServerMain.cpp
    src/nodes/controller_server/ControllerServer.cpp)
target_link_libraries(controller_server ${catkin_LIBRARIES})
add_dependencies(controller_server ${phobos_simulation}_gencfg)

add_executable(gazebo_wheel_controller
    src/nodes/gazebo_wheel_controller/src/Main.cpp
    src/nodes/gazebo_wheel_controller/src/BaseWheelController.cpp)
target_link_libraries(gazebo_wheel_controller ${catkin_LIBRARIES})

add_executable(gazebo_arm_controller
    src/nodes/gazebo_arm_controller/src/Main.cpp
    src/nodes/gazebo_arm_controller/src/ArmJointsController.cpp)
target_link_libraries(gazebo_arm_controller ${catkin_LIBRARIES})
add_dependencies(gazebo_arm_controller ${phobos_simulation}_gencfg)

add_executable(gazebo_joint_publisher
    src/nodes/gazebo_joint_publisher/Main.cpp)
target_link_libraries(gazebo_joint_publisher ${catkin_LIBRARIES})

add_executable(gazebo_joint_dynamometer_recorder
    src/nodes/gazebo_joint_dynamometer_recorder/src/Main.cpp)
target_link_libraries(gazebo_joint_dynamometer_recorder ${catkin_LIBRARIES})

add_executable(gazebo_uart433_controler
    src/nodes/gazebo_uart433_controler/Main.cpp)
add_dependencies(gazebo_uart433_controler ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_uart433_controler ${catkin_LIBRARIES})

add_executable(gazebo_motor_transceiver
    src/nodes/gazebo_motor_transceiver/Main.cpp)
target_link_libraries(gazebo_motor_transceiver ${catkin_LIBRARIES})

add_executable(gazebo_imu_handler
    src/nodes/gazebo_imu_handler/Main.cpp)
target_link_libraries(gazebo_imu_handler ${catkin_LIBRARIES})
add_dependencies(gazebo_imu_handler ${phobos_simulation}_gencfg)

add_executable(uart_433_simulation_bridge
    src/nodes/uart_433_simulation_bridge/Main.cpp)
target_link_libraries(uart_433_simulation_bridge ${catkin_LIBRARIES})

add_executable(keyboard_teleop
    src/nodes/keyboard_teleop/src/Main.cpp
    src/nodes/keyboard_teleop/src/KeyListener.cpp
    src/nodes/keyboard_teleop/src/PublisherFloat64.cpp
    src/nodes/keyboard_teleop/src/PublisherTwist.cpp)
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11")

add_library(JointEncoder SHARED
    src/plugins/joint_encoder/JointEncoder.cc)
target_link_libraries(JointEncoder ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_library(JointDynamometer SHARED
    src/plugins/joint_dynamometer/JointDynamometer.cc)
target_link_libraries(JointDynamometer ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_library(JointLimiter SHARED
    src/plugins/joint_limiter/JointLimiter.cc)
target_link_libraries(JointLimiter ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_library(DifferentialPID SHARED
    src/plugins/differential_pid/DifferentialPID.cc)
target_link_libraries(DifferentialPID ${GAZEBO_libraries} ${roscpp_LIBRARIES})

add_library(ControllerPID SHARED
    src/plugins/controller_pid/ControllerPlugin.cc)
target_link_libraries(ControllerPID ${GAZEBO_libraries} ${roscpp_LIBRARIES})
