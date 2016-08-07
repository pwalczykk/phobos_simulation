# phobos_simulation

Simulation of rover with Gazebo 5.0 and ROS Jade

    # phobos_meshes

    meshes, textures and .world files used to create simulation enviroment

    # phobos_simulation

    simulation startup:

        start ROS:
            $ roscore

        start simulation server (alternative)  
            $ rosrun gazebo_ros gzserver ~/Rover/phobos_simulation/src/mars1.world
            $ rosrun gazebo_ros gzserver

        start simulation GUI (visualization and simulation parameters control)
            $ rosrun gazebo_ros gzclient

        spawn rover:
            $ roslaunch phobos_simulation kongres_rover_spawn.launch

        spawn stones: (dont do it, if u dont have supercomputer :)
            $ roslaunch phobos_simulation kongres_stones_spawn.launch

        wheel controller subscribing to /rover/cmd_vel and video streaming via rosbridge
            $ roslaunch phobos_simulation kongres_control_basic.launch

        teleoperation via keyboard (details and keys in phobos_control/src/READMIE.md)
            $ roslaunch phobos_simulation kongres_control_keyboard.launch

        teleoperation via mini-manipulator
            $ roslaunch phobos_simulation kongres_control_manipulator.launch
