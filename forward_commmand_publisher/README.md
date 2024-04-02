# Forward Command Publisher package #

## Task ##
This package is used to publish a corresponding message for the [tendon driven robot], which was designed by the HERA workgroup in the Intelligent Process Automation and Robotics Lab (IPR) at the Karlsruher Institute for Technology (KIT). 

## Use ##
If a ROS2 workspace does not yet exist, run the following command:

    mkdir -p ~/foxy_ws/src

Source ROS and the workspace:

    source /opt/ros/foxy/setup.bash
    source ~/ros2_foxy/ros2-linux/setup.bash
    source ~/foxy_ws/install/setup.bash
Clone the required packages into your workspace:

    cd ~/foxy_ws/src
    git clone https://git.ipr.iar.kit.edu/student-work/tendon-robot/forward_commmand_publisher.git
Build the packages

    cd ..
    colcon build --packages-select forward_command_pub
Source the workspace

    source ~/foxy_ws/install/setup.bash

To run the command publisher:

    ros2 run forward_command_pub sample_point

## Requirements ##
### Required HW: ### 
- [Tendon driven robot] 


### Required SW: ###
- Computer running Ubuntu 20.04 and ROS2 foxy sourced, make sure joy, ros2-control and ros2-controllers are installed 
- There is already forwardcommand topic 