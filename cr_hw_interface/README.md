# HardwareInterface Package #

## Task ##
This package is used to implement the Static Model Kinematics for the [tendon driven robot], which was designed by the HERA workgroup in the Intelligent Process Automation and Robotics Lab (IPR)
at the Karlsruher Institute for Technology (KIT). 

## Use ##
By launching the package, the ros2_control hardware interface will return the static model predicted pose of the TCP(xyz + beta(Rotation around global Z-Axis) alpha(tcp y rotation) gamma(Rotation around local (tcp) Z-Axis))
Sending the messages to the topic '/forward_command_controller/commands' could change the position of the robot
The changing of the tcp position can be seen in rviz2


## Requirements ##
### Required HW: ### 
- [Tendon driven robot] 


### Required SW: ###
- Computer running Ubuntu 20.04 and ROS2 foxy sourced, make sure ros2-control and ros2-controllers are installed 
- Arduino flashed with the [Robot Arduino Driver] 
- Required packages to control robot: [cr_hw_interface], [cr_inverse_kin], [serial], [forward_command_pub], [cr_sensor_interface]
- Make sure that your user has the rights to access the USB port (dial out user group)
- Required packages to read sensor data: [libphidget22](only the libphidget22 is enough), [phidgets_lib]
- Required libraries to solve numerically non linear problem: [Eigen], [GSL:GNU]
- The load cells can be read with the tns and the [phidgets_drivers] package and calibrated with the [load-cell-calibrator] package. This is currently limited to ROS1  


git clone git@git.ipr.iar.kit.edu:student-work/tendon-robot/cr_hw_interface.git
cd cr_hw_interface/
git checkout CosseratRod
cd ..
git clone https://github.com/joshnewans/serial.git

If ros2-control, ros2-controllers are not installed, the packets can be installed with:

    sudo apt install ros-foxy-ros2-control
    sudo apt install ros-foxy-ros2-controllers

If the phidgets22 lib, eigen or gsl:gnu library are not installed, the libraris can be installed with:

    curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash - &&\sudo apt-get install -y libphidget22
    sudo apt install libeigen3-dev
    sudo apt-get install libgsl-dev
    
If a ROS2 workspace does not yet exist, run the following command:

    mkdir -p ~/foxy_ws/src

Source ROS and the workspace:

    source /opt/ros/foxy/setup.bash
    source ~/ros2_foxy/ros2-linux/setup.bash
    source ~/foxy_ws/install/setup.bash

Clone the required packages into your workspace:

    cd ~/foxy_ws/src
    git clone -b CosseratRod https://git.ipr.iar.kit.edu/student-work/tendon-robot/cr_hw_interface.git
    git clone -b ForwardKin https://git.ipr.iar.kit.edu/student-work/tendon-robot/cr_inverse_kinematics.git
    git clone https://github.com/joshnewans/serial.git
    git clone -b foxy https://github.com/ros-drivers/phidgets_drivers.git
    git clone https://git.ipr.iar.kit.edu/student-work/tendon-robot/cr_sensor_interface.git
    git clone https://git.ipr.iar.kit.edu/student-work/tendon-robot/forward_commmand_publisher.git
    
Build the packages

    cd ..
    colcon build --packages-select serial libphidget22 forward_command_pub
    colcon build --packages-select cr_inverse_kin cr_hw_interface cr_sensor_interface

Source the workspace

    source ~/foxy_ws/install/setup.bash

To launch the controllers with one command:

    ros2 launch cr_hw_interface FWD.launch.py

To view the visualisation of the robots digital twin open rviz2 in a new terminal and follow these instructions:

    source /opt/ros/foxy/setup.bash
    source ~/ros2_foxy/ros2-linux/setup.bash
    source ~/foxy_ws/install/setup.bash
    rviz2

1. *If the robot model geometry is not shown, change the variable in this bash with:*

        export LC_NUMERIC="en_US.UTF-8" 

2. Add tf tree
3. Set Global Options FixedFrame to “world”
4. Add robot model
5. Save config, so you only need to set up once


[tendon driven robot]: https://github.com/ChristianMarzi/Tendon-Driven-Continuum-Robot
[Robot Arduino Driver]: https://gitlab.ipr.iar.kit.edu/student-work/tendon-robot/robot-arduino-driver
[cr_hw_interface]: https://gitlab.ipr.iar.kit.edu/student-work/tendon-robot/cr_hw_interface
[cr_inverse_kin]: https://gitlab.ipr.iar.kit.edu/student-work/tendon-robot/cr_inverse_kinematics
[serial]: https://github.com/joshnewans/serial
[libphidget22]: https://github.com/ros-drivers/phidgets_drivers/tree/foxy
[phidgets_lib]: https://www.phidgets.com/docs/OS_-_Linux
[Eigen]: https://robots.uc3m.es/installation-guides/install-eigen.html
[GSL:GNU]: https://gist.github.com/TysonRayJones/af7bedcdb8dc59868c7966232b4da903#ubuntu
[load-cell-calibrator]: https://git.ipr.iar.kit.edu/cathera/load_cell_calibrator
[phidgets_drivers]: https://gitlab.ipr.kit.edu/cathera/phidgets_drivers
[forward_command_pub]: https://git.ipr.iar.kit.edu/student-work/tendon-robot/forward_commmand_publisher
[cr_sensor_interface]: https://git.ipr.iar.kit.edu/student-work/tendon-robot/cr_sensor_interface
