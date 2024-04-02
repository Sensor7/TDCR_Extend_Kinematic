# SensorInterface Package #

## Task ##
This package is used to create a framework for the ros2_control sensor interface. The Aim of this repostory is to write a framework for ros2_control sensor interface, which could get sensor raw data and transmit to hardware interface.

## Use ##
By launching the package, the packages will read the sensor data, which now the example is phidgetBridge for loadcell voltage_ratio value
By adapting it to your own sensor, need to replace the senser data, and include the .cpp and .h file to ros2_control system interface, and write a coresponding urdf which contains not only the robot hardwares, but also the sensors your use.


## Requirements ##
### Required HW: ### 
- 1046 PhidgetBridge


### Required SW: ###
- Computer running Ubuntu 20.04 and ROS2 foxy sourced, make sure ros2-control and ros2-controllers are installed 
- Make sure that your user has the rights to access the USB port (dial out user group)
- Required packages to read sensor data: [libphidget22](only the libphidget22 is enough), [phidgets_lib]


If ros2-control, ros2-controllers are not installed, the packets can be installed with:

    sudo apt install ros-foxy-ros2-control
    sudo apt install ros-foxy-ros2-controllers

If the phidgets22 lib is not installed, the libraris can be installed with:

    curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash - &&\sudo apt-get install -y libphidget22
    
If a ROS2 workspace does not yet exist, run the following command:

    mkdir -p ~/foxy_ws/src

Source ROS and the workspace:

    source /opt/ros/foxy/setup.bash
    source ~/ros2_foxy/ros2-linux/setup.bash
    source ~/foxy_ws/install/setup.bash

Clone the required packages into your workspace:

    cd ~/foxy_ws/src
    git clone -b foxy https://github.com/ros-drivers/phidgets_drivers.git
    git clone https://git.ipr.iar.kit.edu/student-work/tendon-robot/cr_sensor_interface.git
    
Build the packages

    cd ..
    colcon build --packages-select serial libphidget22
    colcon build --packages-select cr_sensor_interface

Source the workspace

    source ~/foxy_ws/install/setup.bash

To launch the controllers with one command:

    ros2 launch cr_sensor_interface Sensor.launch.py




[libphidget22]: https://github.com/ros-drivers/phidgets_drivers/tree/foxy
[phidgets_lib]: https://www.phidgets.com/docs/OS_-_Linux
[cr_sensor_interface]: https://git.ipr.iar.kit.edu/student-work/tendon-robot/cr_sensor_interface
