cmake_minimum_required(VERSION 3.5)
project(cr_hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(serial REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cr_inverse_kin REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GSL REQUIRED)  
find_package(libphidget22 REQUIRED)


add_library(cr_hw_interface SHARED src/cr_hw_interface.cpp src/arduino_comms.cpp src/tendondrum.cpp src/spacer.cpp src/light_sub.cpp src/piecewiseconstantcurvaturemodel.cpp)
target_include_directories(
        cr_hw_interface
        PRIVATE
        include
)
target_link_libraries(cr_hw_interface cr_inverse_kin::lib_cr_fk)
ament_target_dependencies( cr_hw_interface
        cr_inverse_kin
        hardware_interface
        controller_manager
        serial
        rclcpp
        std_msgs
        pluginlib
        Eigen3
        GSL
        libphidget22
)
pluginlib_export_plugin_description_file(hardware_interface cr_hw_interface.xml)


install(
        TARGETS cr_hw_interface
        DESTINATION lib
)

ament_export_libraries(
        cr_hw_interface
)
ament_export_dependencies(
        hardware_interface
        cr_inverse_kin
        controller_manager
        serial
        rclcpp
        std_msgs
        pluginlib
        Eigen3
        GSL
        libphidget22
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
        DIRECTORY launch controller urdf meshes
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
