cmake_minimum_required(VERSION 3.5)
project(cr_sensor_interface)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cr_inverse_kin REQUIRED)
find_package(phidgets_drivers REQUIRED)
find_package(libphidget22 REQUIRED)
find_package(phidgets_api REQUIRED)


add_library(cr_sensor_interface SHARED src/cr_sensor_interface.cpp )
target_include_directories(
        cr_sensor_interface
        PRIVATE
        include
)
target_link_libraries(cr_sensor_interface cr_inverse_kin::lib_cr_fk)
ament_target_dependencies( cr_sensor_interface
        cr_inverse_kin
        hardware_interface
        controller_manager
        serial
        rclcpp
        std_msgs
        pluginlib
        rclcpp_lifecycle
        phidgets_drivers
        libphidget22
        phidgets_api
)
pluginlib_export_plugin_description_file(hardware_interface cr_sensor_interface.xml)


install(
        TARGETS cr_sensor_interface
        DESTINATION lib
)

ament_export_libraries(
        cr_sensor_interface
)
ament_export_dependencies(
        hardware_interface
        cr_inverse_kin
        controller_manager
        serial
        rclcpp
        std_msgs
        pluginlib
        rclcpp_lifecycle
        phidgets_drivers
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(
        DIRECTORY launch config urdf meshes
        DESTINATION share/${PROJECT_NAME}
)
ament_package()
