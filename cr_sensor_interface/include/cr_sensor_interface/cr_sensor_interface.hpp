#ifndef CR_SENSOR_INTERFACE_HPP_
#define CR_SENSOR_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <phidget22.h>
#include <stdio.h>

namespace cr_sensor_interface
{
class CrSensorInterface
    : public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
{
public:
    CrSensorInterface();
    ~CrSensorInterface();

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    hardware_interface::return_type start() override;

    hardware_interface::return_type stop() override;

    hardware_interface::return_type read() override;


private:
  // Dummy
  rclcpp::Logger logger_;
  std::chrono::time_point<std::chrono::system_clock> time_;
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_sensor_change_;
  // part for specific sensor data
  // TODO: what kind of data? where I can get the real data? Which data type?
	//Declare your Phidget channels and other variables
	PhidgetVoltageRatioInputHandle voltageRatioInput0;
    PhidgetVoltageRatioInputHandle voltageRatioInput1;
    PhidgetVoltageRatioInputHandle voltageRatioInput2;
    PhidgetVoltageRatioInputHandle voltageRatioInput3;
    PhidgetReturnCode res;

  // part for specific sensor data



  // Store the command for the simulated sensor
  std::vector<double> hw_sensor_states_;
  
};

}  // namespace cr_sensor_interface

#endif  // CR_SENSOR_INTERFACE_HPP_
