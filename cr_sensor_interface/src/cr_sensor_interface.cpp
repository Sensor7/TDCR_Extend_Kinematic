#include "cr_sensor_interface/cr_sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cstring>
#include <math.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"



using namespace cr_sensor_interface;

CrSensorInterface::CrSensorInterface()
    : logger_(rclcpp::get_logger("CrSensorInterface"))
{}

CrSensorInterface::~CrSensorInterface()
{
    RCLCPP_INFO(logger_, "finish now");
}

hardware_interface::return_type CrSensorInterface::configure(const hardware_interface::HardwareInfo & info)
{

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

    time_ = std::chrono::system_clock::now();
    RCLCPP_INFO(logger_, "Sensor Hardware Configuring...");
    // Sensor parameter setup
    // TODO: add real sensor data
    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    hw_sensor_change_ = std::stod(info_.hardware_parameters["example_param_max_sensor_change"]);
    hw_sensor_states_.resize(
      info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
    PhidgetLog_enable(PHIDGET_LOG_INFO, "phidgetlog.log");
    //done
    RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> CrSensorInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
    }
    
    return state_interfaces;
}

hardware_interface::return_type CrSensorInterface::start()
{
  RCLCPP_INFO(
  logger_, "Activating ...please wait...");
  // for sensor init
    if (std::isnan(hw_sensor_states_[0]))
    {
        hw_sensor_states_[0] = 0;
    }
    //Create your Phidget channels
    PhidgetVoltageRatioInput_create(&voltageRatioInput0);
    PhidgetVoltageRatioInput_create(&voltageRatioInput1);
    PhidgetVoltageRatioInput_create(&voltageRatioInput2);
    PhidgetVoltageRatioInput_create(&voltageRatioInput3);
    Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput0, 566303);
    Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput1, 566303);
    Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput2, 566303);
    Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput3, 566303);
    Phidget_setChannel((PhidgetHandle)voltageRatioInput0, 0);
    Phidget_setChannel((PhidgetHandle)voltageRatioInput1, 1);
    Phidget_setChannel((PhidgetHandle)voltageRatioInput2, 2);
    Phidget_setChannel((PhidgetHandle)voltageRatioInput3, 3);
    Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput0, 5000);
    Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput1, 5000);
    Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput2, 5000);
    Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput3, 5000);
	  RCLCPP_INFO(
  logger_, "sucessfully start");
  // for sensor init
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CrSensorInterface::stop()
{
  RCLCPP_INFO(logger_, "Stopping Sensor reader...");
  // lines for sensor deactivate
	  Phidget_close((PhidgetHandle)voltageRatioInput0);
    Phidget_close((PhidgetHandle)voltageRatioInput1);
    Phidget_close((PhidgetHandle)voltageRatioInput2);
    Phidget_close((PhidgetHandle)voltageRatioInput3);

	  PhidgetVoltageRatioInput_delete(&voltageRatioInput0);
    PhidgetVoltageRatioInput_delete(&voltageRatioInput1);
    PhidgetVoltageRatioInput_delete(&voltageRatioInput2);
    PhidgetVoltageRatioInput_delete(&voltageRatioInput3);
  
  // lines for sensor deactivate
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CrSensorInterface::read()
{
    // RCLCPP_INFO(logger_, "Reading...please wait...");
    std::vector<double> voltageRatios;
    std::vector<PhidgetVoltageRatioInputHandle> voltageRatioInputs;
    voltageRatioInputs.push_back(voltageRatioInput0);
    voltageRatioInputs.push_back(voltageRatioInput1);
    voltageRatioInputs.push_back(voltageRatioInput2);
    voltageRatioInputs.push_back(voltageRatioInput3);
    voltageRatios.push_back(0.0); // voltageRatio0
    voltageRatios.push_back(0.0); // voltageRatio1
    voltageRatios.push_back(0.0); // voltageRatio2
    voltageRatios.push_back(0.0); // voltageRatio3
  // this part for reading
    for (uint i = 0; i < 4; i++)
  {
    // generate random sensor data
    // TODO: how to get real sensor data?
      res = PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInputs[i], &voltageRatios[i]);
    hw_sensor_states_[i] = voltageRatios[i];
      RCLCPP_INFO(
      logger_, "Got state %e for interface %s!",
      hw_sensor_states_[i], info_.sensors[0].state_interfaces[i].name.c_str());
  }
    // RCLCPP_INFO(logger_, "Sensors successfully read!");
  // this part for reading
  
    return hardware_interface::return_type::OK;
}



// -------    PLUGIN    --------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    cr_sensor_interface::CrSensorInterface,
    hardware_interface::SensorInterface
)
