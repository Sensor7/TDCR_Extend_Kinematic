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
    // Get phidgetBridge serial number
    robot1_load_cell1_serialNum = std::stod(info_.hardware_parameters["robot1_load_cell1_serialNum"]);
    robot1_load_cell2_serialNum = std::stod(info_.hardware_parameters["robot1_load_cell1_serialNum"]);
    robot2_load_cell1_serialNum = std::stod(info_.hardware_parameters["robot2_load_cell1_serialNum"]);
    robot2_load_cell2_serialNum = std::stod(info_.hardware_parameters["robot2_load_cell2_serialNum"]);
    hw_sensor_states0_.resize(
      info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
    hw_sensor_states1_.resize(
      info_.sensors[1].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
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
        info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states0_[i]));
    }
    for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &hw_sensor_states1_[i]));
    }
    
    return state_interfaces;
}

hardware_interface::return_type CrSensorInterface::start()
{
  RCLCPP_INFO(
  logger_, "Activating ...please wait...");
  // for sensor init
    //Create your Phidget channels
  // //Create your Phidget channels, segment 1 
  // PhidgetVoltageRatioInput_create(&voltageRatioInput10);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput11);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput12);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput13);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput10, robot1_load_cell1_serialNum);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput11, robot1_load_cell1_serialNum);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput12, robot1_load_cell1_serialNum);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput13, robot1_load_cell1_serialNum);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput10, 0);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput11, 1);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput12, 2);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput13, 3);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput10, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput11, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput12, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput13, 5000);

  // //create your Phidget channels, segment 2
  // PhidgetVoltageRatioInput_create(&voltageRatioInput20);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput21);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput22);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput23);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput20, robot1_load_cell2_serialNum);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput21, robot1_load_cell2_serialNum);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput22, robot1_load_cell2_serialNum);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput23, robot1_load_cell2_serialNum);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput20, 0);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput21, 1);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput22, 2);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput23, 3);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput20, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput21, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput22, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput23, 5000);


  //Create your Phidget channels, segment 1 
  PhidgetVoltageRatioInput_create(&voltageRatioInput10);
  PhidgetVoltageRatioInput_create(&voltageRatioInput11);
  PhidgetVoltageRatioInput_create(&voltageRatioInput12);
  PhidgetVoltageRatioInput_create(&voltageRatioInput13);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput10, robot2_load_cell1_serialNum);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput11, robot2_load_cell1_serialNum);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput12, robot2_load_cell1_serialNum);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput13, robot2_load_cell1_serialNum);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput10, 0);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput11, 1);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput12, 2);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput13, 3);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput10, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput11, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput12, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput13, 5000);

  //create your Phidget channels, segment 2
  PhidgetVoltageRatioInput_create(&voltageRatioInput20);
  PhidgetVoltageRatioInput_create(&voltageRatioInput21);
  PhidgetVoltageRatioInput_create(&voltageRatioInput22);
  PhidgetVoltageRatioInput_create(&voltageRatioInput23);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput20, robot2_load_cell2_serialNum);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput21, robot2_load_cell2_serialNum);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput22, robot2_load_cell2_serialNum);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput23, robot2_load_cell2_serialNum);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput20, 0);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput21, 1);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput22, 2);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput23, 3);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput20, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput21, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput22, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput23, 5000);
	  RCLCPP_INFO(
  logger_, "sucessfully start");
  // for sensor init
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CrSensorInterface::stop()
{
  RCLCPP_INFO(logger_, "Stopping Sensor reader...");
  // lines for sensor deactivate, segment1
  Phidget_close((PhidgetHandle)voltageRatioInput10);
  Phidget_close((PhidgetHandle)voltageRatioInput11);
  Phidget_close((PhidgetHandle)voltageRatioInput12);
  Phidget_close((PhidgetHandle)voltageRatioInput13);

  PhidgetVoltageRatioInput_delete(&voltageRatioInput10);
  PhidgetVoltageRatioInput_delete(&voltageRatioInput11);
  PhidgetVoltageRatioInput_delete(&voltageRatioInput12);
  PhidgetVoltageRatioInput_delete(&voltageRatioInput13);

  //lines for sensor deactivate, segment2
  Phidget_close((PhidgetHandle)voltageRatioInput20);
  Phidget_close((PhidgetHandle)voltageRatioInput21);
  Phidget_close((PhidgetHandle)voltageRatioInput22);
  Phidget_close((PhidgetHandle)voltageRatioInput23);

  PhidgetVoltageRatioInput_delete(&voltageRatioInput20);
  PhidgetVoltageRatioInput_delete(&voltageRatioInput21);
  PhidgetVoltageRatioInput_delete(&voltageRatioInput22);
  PhidgetVoltageRatioInput_delete(&voltageRatioInput23);
  
  // lines for sensor deactivate
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CrSensorInterface::read()
{
    // RCLCPP_INFO(logger_, "Reading...please wait...");
    std::vector<double> voltageRatios;

    voltageRatios.push_back(0.0); // voltageRatio0
    voltageRatios.push_back(0.0); // voltageRatio1
    voltageRatios.push_back(0.0); // voltageRatio2
    voltageRatios.push_back(0.0); // voltageRatio3
    voltageRatios.push_back(0.0); // voltageRatio4
    voltageRatios.push_back(0.0); // voltageRatio5
    voltageRatios.push_back(0.0); // voltageRatio6
    voltageRatios.push_back(0.0); // voltageRatio7
    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput10, &voltageRatios[0]);
    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput11, &voltageRatios[1]);
    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput12, &voltageRatios[2]);
    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput13, &voltageRatios[3]);

    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput20, &voltageRatios[4]);
    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput21, &voltageRatios[5]);
    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput22, &voltageRatios[6]);
    PhidgetVoltageRatioInput_getVoltageRatio(voltageRatioInput23, &voltageRatios[7]);
  // this part for reading
    for (uint i = 0; i < 4; i++)
  {
    hw_sensor_states0_[i] = voltageRatios[i];
    RCLCPP_INFO(
    logger_, "Got state %.8f for interface %s!",
    hw_sensor_states0_[i], info_.sensors[0].state_interfaces[i].name.c_str());
  }
    for (uint i = 4; i < 8; i++)
  {
    hw_sensor_states1_[i-4] = voltageRatios[i];
    RCLCPP_INFO(
    logger_, "Got state %.8f for interface %s!",
    hw_sensor_states1_[i-4], info_.sensors[1].state_interfaces[i-4].name.c_str());
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
