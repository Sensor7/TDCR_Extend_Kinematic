#ifndef CR_HW_INTERFACE_CR_SERVO_H
#define CR_HW_INTERFACE_CR_SERVO_H

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include <libphidget22/phidget22.h>

#include "config.h"
#include "Tendondrum.h"
#include "spacer.h"
#include "ForwardKinematics.hpp"
#include "arduino_comms.h"
#include "light_sub.hpp"
#include "piecewiseconstantcurvaturemodel.h"




namespace cr_hw_interface
{
class CrHwInterface
        : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
    CrHwInterface();
    ~CrHwInterface();

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type start() override;
    hardware_interface::return_type stop() override;
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;
    double do_calibrate(double voltageRatio,double p1,double p2);

private:
    std::shared_ptr<light_sub::CrCommunication> comms;
    rclcpp::executors::SingleThreadedExecutor executor_;  //Executor needed to subscriber
    rclcpp::Logger logger_;
    std::chrono::time_point<std::chrono::system_clock> time_;

    ArduinoComms arduino_;
    Config cfg_;
    ForwardKinematics cr_model_;
    PiecewiseConstantCurvatureModel* pcc_model_;
    
    Tendondrum xz1_servo_;
    Tendondrum yz1_servo_;
    Tendondrum xz2_servo_;
    Tendondrum yz2_servo_;

    Spacer sensorjointx_;
    Spacer sensorjointy_;
    Spacer sensorjointz_;
    Spacer sensorjoint_beta_;
    Spacer sensorjoint_alpha_;
    Spacer sensorjoint_gamma_;

    // disks for display in rviz2
    Spacer disk1_4jointx_;
    Spacer disk1_4jointy_;
    Spacer disk1_4jointz_;
    Spacer disk1_4joint_beta_;
    Spacer disk1_4joint_alpha_;
    Spacer disk1_4joint_gamma_;
    Spacer disk1_8jointx_;
    Spacer disk1_8jointy_;
    Spacer disk1_8jointz_;
    Spacer disk1_8joint_beta_;
    Spacer disk1_8joint_alpha_;
    Spacer disk1_8joint_gamma_;
    Spacer disk2_4jointx_;
    Spacer disk2_4jointy_;
    Spacer disk2_4jointz_;
    Spacer disk2_4joint_beta_;
    Spacer disk2_4joint_alpha_;
    Spacer disk2_4joint_gamma_;

    Spacer spacer1_1x_;
    Spacer spacer1_1y_;
    Spacer spacer1_1z_;
    Spacer spacer1_2x_;
    Spacer spacer1_2y_;
    Spacer spacer1_2z_;
    Spacer spacer1_3x_;
    Spacer spacer1_3y_;
    Spacer spacer1_3z_;
    Spacer spacer1_4x_;
    Spacer spacer1_4y_;
    Spacer spacer1_4z_;
    Spacer spacer1_5x_;
    Spacer spacer1_5y_;
    Spacer spacer1_5z_;
    Spacer spacer1_6x_;
    Spacer spacer1_6y_;
    Spacer spacer1_6z_;
    Spacer spacer1_7x_;
    Spacer spacer1_7y_;
    Spacer spacer1_7z_;
    Spacer spacer1_8x_;
    Spacer spacer1_8y_;
    Spacer spacer1_8z_;

    Spacer spacer2_1x_;
    Spacer spacer2_1y_;
    Spacer spacer2_1z_;
    Spacer spacer2_2x_;
    Spacer spacer2_2y_;
    Spacer spacer2_2z_;
    Spacer spacer2_3x_;
    Spacer spacer2_3y_;
    Spacer spacer2_3z_;
    Spacer spacer2_4x_;
    Spacer spacer2_4y_;
    Spacer spacer2_4z_;
    Spacer spacer2_5x_;
    Spacer spacer2_5y_;
    Spacer spacer2_5z_;
    Spacer spacer2_6x_;
    Spacer spacer2_6y_;
    Spacer spacer2_6z_;
    Spacer spacer2_7x_;
    Spacer spacer2_7y_;
    Spacer spacer2_7z_;

    std::array<double,2> m_length;
	double m_youngs_modulus;
	std::vector<Eigen::Vector3d> m_routing;
	std::array<int,2> m_number_disks;
	std::array<double,2> m_pradius_disks;
	double m_ro;
	Eigen::MatrixXd m_disk_frames;
    
    PhidgetVoltageRatioInputHandle voltageRatioInput10;
    PhidgetVoltageRatioInputHandle voltageRatioInput11;
    PhidgetVoltageRatioInputHandle voltageRatioInput12;
    PhidgetVoltageRatioInputHandle voltageRatioInput13;

    PhidgetVoltageRatioInputHandle voltageRatioInput20;
    PhidgetVoltageRatioInputHandle voltageRatioInput21;
    PhidgetVoltageRatioInputHandle voltageRatioInput22;
    PhidgetVoltageRatioInputHandle voltageRatioInput23;
    PhidgetReturnCode res;
    // Store the command for the simulated sensor
};
}

#endif // CR_HW_INTERFACE_CR_SERVO_H