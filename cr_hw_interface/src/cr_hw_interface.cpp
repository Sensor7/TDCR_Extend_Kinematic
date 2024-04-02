#include "cr_hw_interface/cr_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ForwardKinematics.hpp"
#include "cr_hw_interface/piecewiseconstantcurvaturemodel.h"
#include <cstring>
#include <math.h>
#include <chrono>


using namespace cr_hw_interface;

CrHwInterface::CrHwInterface()
    : logger_(rclcpp::get_logger("CrHwInterface"))
{}

CrHwInterface::~CrHwInterface()
{
    arduino_.active(false);
    RCLCPP_INFO(logger_, "Robot sleep now...");
}

hardware_interface::return_type CrHwInterface::configure(const hardware_interface::HardwareInfo & info)
{
    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    time_ = std::chrono::system_clock::now();
    RCLCPP_INFO(logger_, "Configuring...");

    /*
     * Get Parameters from Robot URDF
     */
    // Get Servo Names
    cfg_.xz1_servo_name = info_.hardware_parameters["xz1_servo_name"];
    cfg_.yz1_servo_name = info_.hardware_parameters["yz1_servo_name"];
    cfg_.xz2_servo_name = info_.hardware_parameters["xz2_servo_name"];
    cfg_.yz2_servo_name = info_.hardware_parameters["yz2_servo_name"];

    // Get Spacer Joint Names
    cfg_.spacer1_1x_name = info_.hardware_parameters["spacer1_1x_name"];
    cfg_.spacer1_1y_name = info_.hardware_parameters["spacer1_1y_name"];
    cfg_.spacer1_1z_name = info_.hardware_parameters["spacer1_1z_name"];
    cfg_.spacer1_2x_name = info_.hardware_parameters["spacer1_2x_name"];
    cfg_.spacer1_2y_name = info_.hardware_parameters["spacer1_2y_name"];
    cfg_.spacer1_2z_name = info_.hardware_parameters["spacer1_2z_name"];
    cfg_.spacer1_3x_name = info_.hardware_parameters["spacer1_3x_name"];
    cfg_.spacer1_3y_name = info_.hardware_parameters["spacer1_3y_name"];
    cfg_.spacer1_3z_name = info_.hardware_parameters["spacer1_3z_name"];
    cfg_.spacer1_4x_name = info_.hardware_parameters["spacer1_4x_name"];
    cfg_.spacer1_4y_name = info_.hardware_parameters["spacer1_4y_name"];
    cfg_.spacer1_4z_name = info_.hardware_parameters["spacer1_4z_name"];
    cfg_.spacer1_5x_name = info_.hardware_parameters["spacer1_5x_name"];
    cfg_.spacer1_5y_name = info_.hardware_parameters["spacer1_5y_name"];
    cfg_.spacer1_5z_name = info_.hardware_parameters["spacer1_5z_name"];
    cfg_.spacer1_6x_name = info_.hardware_parameters["spacer1_6x_name"];
    cfg_.spacer1_6y_name = info_.hardware_parameters["spacer1_6y_name"];
    cfg_.spacer1_6z_name = info_.hardware_parameters["spacer1_6z_name"];
    cfg_.spacer1_7x_name = info_.hardware_parameters["spacer1_7x_name"];
    cfg_.spacer1_7y_name = info_.hardware_parameters["spacer1_7y_name"];
    cfg_.spacer1_7z_name = info_.hardware_parameters["spacer1_7z_name"];
    cfg_.spacer1_8x_name = info_.hardware_parameters["spacer1_8x_name"];
    cfg_.spacer1_8y_name = info_.hardware_parameters["spacer1_8y_name"];
    cfg_.spacer1_8z_name = info_.hardware_parameters["spacer1_8z_name"];
    cfg_.spacer2_1x_name = info_.hardware_parameters["spacer2_1x_name"];
    cfg_.spacer2_1y_name = info_.hardware_parameters["spacer2_1y_name"];
    cfg_.spacer2_1z_name = info_.hardware_parameters["spacer2_1z_name"];
    cfg_.spacer2_2x_name = info_.hardware_parameters["spacer2_2x_name"];
    cfg_.spacer2_2y_name = info_.hardware_parameters["spacer2_2y_name"];
    cfg_.spacer2_2z_name = info_.hardware_parameters["spacer2_2z_name"];
    cfg_.spacer2_3x_name = info_.hardware_parameters["spacer2_3x_name"];
    cfg_.spacer2_3y_name = info_.hardware_parameters["spacer2_3y_name"];
    cfg_.spacer2_3z_name = info_.hardware_parameters["spacer2_3z_name"];
    cfg_.spacer2_4x_name = info_.hardware_parameters["spacer2_4x_name"];
    cfg_.spacer2_4y_name = info_.hardware_parameters["spacer2_4y_name"];
    cfg_.spacer2_4z_name = info_.hardware_parameters["spacer2_4z_name"];
    cfg_.spacer2_5x_name = info_.hardware_parameters["spacer2_5x_name"];
    cfg_.spacer2_5y_name = info_.hardware_parameters["spacer2_5y_name"];
    cfg_.spacer2_5z_name = info_.hardware_parameters["spacer2_5z_name"];
    cfg_.spacer2_6x_name = info_.hardware_parameters["spacer2_6x_name"];
    cfg_.spacer2_6y_name = info_.hardware_parameters["spacer2_6y_name"];
    cfg_.spacer2_6z_name = info_.hardware_parameters["spacer2_6z_name"];
    cfg_.spacer2_7x_name = info_.hardware_parameters["spacer2_7x_name"];
    cfg_.spacer2_7y_name = info_.hardware_parameters["spacer2_7y_name"];
    cfg_.spacer2_7z_name = info_.hardware_parameters["spacer2_7z_name"];

    // Get Sensor Joint Name
    cfg_.sensorjointx_name = info_.hardware_parameters["sensorjointx_name"];
    cfg_.sensorjointy_name = info_.hardware_parameters["sensorjointy_name"];
    cfg_.sensorjointz_name = info_.hardware_parameters["sensorjointz_name"];
    cfg_.sensorjoint_beta_name = info_.hardware_parameters["sensorjoint_beta_name"];
    cfg_.sensorjoint_alpha_name = info_.hardware_parameters["sensorjoint_alpha_name"];
    cfg_.sensorjoint_gamma_name = info_.hardware_parameters["sensorjoint_gamma_name"];

    // Get Disk Name
    cfg_.disk1_4jointx_name = info_.hardware_parameters["disk1_4jointx_name"];
    cfg_.disk1_4jointy_name = info_.hardware_parameters["disk1_4jointy_name"];
    cfg_.disk1_4jointz_name = info_.hardware_parameters["disk1_4jointz_name"];
    cfg_.disk1_4joint_beta_name = info_.hardware_parameters["disk1_4joint_beta_name"];
    cfg_.disk1_4joint_alpha_name = info_.hardware_parameters["disk1_4joint_alpha_name"];
    cfg_.disk1_4joint_gamma_name = info_.hardware_parameters["disk1_4joint_gamma_name"];

    cfg_.disk1_8jointx_name = info_.hardware_parameters["disk1_8jointx_name"];
    cfg_.disk1_8jointy_name = info_.hardware_parameters["disk1_8jointy_name"];
    cfg_.disk1_8jointz_name = info_.hardware_parameters["disk1_8jointz_name"];
    cfg_.disk1_8joint_beta_name = info_.hardware_parameters["disk1_8joint_beta_name"];
    cfg_.disk1_8joint_alpha_name = info_.hardware_parameters["disk1_8joint_alpha_name"];
    cfg_.disk1_8joint_gamma_name = info_.hardware_parameters["disk1_8joint_gamma_name"];

    cfg_.disk2_4jointx_name = info_.hardware_parameters["disk2_4jointx_name"];
    cfg_.disk2_4jointy_name = info_.hardware_parameters["disk2_4jointy_name"];
    cfg_.disk2_4jointz_name = info_.hardware_parameters["disk2_4jointz_name"];
    cfg_.disk2_4joint_beta_name = info_.hardware_parameters["disk2_4joint_beta_name"];
    cfg_.disk2_4joint_alpha_name = info_.hardware_parameters["disk2_4joint_alpha_name"];
    cfg_.disk2_4joint_gamma_name = info_.hardware_parameters["disk2_4joint_gamma_name"];

    // Configure Arduino
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout_ms"]);

    // Get Robot Structure
    cfg_.spacer_distance = std::stod(info_.hardware_parameters["spacer_distance"]);
    cfg_.dist_to_tendon = std::stod(info_.hardware_parameters["dist_to_tendon"]);
    cfg_.spacer_num = std::stod(info_.hardware_parameters["spacer_num"]);
    cfg_.segment_num = std::stod(info_.hardware_parameters["segment_num"]);
    cfg_.tendondrum_rad = std::stod(info_.hardware_parameters["tendondrum_rad"]);


    /*
     * Configure Hardware and CR Model
     */
    // Set up the tendons
    xz1_servo_.setup(cfg_.xz1_servo_name);
    yz1_servo_.setup(cfg_.yz1_servo_name);
    xz2_servo_.setup(cfg_.xz2_servo_name);
    yz2_servo_.setup(cfg_.yz2_servo_name);

    // Set up Spacer / Backbone
    spacer1_1x_.setup(cfg_.spacer1_1x_name);
    spacer1_1y_.setup(cfg_.spacer1_1y_name);
    spacer1_1z_.setup(cfg_.spacer1_1z_name);
    spacer1_2x_.setup(cfg_.spacer1_2x_name);
    spacer1_2y_.setup(cfg_.spacer1_2y_name);
    spacer1_2z_.setup(cfg_.spacer1_2z_name);
    spacer1_3x_.setup(cfg_.spacer1_3x_name);
    spacer1_3y_.setup(cfg_.spacer1_3y_name);
    spacer1_3z_.setup(cfg_.spacer1_3z_name);
    spacer1_4x_.setup(cfg_.spacer1_4x_name);
    spacer1_4y_.setup(cfg_.spacer1_4y_name);
    spacer1_4z_.setup(cfg_.spacer1_4z_name);
    spacer1_5x_.setup(cfg_.spacer1_5x_name);
    spacer1_5y_.setup(cfg_.spacer1_5y_name);
    spacer1_5z_.setup(cfg_.spacer1_5z_name);
    spacer1_6x_.setup(cfg_.spacer1_6x_name);
    spacer1_6y_.setup(cfg_.spacer1_6y_name);
    spacer1_6z_.setup(cfg_.spacer1_6z_name);
    spacer1_7x_.setup(cfg_.spacer1_7x_name);
    spacer1_7y_.setup(cfg_.spacer1_7y_name);
    spacer1_7z_.setup(cfg_.spacer1_7z_name);
    spacer1_8x_.setup(cfg_.spacer1_8x_name);
    spacer1_8y_.setup(cfg_.spacer1_8y_name);
    spacer1_8z_.setup(cfg_.spacer1_8z_name);

    spacer2_1x_.setup(cfg_.spacer2_1x_name);
    spacer2_1y_.setup(cfg_.spacer2_1y_name);
    spacer2_1z_.setup(cfg_.spacer2_1z_name);
    spacer2_2x_.setup(cfg_.spacer2_2x_name);
    spacer2_2y_.setup(cfg_.spacer2_2y_name);
    spacer2_2z_.setup(cfg_.spacer2_2z_name);
    spacer2_3x_.setup(cfg_.spacer2_3x_name);
    spacer2_3y_.setup(cfg_.spacer2_3y_name);
    spacer2_3z_.setup(cfg_.spacer2_3z_name);
    spacer2_4x_.setup(cfg_.spacer2_4x_name);
    spacer2_4y_.setup(cfg_.spacer2_4y_name);
    spacer2_4z_.setup(cfg_.spacer2_4z_name);
    spacer2_5x_.setup(cfg_.spacer2_5x_name);
    spacer2_5y_.setup(cfg_.spacer2_5y_name);
    spacer2_5z_.setup(cfg_.spacer2_5z_name);
    spacer2_6x_.setup(cfg_.spacer2_6x_name);
    spacer2_6y_.setup(cfg_.spacer2_6y_name);
    spacer2_6z_.setup(cfg_.spacer2_6z_name);
    spacer2_7x_.setup(cfg_.spacer2_7x_name);
    spacer2_7y_.setup(cfg_.spacer2_7y_name);
    spacer2_7z_.setup(cfg_.spacer2_7z_name);

    sensorjointx_.setup(cfg_.sensorjointx_name);
    sensorjointy_.setup(cfg_.sensorjointy_name);
    sensorjointz_.setup(cfg_.sensorjointz_name);
    sensorjoint_beta_.setup(cfg_.sensorjoint_beta_name);
    sensorjoint_alpha_.setup(cfg_.sensorjoint_alpha_name);
    sensorjoint_gamma_.setup(cfg_.sensorjoint_gamma_name);

    disk1_4jointx_.setup(cfg_.disk1_4jointx_name);
    disk1_4jointy_.setup(cfg_.disk1_4jointy_name);
    disk1_4jointz_.setup(cfg_.disk1_4jointz_name);
    disk1_4joint_beta_.setup(cfg_.disk1_4joint_beta_name);
    disk1_4joint_alpha_.setup(cfg_.disk1_4joint_alpha_name);
    disk1_4joint_gamma_.setup(cfg_.disk1_4joint_gamma_name);
    disk1_8jointx_.setup(cfg_.disk1_8jointx_name);
    disk1_8jointy_.setup(cfg_.disk1_8jointy_name);
    disk1_8jointz_.setup(cfg_.disk1_8jointz_name);
    disk1_8joint_beta_.setup(cfg_.disk1_8joint_beta_name);
    disk1_8joint_alpha_.setup(cfg_.disk1_8joint_alpha_name);
    disk1_8joint_gamma_.setup(cfg_.disk1_8joint_gamma_name);
    disk2_4jointx_.setup(cfg_.disk2_4jointx_name);
    disk2_4jointy_.setup(cfg_.disk2_4jointy_name);
    disk2_4jointz_.setup(cfg_.disk2_4jointz_name);
    disk2_4joint_beta_.setup(cfg_.disk2_4joint_beta_name);
    disk2_4joint_alpha_.setup(cfg_.disk2_4joint_alpha_name);
    disk2_4joint_gamma_.setup(cfg_.disk2_4joint_gamma_name);




    // Set up the Arduino
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

    // Set up Continuum Robot model
    cr_model_ = ForwardKinematics(cfg_.spacer_distance, cfg_.dist_to_tendon, cfg_.spacer_num, cfg_.segment_num, cfg_.tendondrum_rad);
    pcc_model_ = new PiecewiseConstantCurvatureModel();


    // Set up Continuum Robot model for piecewiseconstantcurvature, also static model
    pcc_model_ -> setdefaultParameter(cfg_.spacer_distance, cfg_.dist_to_tendon, cfg_.spacer_num, cfg_.segment_num);
    RCLCPP_INFO(logger_, "Finished Configuration");

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> CrHwInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(xz1_servo_.name, hardware_interface::HW_IF_POSITION, &xz1_servo_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(yz1_servo_.name, hardware_interface::HW_IF_POSITION, &yz1_servo_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(xz2_servo_.name, hardware_interface::HW_IF_POSITION, &xz2_servo_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(yz2_servo_.name, hardware_interface::HW_IF_POSITION, &yz2_servo_.pos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(sensorjointx_.name, hardware_interface::HW_IF_POSITION, &sensorjointx_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensorjointy_.name, hardware_interface::HW_IF_POSITION, &sensorjointy_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensorjointz_.name, hardware_interface::HW_IF_POSITION, &sensorjointz_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensorjoint_beta_.name, hardware_interface::HW_IF_POSITION, &sensorjoint_beta_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensorjoint_alpha_.name, hardware_interface::HW_IF_POSITION, &sensorjoint_alpha_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensorjoint_gamma_.name, hardware_interface::HW_IF_POSITION, &sensorjoint_gamma_.pos));


    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_4jointx_.name, hardware_interface::HW_IF_POSITION, &disk1_4jointx_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_4jointy_.name, hardware_interface::HW_IF_POSITION, &disk1_4jointy_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_4jointz_.name, hardware_interface::HW_IF_POSITION, &disk1_4jointz_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_4joint_beta_.name, hardware_interface::HW_IF_POSITION, &disk1_4joint_beta_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_4joint_alpha_.name, hardware_interface::HW_IF_POSITION, &disk1_4joint_alpha_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_4joint_gamma_.name, hardware_interface::HW_IF_POSITION, &disk1_4joint_gamma_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_8jointx_.name, hardware_interface::HW_IF_POSITION, &disk1_8jointx_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_8jointy_.name, hardware_interface::HW_IF_POSITION, &disk1_8jointy_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_8jointz_.name, hardware_interface::HW_IF_POSITION, &disk1_8jointz_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_8joint_beta_.name, hardware_interface::HW_IF_POSITION, &disk1_8joint_beta_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_8joint_alpha_.name, hardware_interface::HW_IF_POSITION, &disk1_8joint_alpha_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk1_8joint_gamma_.name, hardware_interface::HW_IF_POSITION, &disk1_8joint_gamma_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk2_4jointx_.name, hardware_interface::HW_IF_POSITION, &disk2_4jointx_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk2_4jointy_.name, hardware_interface::HW_IF_POSITION, &disk2_4jointy_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk2_4jointz_.name, hardware_interface::HW_IF_POSITION, &disk2_4jointz_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk2_4joint_beta_.name, hardware_interface::HW_IF_POSITION, &disk2_4joint_beta_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk2_4joint_alpha_.name, hardware_interface::HW_IF_POSITION, &disk2_4joint_alpha_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(disk2_4joint_gamma_.name, hardware_interface::HW_IF_POSITION, &disk2_4joint_gamma_.pos));

    // Backbone Sections
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_1x_.name, hardware_interface::HW_IF_POSITION, &spacer1_1x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_1y_.name, hardware_interface::HW_IF_POSITION, &spacer1_1y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_1z_.name, hardware_interface::HW_IF_POSITION, &spacer1_1z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_2x_.name, hardware_interface::HW_IF_POSITION, &spacer1_2x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_2y_.name, hardware_interface::HW_IF_POSITION, &spacer1_2y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_2z_.name, hardware_interface::HW_IF_POSITION, &spacer1_2z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_3x_.name, hardware_interface::HW_IF_POSITION, &spacer1_3x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_3y_.name, hardware_interface::HW_IF_POSITION, &spacer1_3y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_3z_.name, hardware_interface::HW_IF_POSITION, &spacer1_3z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_4x_.name, hardware_interface::HW_IF_POSITION, &spacer1_4x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_4y_.name, hardware_interface::HW_IF_POSITION, &spacer1_4y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_4z_.name, hardware_interface::HW_IF_POSITION, &spacer1_4z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_5x_.name, hardware_interface::HW_IF_POSITION, &spacer1_5x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_5y_.name, hardware_interface::HW_IF_POSITION, &spacer1_5y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_5z_.name, hardware_interface::HW_IF_POSITION, &spacer1_5z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_6x_.name, hardware_interface::HW_IF_POSITION, &spacer1_6x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_6y_.name, hardware_interface::HW_IF_POSITION, &spacer1_6y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_6z_.name, hardware_interface::HW_IF_POSITION, &spacer1_6z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_7x_.name, hardware_interface::HW_IF_POSITION, &spacer1_7x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_7y_.name, hardware_interface::HW_IF_POSITION, &spacer1_7y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_7z_.name, hardware_interface::HW_IF_POSITION, &spacer1_7z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_8x_.name, hardware_interface::HW_IF_POSITION, &spacer1_8x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_8y_.name, hardware_interface::HW_IF_POSITION, &spacer1_8y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer1_8z_.name, hardware_interface::HW_IF_POSITION, &spacer1_8z_.pos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_1x_.name, hardware_interface::HW_IF_POSITION, &spacer2_1x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_1y_.name, hardware_interface::HW_IF_POSITION, &spacer2_1y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_1z_.name, hardware_interface::HW_IF_POSITION, &spacer2_1z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_2x_.name, hardware_interface::HW_IF_POSITION, &spacer2_2x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_2y_.name, hardware_interface::HW_IF_POSITION, &spacer2_2y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_2z_.name, hardware_interface::HW_IF_POSITION, &spacer2_2z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_3x_.name, hardware_interface::HW_IF_POSITION, &spacer2_3x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_3y_.name, hardware_interface::HW_IF_POSITION, &spacer2_3y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_3z_.name, hardware_interface::HW_IF_POSITION, &spacer2_3z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_4x_.name, hardware_interface::HW_IF_POSITION, &spacer2_4x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_4y_.name, hardware_interface::HW_IF_POSITION, &spacer2_4y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_4z_.name, hardware_interface::HW_IF_POSITION, &spacer2_4z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_5x_.name, hardware_interface::HW_IF_POSITION, &spacer2_5x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_5y_.name, hardware_interface::HW_IF_POSITION, &spacer2_5y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_5z_.name, hardware_interface::HW_IF_POSITION, &spacer2_5z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_6x_.name, hardware_interface::HW_IF_POSITION, &spacer2_6x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_6y_.name, hardware_interface::HW_IF_POSITION, &spacer2_6y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_6z_.name, hardware_interface::HW_IF_POSITION, &spacer2_6z_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_7x_.name, hardware_interface::HW_IF_POSITION, &spacer2_7x_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_7y_.name, hardware_interface::HW_IF_POSITION, &spacer2_7y_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(spacer2_7z_.name, hardware_interface::HW_IF_POSITION, &spacer2_7z_.pos));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CrHwInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(xz1_servo_.name, hardware_interface::HW_IF_POSITION, &xz1_servo_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(yz1_servo_.name, hardware_interface::HW_IF_POSITION, &yz1_servo_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(xz2_servo_.name, hardware_interface::HW_IF_POSITION, &xz2_servo_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(yz2_servo_.name, hardware_interface::HW_IF_POSITION, &yz2_servo_.cmd));

  return command_interfaces;
}

hardware_interface::return_type CrHwInterface::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");
  arduino_.sendEmptyMsg();
  arduino_.setPidValues(30, 20, 0, 100);
  arduino_.active(true);
  arduino_.activeLED(20);

  ////--this part is used for the old robot
  // //Create your Phidget channels, segment 1 
  // PhidgetVoltageRatioInput_create(&voltageRatioInput10);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput11);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput12);
  // PhidgetVoltageRatioInput_create(&voltageRatioInput13);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput10, 566303);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput11, 566303);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput12, 566303);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput13, 566303);
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
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput20, 566471);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput21, 566471);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput22, 566471);
  // Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput23, 566471);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput20, 0);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput21, 1);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput22, 2);
  // Phidget_setChannel((PhidgetHandle)voltageRatioInput23, 3);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput20, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput21, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput22, 5000);
  // Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput23, 5000);
  ////--this part is used for the old robot

  //--this part is used for the new robot
  //Create your Phidget channels, segment 1 
  PhidgetVoltageRatioInput_create(&voltageRatioInput10);
  PhidgetVoltageRatioInput_create(&voltageRatioInput11);
  PhidgetVoltageRatioInput_create(&voltageRatioInput12);
  PhidgetVoltageRatioInput_create(&voltageRatioInput13);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput10, 587030);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput11, 587030);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput12, 587030);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput13, 587030);
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
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput20, 580129);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput21, 580129);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput22, 580129);
  Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput23, 580129);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput20, 0);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput21, 1);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput22, 2);
  Phidget_setChannel((PhidgetHandle)voltageRatioInput23, 3);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput20, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput21, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput22, 5000);
  Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput23, 5000);
  //--this part is used for the new robot

  RCLCPP_INFO(
  logger_, "sucessfully start");

  comms = std::make_shared<light_sub::CrCommunication>();
  executor_.add_node(comms);
  std::thread([this](){executor_.spin();}).detach();

  status_ = hardware_interface::status::STARTED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CrHwInterface::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  arduino_.active(false);
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

  status_ = hardware_interface::status::STOPPED;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CrHwInterface::read()
{
    if (!arduino_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }
    std::string resp;

    resp = arduino_.readServosPos(xz1_servo_.pwm, yz1_servo_.pwm, xz2_servo_.pwm, yz2_servo_.pwm);
    RCLCPP_DEBUG_STREAM(logger_, "Read: " << resp);

    // Map from 0 - 180° (this is what the arduino accepts) to -135° to 135° (Servo has 270° range of motion)
    xz1_servo_.pos = arduino_.pwmToPos(xz1_servo_.pwm);
    yz1_servo_.pos = arduino_.pwmToPos(yz1_servo_.pwm);
    xz2_servo_.pos = arduino_.pwmToPos(xz2_servo_.pwm);
    yz2_servo_.pos = arduino_.pwmToPos(yz2_servo_.pwm);

    // get tendon forces
    std::vector<double> voltageRatios;
    std::vector<double> calibrated_forces;

    // get load cells' voltage_ratios
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

    // arrange tendon forces：tendon1:+y tendon2:+x tendon3:-y tendon4:-x 
    Eigen::MatrixXd f_tendon;
    f_tendon.resize(8,1);
    f_tendon << 
                (voltageRatios[3]*cfg_.tendon1_p1+cfg_.tendon1_p2),
                (voltageRatios[1]*cfg_.tendon2_p1+cfg_.tendon2_p2),
                (voltageRatios[2]*cfg_.tendon3_p1+cfg_.tendon3_p2),
                (voltageRatios[0]*cfg_.tendon4_p1+cfg_.tendon4_p2),

                (voltageRatios[7]*cfg_.tendon5_p1+cfg_.tendon5_p2),
                (voltageRatios[5]*cfg_.tendon6_p1+cfg_.tendon6_p2),
                (voltageRatios[6]*cfg_.tendon7_p1+cfg_.tendon7_p2),
                (voltageRatios[4]*cfg_.tendon8_p1+cfg_.tendon8_p2);
                                           
    // Backbone pose (xyz + beta alpha gamma)
    //External force at tip set to zero
    Eigen::Vector3d f_ext;
    f_ext.setZero();

	  //External monment at tip set to zero
    Eigen::Vector3d l_ext;
    l_ext.setZero();

    //Variable to store the tip frame
    Eigen::Matrix4d ee_frame;
    Eigen::MatrixXd diskFrames;
    pcc_model_ -> forwardKinematics(diskFrames,f_tendon,f_ext,l_ext);
    m_disk_frames = diskFrames;
    ee_frame = m_disk_frames.rightCols(4);// extracting the rightmost 4 columns of the matrix
    
    position_cfg pcc_pose = pcc_model_->calculate_pose(ee_frame);

    // could use this to show the tendon shortenning
    Eigen::MatrixXd Tendonlength = pcc_model_ -> getTendonDisplacements(diskFrames);

    // Get translations, show in rviz2
    sensorjointx_.pos = pcc_pose.x;
    sensorjointy_.pos = pcc_pose.y;
    sensorjointz_.pos = pcc_pose.z;
    sensorjoint_beta_.pos = pcc_pose.beta;
    sensorjoint_alpha_.pos = pcc_pose.alpha;
    sensorjoint_gamma_.pos = pcc_pose.gamma;

    // get each part of disk pose
    position_cfg pose1_4 = pcc_model_ -> get_segment_pose(m_disk_frames,1);// 1_4 spacer
    position_cfg pose1_8 = pcc_model_ -> get_segment_pose(m_disk_frames,2);//1_8 spacer
    position_cfg pose2_4 = pcc_model_ -> get_segment_pose(m_disk_frames,3);//2_4 spacer

    // define a base frame
    position_cfg pose_base;
    pose_base.x = 0;
    pose_base.y = 0;
    pose_base.z = 0;
    pose_base.beta = 0;
    pose_base.alpha = 0;
    pose_base.gamma = 0;

    // get position, show in rviz2
    disk1_4jointx_.pos = pose1_4.x;
    disk1_4jointy_.pos = pose1_4.y;
    disk1_4jointz_.pos = pose1_4.z;
    disk1_4joint_beta_.pos = pose1_4.beta;
    disk1_4joint_alpha_.pos = pose1_4.alpha;
    disk1_4joint_gamma_.pos = pose1_4.gamma;
    disk1_8jointx_.pos = pose1_8.x;
    disk1_8jointy_.pos = pose1_8.y;
    disk1_8jointz_.pos = pose1_8.z;
    disk1_8joint_beta_.pos = pose1_8.beta;
    disk1_8joint_alpha_.pos = pose1_8.alpha;
    disk1_8joint_gamma_.pos = pose1_8.gamma;
    disk2_4jointx_.pos = pose2_4.x;
    disk2_4jointy_.pos = pose2_4.y;
    disk2_4jointz_.pos = pose2_4.z;
    disk2_4joint_beta_.pos = pose2_4.beta;
    disk2_4joint_alpha_.pos = pose2_4.alpha;
    disk2_4joint_gamma_.pos = pose2_4.gamma;


    //Dummy
    spacer1_1x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_1y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_1z_.pos = pcc_model_->get_segmentz(pose_base,pose1_4,4);//0.016+xz2_servo_.pos/100;
    spacer1_2x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_2y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_2z_.pos = pcc_model_->get_segmentz(pose_base,pose1_4,4);//0.016+xz2_servo_.pos/100;
    spacer1_3x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_3y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_3z_.pos = pcc_model_->get_segmentz(pose_base,pose1_4,4);//0.016+xz2_servo_.pos/100;
    spacer1_4x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_4y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,0,4,4),m_disk_frames.block(0,4,4,4),4);//xz2_servo_.pos/10;
    spacer1_4z_.pos = pcc_model_->get_segmentz(pose_base,pose1_4,4);//0.016+xz2_servo_.pos/100;
    spacer1_5x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_5y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_5z_.pos = pcc_model_->get_segmentz(pose1_4,pose1_8,4);//0.016+xz2_servo_.pos/100;
    spacer1_6x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_6y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_6z_.pos = pcc_model_->get_segmentz(pose1_4,pose1_8,4);//0.016+xz2_servo_.pos/100;
    spacer1_7x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_7y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_7z_.pos = pcc_model_->get_segmentz(pose1_4,pose1_8,4);//0.016+xz2_servo_.pos/100;
    spacer1_8x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_8y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,4,4,4),m_disk_frames.block(0,8,4,4),4);//xz2_servo_.pos/10;
    spacer1_8z_.pos = pcc_model_->get_segmentz(pose1_4,pose1_8,4);//0.016+xz2_servo_.pos/100;

    spacer2_1x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_1y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_1z_.pos = pcc_model_->get_segmentz(pose1_8,pose2_4,4);//0.016+xz2_servo_.pos/100;
    spacer2_2x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_2y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_2z_.pos = pcc_model_->get_segmentz(pose1_8,pose2_4,4);//0.016+xz2_servo_.pos/100;
    spacer2_3x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_3y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_3z_.pos = pcc_model_->get_segmentz(pose1_8,pose2_4,4);//0.016+xz2_servo_.pos/100;
    spacer2_4x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_4y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,8,4,4),m_disk_frames.block(0,12,4,4),4);//xz2_servo_.pos/10;
    spacer2_4z_.pos = pcc_model_->get_segmentz(pose1_8,pose2_4,4);//0.016+xz2_servo_.pos/100;
    spacer2_5x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,12,4,4),ee_frame,4);//xz2_servo_.pos/10;
    spacer2_5y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,12,4,4),ee_frame,4);//xz2_servo_.pos/10;
    spacer2_5z_.pos = pcc_model_->get_segmentz(pose2_4,pcc_pose,4);//0.016+xz2_servo_.pos/100;
    spacer2_6x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,12,4,4),ee_frame,4);//xz2_servo_.pos/10;
    spacer2_6y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,12,4,4),ee_frame,4);//xz2_servo_.pos/10;
    spacer2_6z_.pos = pcc_model_->get_segmentz(pose2_4,pcc_pose,4);//0.016+xz2_servo_.pos/100;
    spacer2_7x_.pos = pcc_model_->get_segmentx(m_disk_frames.block(0,12,4,4),ee_frame,4);//xz2_servo_.pos/10;
    spacer2_7y_.pos = pcc_model_->get_segmenty(m_disk_frames.block(0,12,4,4),ee_frame,4);//xz2_servo_.pos/10;
    spacer2_7z_.pos = pcc_model_->get_segmentz(pose2_4,pcc_pose,4);//0.016+xz2_servo_.pos/100;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CrHwInterface::write()
{
  if (!arduino_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  std::string resp;
  RCLCPP_DEBUG_STREAM(logger_, "cmd: " << yz2_servo_.cmd);
  RCLCPP_DEBUG_STREAM(logger_, "pwm: " << arduino_.posToPwm(yz2_servo_.cmd));
  resp = arduino_.setServos(arduino_.posToPwm(xz1_servo_.cmd), arduino_.posToPwm(yz1_servo_.cmd), arduino_.posToPwm(xz2_servo_.cmd), arduino_.posToPwm(yz2_servo_.cmd));

  arduino_.activeLED(comms->getLEDValue());

  return hardware_interface::return_type::OK;
}


// -------    PLUGIN    --------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    cr_hw_interface::CrHwInterface,
    hardware_interface::SystemInterface
)