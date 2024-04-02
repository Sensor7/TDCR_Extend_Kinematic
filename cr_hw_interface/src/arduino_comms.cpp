#include "cr_hw_interface/arduino_comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}

void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

std::string ArduinoComms::readServosPos(int &val_1, int &val_2, int &val_3, int &val_4)
{
    std::string response1 = sendMsg("t 0\r");
    //RCLCPP_INFO_STREAM(logger_, "response " << response);
    val_1 = std::atoi(response1.c_str());

    std::string response2 = sendMsg("t 1\r");
    val_2 = std::atoi(response2.c_str());

    std::string response3 = sendMsg("t 2\r");
    val_3 = std::atoi(response3.c_str());

    std::string response4 = sendMsg("t 3\r");
    val_4 = std::atoi(response4.c_str());

    return response1;
}

std::string ArduinoComms::setServos(int val_1, int val_2, int val_3, int val_4)
{
    std::stringstream ss1;
    ss1 << "s 0 " << val_1 << "\r";
    sendMsg(ss1.str(), true);

    std::stringstream ss2;
    ss2 << "s 1 " << val_2 << "\r";
    sendMsg(ss2.str(), true);

    std::stringstream ss3;
    ss3 << "s 2 " << val_3 << "\r";
    sendMsg(ss3.str(), true);

    std::stringstream ss4;
    ss4 << "s 3 " << val_4 << "\r";
    return sendMsg(ss4.str(), true);
}

std::string ArduinoComms::activeLED(int led_pwm)
{
    int ledval = 0;
    if (led_pwm >=0 && led_pwm <= 255) {
        ledval = led_pwm;
    }
    else if (led_pwm > 255) {
        ledval = 255;
    }
    std::stringstream ledstr;
    ledstr << "l " << ledval<< "\r";
    return sendMsg(ledstr.str(), true);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();


    if (print_output)
    {
        //RCLCPP_INFO_STREAM(logger_, "xz1_servo_.cmd " << xz1_servo_.cmd);
        //RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        //RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }


    /*std::string msg = msg_to_send.c_str();

    std::size_t pos = msg.find("\r");
    if (pos == std::string::npos) ;
    else msg.replace(pos, 1, " ");

    pos = response.find("\r");
    if (pos == std::string::npos) ;
    else response.replace(pos, 1, " ");

    std::string retVal = msg + response;*/

    std::string retVal = response;

    return retVal;
}

float ArduinoComms::pwmToPos(int pwm)
{
    return (pwm/this->pwm_max * this->servo_range - this->servo_neutral_pos)/180*3.141;
}

int ArduinoComms::posToPwm(float pos)
{
    float result = (pos*180/3.141 + this->servo_neutral_pos)/this->servo_range * this->pwm_max;
    return int(result);
}

void ArduinoComms::active(bool active)
{
    std::stringstream ss;
    ss << "f ";
    if (active) ss << "1";
    else ss << "0";
    ss << "\r";
    sendMsg(ss.str(), true);
}
