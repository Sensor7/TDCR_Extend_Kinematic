#ifndef CR_HW_INTERFACE_ARDUINO_COMMS_H
#define CR_HW_INTERFACE_ARDUINO_COMMS_H

#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>


class ArduinoComms
{
public:

  ArduinoComms()
  {
      this->servo_range = 270;
      this->servo_neutral_pos = 135;
      this->pwm_max = 180;
  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
      : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
  {
      this->servo_range = 270;
      this->servo_neutral_pos = 135;
      this->pwm_max = 180;
  }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  std::string readServosPos(int &val_1, int &val_2, int &val_3, int &val_4);
  std::string setServos(int val_1, int val_2, int val_3, int val_4);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  void active(bool active);

  std::string activeLED(int led_pwm);

  float pwmToPos(int pwm);
  int posToPwm(float pos);

  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  serial::Serial serial_conn_;
  float servo_range;
  float servo_neutral_pos;
  float pwm_max;

};

#endif // CR_HW_INTERFACE_ARDUINO_COMMS_H