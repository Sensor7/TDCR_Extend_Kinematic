#include "cr_hw_interface/Tendondrum.h"


Tendondrum::Tendondrum(const std::string &wheel_name)
{
  setup(wheel_name);
}


void Tendondrum::setup(const std::string &wheel_name)
{
  name = wheel_name;
}