#include "cr_hw_interface/spacer.h"


Spacer::Spacer(const std::string &spacer_name)
{
  setup(spacer_name);
}


void Spacer::setup(const std::string &spacer_name)
{
  name = spacer_name;
}
