#ifndef CR_HW_INTERFACE_SPACER_H
#define CR_HW_INTERFACE_SPACER_H

#include <string>


class Spacer
{
    public:

    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;

    Spacer() = default;

    Spacer(const std::string &spacer_name);
    
    void setup(const std::string &spacer_name);
};

#endif // CR_HW_INTERFACE_SPACER_H