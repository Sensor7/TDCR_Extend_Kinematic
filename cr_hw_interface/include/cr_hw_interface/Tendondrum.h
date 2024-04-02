#ifndef CR_HW_INTERFACE_TENDONDRUM_H
#define CR_HW_INTERFACE_TENDONDRUM_H

#include <string>


class Tendondrum
{
    public:

    std::string name = "";
    int pwm = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;

    Tendondrum() = default;

    Tendondrum(const std::string &wheel_name);
    
    void setup(const std::string &wheel_name);
};


#endif // CR_HW_INTERFACE_TENDONDRUM_H