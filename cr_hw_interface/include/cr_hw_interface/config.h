#ifndef CR_HW_INTERFACE_CONFIG_H
#define CR_HW_INTERFACE_CONFIG_H

#include <string>

struct Config
{
    // Servos
    std::string xz1_servo_name = "xz1_servo";
    std::string yz1_servo_name = "yz1_servo";
    std::string xz2_servo_name = "xz2_servo";
    std::string yz2_servo_name = "yz2_servo";

    // Direct Transform to TCP/Sensor
    std::string sensorjointx_name = "sensorjointx";
    std::string sensorjointy_name = "sensorjointy";
    std::string sensorjointz_name = "sensorjointz";
    std::string sensorjoint_beta_name = "sensorjoint_beta";
    std::string sensorjoint_alpha_name = "sensorjoint_alpha";
    std::string sensorjoint_gamma_name = "sensorjoint_gamma";

    // Direct Transform to each important Disk
    std::string disk1_4jointx_name = "disk1_4jointx";
    std::string disk1_4jointy_name = "disk1_4jointy";
    std::string disk1_4jointz_name = "disk1_4jointz";
    std::string disk1_4joint_beta_name = "disk1_4joint_beta";
    std::string disk1_4joint_alpha_name = "disk1_4joint_alpha";
    std::string disk1_4joint_gamma_name = "disk1_4joint_gamma";

    std::string disk1_8jointx_name = "disk1_8jointx";
    std::string disk1_8jointy_name = "disk1_8jointy";
    std::string disk1_8jointz_name = "disk1_8jointz";
    std::string disk1_8joint_beta_name = "disk1_8joint_beta";
    std::string disk1_8joint_alpha_name = "disk1_8joint_alpha";
    std::string disk1_8joint_gamma_name = "disk1_8joint_gamma";

    std::string disk2_4jointx_name = "disk2_4jointx";
    std::string disk2_4jointy_name = "disk2_4jointy";
    std::string disk2_4jointz_name = "disk2_4jointz";
    std::string disk2_4joint_beta_name = "disk2_4joint_beta";
    std::string disk2_4joint_alpha_name = "disk2_4joint_alpha";
    std::string disk2_4joint_gamma_name = "disk2_4joint_gamma";




    // Transform for each Spacer
    std::string spacer1_1x_name = "spacer1_1x";
    std::string spacer1_1y_name = "spacer1_1y";
    std::string spacer1_1z_name = "spacer1_1z";
    std::string spacer1_2x_name = "spacer1_2x";
    std::string spacer1_2y_name = "spacer1_2y";
    std::string spacer1_2z_name = "spacer1_2z";
    std::string spacer1_3x_name = "spacer1_3x";
    std::string spacer1_3y_name = "spacer1_3y";
    std::string spacer1_3z_name = "spacer1_3z";
    std::string spacer1_4x_name = "spacer1_4x";
    std::string spacer1_4y_name = "spacer1_4y";
    std::string spacer1_4z_name = "spacer1_4z";
    std::string spacer1_5x_name = "spacer1_5x";
    std::string spacer1_5y_name = "spacer1_5y";
    std::string spacer1_5z_name = "spacer1_5z";
    std::string spacer1_6x_name = "spacer1_6x";
    std::string spacer1_6y_name = "spacer1_6y";
    std::string spacer1_6z_name = "spacer1_6z";
    std::string spacer1_7x_name = "spacer1_7x";
    std::string spacer1_7y_name = "spacer1_7y";
    std::string spacer1_7z_name = "spacer1_7z";
    std::string spacer1_8x_name = "spacer1_8x";
    std::string spacer1_8y_name = "spacer1_8y";
    std::string spacer1_8z_name = "spacer1_8z";
    std::string spacer2_1x_name = "spacer2_1x";
    std::string spacer2_1y_name = "spacer2_1y";
    std::string spacer2_1z_name = "spacer2_1z";
    std::string spacer2_2x_name = "spacer2_2x";
    std::string spacer2_2y_name = "spacer2_2y";
    std::string spacer2_2z_name = "spacer2_2z";
    std::string spacer2_3x_name = "spacer2_3x";
    std::string spacer2_3y_name = "spacer2_3y";
    std::string spacer2_3z_name = "spacer2_3z";
    std::string spacer2_4x_name = "spacer2_4x";
    std::string spacer2_4y_name = "spacer2_4y";
    std::string spacer2_4z_name = "spacer2_4z";
    std::string spacer2_5x_name = "spacer2_5x";
    std::string spacer2_5y_name = "spacer2_5y";
    std::string spacer2_5z_name = "spacer2_5z";
    std::string spacer2_6x_name = "spacer2_6x";
    std::string spacer2_6y_name = "spacer2_6y";
    std::string spacer2_6z_name = "spacer2_6z";
    std::string spacer2_7x_name = "spacer2_7x";
    std::string spacer2_7y_name = "spacer2_7y";
    std::string spacer2_7z_name = "spacer2_7z";

    // Arduino settings
    std::string device = "/dev/ttyUSB0";
    int baud_rate = 57600;
    int timeout = 1000;
    int enc_counts_per_rev = 1920;

    // Continuum Robot structure
    double spacer_distance = 0.01625;
    double dist_to_tendon = 0.0035;
    double spacer_num = 8;
    double segment_num  = 2;
    double tendondrum_rad = 0.006;

    // // robot 1
    // //parameters for calibration,segment 1 
    // double tendon1_p1 = -44217.463090;
    // double tendon1_p2 = 2.287687; //voltageratio13
    // double tendon2_p1 = -42555.821970;
    // double tendon2_p2 = 0.326224; //voltageratio11
    // double tendon3_p1 = -44882.711679;
    // double tendon3_p2 = -2.466671; //voltageratio12
    // double tendon4_p1 = -42302.842609;
    // double tendon4_p2 = -3.119007; //voltageratio10

    // //parameters for calibration, segment 2
    // double tendon5_p1 = -42070.817425;
    // double tendon5_p2 = 2.124886; //voltageratio23
    // double tendon6_p1 = -44466.867639;
    // double tendon6_p2 = 2.223417; //voltageratio21
    // double tendon7_p1 = -42494.916302;
    // double tendon7_p2 = -2.288548; //voltageratio22
    // double tendon8_p1 = -42769.126334;
    // double tendon8_p2 = 1.983674; //voltageratio20


    // robot 2
    //parameters for calibration,segment 1 
    double tendon1_p1 = 42617.068540;
    double tendon1_p2 = 0.406812; //voltageratio13
    double tendon2_p1 = 43280.945419;
    double tendon2_p2 = 0.092221; //voltageratio11
    double tendon3_p1 = 43077.485937;
    double tendon3_p2 = 0.651276; //voltageratio12
    double tendon4_p1 = 42878.899824;
    double tendon4_p2 = -4.688192; //voltageratio10

    //parameters for calibration, segment 2
    double tendon5_p1 = 43146.669042;
    double tendon5_p2 = -5.398363; //voltageratio23
    double tendon6_p1 = 43231.627194;
    double tendon6_p2 = -7.798452; //voltageratio21
    double tendon7_p1 = 43297.897578;
    double tendon7_p2 = -4.096361; //voltageratio22
    double tendon8_p1 = 42979.712312;
    double tendon8_p2 = -3.650486; //voltageratio20
};


#endif // CR_HW_INTERFACE_CONFIG_H