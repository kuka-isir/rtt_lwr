// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef FRI_EXAMPLE_TORQUE_RTNET_HPP
#define FRI_EXAMPLE_TORQUE_RTNET_HPP

#include "friRTNetExampleAbstract.hpp"

#include <string>

#include <rtt/RTT.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <motion_control_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

class FriExampleTorqueRTNet : public FriRTNetExampleAbstract{
    public:
        FriExampleTorqueRTNet(std::string const& name);
        ~FriExampleTorqueRTNet();

        void updateHook();
        bool configureHook();
        bool doStart();

        std::vector<double> fri_joint_state_data;
         

        void getFRIJointState();
        void setT(double t);
        double torque;
};

#endif

