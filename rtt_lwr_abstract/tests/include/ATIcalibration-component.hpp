// Copyright (C) 2014 ISIR-CNRS
// Author: Guillaume Hamon, hamon@isir.upmc.fr

#ifndef OROCOS_ATICALIBRATION_COMPONENT_HPP
#define OROCOS_ATICALIBRATION_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include "friRTNetExampleAbstract.hpp"
#include <Eigen/Dense>
#include <rtt/Component.hpp>
#include <iostream>
#include <cmath>
#include <boost/foreach.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/typekit/Types.h>

class ATIcalibration : public FriRTNetExampleAbstract{
  public:
    ATIcalibration(std::string const& name);
    ~ATIcalibration();
    bool configureHook();
    bool doStart();
    void updateHook();
    void setFRIRate(double period_ms);

    std::vector<double> JState_init;//msr_joint_positions
    bool end_calibration;
    std::vector<double> joints_position_command;
    std::vector<double> joints_position_command_interp;
   //Positions de calibration
    std::vector<double> position1;
    std::vector<double> position2;
    std::vector<double> position3;
    std::vector<double> position4;
    std::vector<double> position5;
    std::vector<double> JState;

    double FRIRate;
    double velocity_limit;
    double tf;
    double t;
    int n;

    std::vector<double> valeurZ;
    std::vector<double> valeurX;
    std::vector<double> valeurY;

    std::vector<double> tf_min;

    std::vector<double> external_torque;
    std::vector<double> msr_torque;

    int i;
    RTT::InputPort< std::vector<double> > iport_msr_joint_torque;
    RTT::InputPort< std::vector<double> > iport_ATI_values;
    RTT::InputPort< std::vector<double> > iport_est_ext_joint_torque;
    RTT::OutputPort< Eigen::Matrix<double,3,6> > oport_calibration_results;
    RTT::OutputPort< bool > oport_bias_order;
    RTT::OutputPort< std::vector<double> > oport_add_joint_torque;
    RTT::OutputPort< std_msgs::Float32 > oport_ati_force;
    RTT::OutputPort< std_msgs::Float32 > oport_ati_torque;
    RTT::OutputPort< std_msgs::Float32 > oport_kuka_force;
    RTT::OutputPort< std_msgs::Float32 > oport_kuka_torque;

};
#endif
