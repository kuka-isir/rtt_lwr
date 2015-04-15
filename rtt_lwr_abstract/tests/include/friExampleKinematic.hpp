// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef FRI_EXAMPLE_KINEMATIC_HPP
#define FRI_EXAMPLE_KINEMATIC_HPP

#include "friExampleAbstract.hpp"

#include <string>

#include <rtt/RTT.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <motion_control_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

class FriExampleKinematic : public FriExampleAbstract{
    public:
        FriExampleKinematic(std::string const& name);
        ~FriExampleKinematic();

        void updateHook();

        bool doStart();

        void setDesiredQ(std::vector<double> &qdes_);

        /** @brief Set stiffness and damping for joint 
         * @param stiffness The desired stiffness for the joint (Nm/rad)
         * @param damping The desired damping (Normalized)
         */
        void setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping);

        void setLambda(double l);

        RTT::InputPort<lwr_fri::FriJointState>  iport_fri_joint_state;
        RTT::InputPort<sensor_msgs::JointState> iport_joint_state;

        RTT::OutputPort<motion_control_msgs::JointPositions>  oport_joint_position;
        RTT::OutputPort<motion_control_msgs::JointVelocities> oport_joint_velocities;
        RTT::OutputPort<motion_control_msgs::JointEfforts>    oport_joint_efforts;
        RTT::OutputPort<lwr_fri::FriJointImpedance>           oport_joint_impedance;

        double lambda;
        std::vector<double> qdes;
};

#endif

