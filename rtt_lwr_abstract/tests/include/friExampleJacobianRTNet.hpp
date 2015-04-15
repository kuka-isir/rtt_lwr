// Copyright (C) 2014 ISIR-CNRS
// Author: Guillaume Hamon

#ifndef FRI_EXAMPLE_JACOBIAN_RT_HPP
#define FRI_EXAMPLE_JACOBIAN_RT_HPP

#include "friRTNetExampleAbstract.hpp"

#include <string>

#include <rtt/RTT.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <motion_control_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

class FriExampleJacobianRTNet : public FriRTNetExampleAbstract{
    public:
        FriExampleJacobianRTNet(std::string const& name);
        ~FriExampleJacobianRTNet();

        void updateHook();
        bool configureHook();
        bool doStart();

        std::vector<double> fri_joint_state_data;

        /** @brief Set stiffness and damping for joint 
         * @param stiffness The desired stiffness for the joint (Nm/rad)
         * @param damping The desired damping (Normalized)
         */
        //void setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping);

        void getFRIJointState();
        void setT(double t);
        double torque;
	//KDL::Vector Xdes;
	KDL::Jacobian J;
	geometry_msgs::Pose X;
	double Kp, Kd;
	std::vector<double> Joint_vel; /** à changer: non realtime **/
};

#endif

