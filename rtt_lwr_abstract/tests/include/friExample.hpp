// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef FRI_EXAMPLE__HPP
#define FRI_EXAMPLE__HPP

#include "friExampleAbstract.hpp"

#include <string>

#include <rtt/RTT.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

class FriExample : public FriExampleAbstract{
    public:
        FriExample(std::string const& name);
        ~FriExample();

        void updateHook();

        void getFRIJointState();
        void getCartesianPosition();
        void getCartesianFrame();
        void getRobotState();
        void getJointState();
        void getCartesianWrench();
        void getJacobian();

        /** @brief Get the mass matrix
         */
        void getMassMatrix();

        /** @brief Send desired joint position
         * @pre Required control mode is 10
         * @param command The desired joint position (rad), size 7
         */
        void sendJointPositions(std::vector<double> &command);

        /** @brief Send desired joint velocities
         * @pre Required control mode is 10
         * @param command The desired joint velocities (rad/s), size 7s
         */
        void sendJointVelocities(std::vector<double> &command);

        /** @brief Send desired joint torque
         * @pre Required control mode is 30
         * @param command The desired joint torque (Nm), size 7s
         */
        void sendJointTorque(std::vector<double> &command);

        /** @brief Set stiffness and damping for joint 
         * @param stiffness The desired stiffness for the joint (Nm/rad)
         * @param damping The desired damping (Normalized)
         */
        void sendJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping);

        /** @brief Set desired cartesian position of $STIFFNESS.TOOL in $STIFFNESS.BASE
         * @pre Required control mode is 20
         * @param position The desired position (m) vector 3
         * @param orientation The desired quaternion (m)
         */
        void sendCartesianPose(std::vector<double> &position, std::vector<double> &orientation);

        /** @brief Set desired cartesian velocity of $STIFFNESS.TOOL in $STIFFNESS.BASE
         * @pre Required control mode is 20
         * @param linear The desired linear velocity (m/s) vector3
         * @param angular The desired angular velocity (rad/s) vector3
         */
        void sendCartesianVel(std::vector<double> &linear, std::vector<double> &angular);

        /** @brief Set desired cartesian force in $STIFFNESS.TOOL
         * @pre Required control mode is 20
         * @param force The desired force (N)
         * @param torque The desired torque (Nm)
         */
        void sendCartesianWrench(std::vector<double> &force, std::vector<double> &torque);

        /** @brief Set desired cartesian stiffness
         * @pre Required control mode is 20
         * @param stiffness The desired stiffness (N/m, Nm/rad)
         * @param damping The desired damping (Normalized)
         */
        void sendCartesianImpedance(std::vector<double> &stiffness, std::vector<double> &damping);

};

#endif
