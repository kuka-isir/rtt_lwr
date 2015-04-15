// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExample.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <kdl/frames_io.hpp>
#include <math.h>

FriExample::FriExample(std::string const& name) : FriExampleAbstract(name){
    this->addOperation("getFRIJointState", &FriExample::getFRIJointState, this, RTT::OwnThread);
    this->addOperation("getCartesianPosition", &FriExample::getCartesianPosition, this, RTT::OwnThread);
    this->addOperation("getCartesianFrame", &FriExample::getCartesianFrame, this, RTT::OwnThread);
    this->addOperation("getRobotState", &FriExample::getRobotState, this, RTT::OwnThread);
    this->addOperation("getJointState", &FriExample::getJointState, this, RTT::OwnThread);
    this->addOperation("getCartesianWrench", &FriExample::getCartesianWrench, this, RTT::OwnThread);
    this->addOperation("getJacobian", &FriExample::getJacobian, this, RTT::OwnThread);
    this->addOperation("getMassMatrix", &FriExample::getMassMatrix, this, RTT::OwnThread);

    this->addOperation("sendJointPositions", &FriExample::sendJointPositions, this, RTT::OwnThread);
    this->addOperation("sendJointVelocities", &FriExample::sendJointVelocities, this, RTT::OwnThread);
    this->addOperation("sendJointTorque", &FriExample::sendJointTorque, this, RTT::OwnThread);
    this->addOperation("sendJointImpedance", &FriExample::sendJointImpedance, this, RTT::OwnThread);
    this->addOperation("sendCartesianPose", &FriExample::sendCartesianPose, this, RTT::OwnThread);
    this->addOperation("sendCartesianVel", &FriExample::sendCartesianVel, this, RTT::OwnThread);
    this->addOperation("sendCartesianWrench", &FriExample::sendCartesianWrench, this, RTT::OwnThread);
    this->addOperation("sendCartesianImpedance", &FriExample::sendCartesianImpedance, this, RTT::OwnThread);
}

FriExample::~FriExample(){
}

void FriExample::updateHook(){
}

void FriExample::getFRIJointState(){
    lwr_fri::FriJointState fri_joint_state_data;
    RTT::FlowStatus fri_jointStateFS = iport_fri_joint_state.read(fri_joint_state_data);

    if(fri_jointStateFS == RTT::NewData){
        std::cout << "Measured Joint configuration" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration before FRI" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration FRI offset" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPosFriOffset){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Measured Joint torque actuator" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Estimated External Joint torque sensor" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.estExtJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Gravity compensation" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.gravity){
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
}

void FriExample::getCartesianPosition(){
    geometry_msgs::Pose cartPosData;
    RTT::FlowStatus cartPosFS = iport_cart_pos.read(cartPosData);

    if(cartPosFS == RTT::NewData){
        std::cout << "Position end effector [x, y, z]:" << std::endl;
        std::cout << "[" << cartPosData.position.x
                  << ", " << cartPosData.position.y 
                  << ", " << cartPosData.position.z
                  << "]" << std::endl;
        std::cout << "Orientation end effector [x, y, z, w]:" << std::endl;
        std::cout << "[" << cartPosData.orientation.x
                  << ", " << cartPosData.orientation.y
                  << ", " << cartPosData.orientation.z
                  << ", " << cartPosData.orientation.w
                  << "]" << std::endl;
    }
}

void FriExample::getCartesianFrame(){
    KDL::Frame cart_frame_data;
    RTT::FlowStatus cartFrameFS = iport_cart_frame.read(cart_frame_data);
    if(cartFrameFS == RTT::NewData){
        std::cout << "Cartesian Frame end effector" << std::endl;
        std::cout << cart_frame_data << std::endl;
    }
}

void FriExample::getRobotState(){
    tFriRobotState robot_state_data;
    RTT::FlowStatus robotStateFS = iport_robot_state.read(robot_state_data);
    if(robotStateFS == RTT::NewData){
        std::cout << "Power state drive" << std::endl;
        std::cout << robot_state_data.power << std::endl;
        std::cout << "Selected control strategy" << std::endl;
        std::cout << robot_state_data.control << std::endl;
        std::cout << "Drive error" << std::endl;
        std::cout << robot_state_data.error << std::endl;
        std::cout << "Temperature of drives" << std::endl;
        for(unsigned int i=0; i<7; i++)
            std::cout << robot_state_data.temperature[i] << " ";
        std::cout << std::endl;
    }
}

void FriExample::getJointState(){
    sensor_msgs::JointState joint_state_data;
    RTT::FlowStatus joint_state_fs = iport_joint_state.read(joint_state_data);
    if(joint_state_fs == RTT::NewData){
        std::cout << "Joint State" << std::endl;
        //joint_state_data.position is a std::vector
        BOOST_FOREACH(double d, joint_state_data.position){
            std::cout << d << " " << std::endl;
        }
        std::cout << "Joint effort" << std::endl;
        BOOST_FOREACH(double d, joint_state_data.effort){
            std::cout << d << " " << std::endl;
        }
    }
}

void FriExample::getCartesianWrench(){
    geometry_msgs::Wrench cart_wrench_data;
    RTT::FlowStatus cart_wrench_fs = iport_cart_wrench.read(cart_wrench_data);
    if(cart_wrench_fs == RTT::NewData){
        std::cout << "Cartesian Force at end effector" << std::endl;
        std::cout << cart_wrench_data.force << std::endl;
        std::cout << "Cartesian torque at end effector" << std::endl;
        std::cout << cart_wrench_data.torque << std::endl;
    }
}

void FriExample::getJacobian(){
    KDL::Jacobian jacobian_data;
    RTT::FlowStatus jacobian_fs = iport_jacobian.read(jacobian_data);
    if(jacobian_fs == RTT::NewData){
        std::cout << "Jacobian: " << std::endl;
        std::cout << jacobian_data.data << std::endl;
    }
}

void FriExample::getMassMatrix(){
    Eigen::Matrix<double, 7, 7> mass_matrix;
    RTT::FlowStatus mass_matrix_fs = iport_mass_matrix.read(mass_matrix);
    if(mass_matrix_fs == RTT::NewData){
        std::cout << "Mass matrix " << std::endl;
        std::cout << mass_matrix << std::endl;
    }
}

void FriExample::sendJointPositions(std::vector<double> &command){
    if(command.size() != 7){
        std::cout << "Wrong vector size" << std::endl;
        return;
    }
    else{
        motion_control_msgs::JointPositions joint_position_command;
        joint_position_command.positions = command;

        oport_joint_position.write(joint_position_command);
    }
}

void FriExample::sendJointVelocities(std::vector<double> &command){
    if(!requiresControlMode(10)){
        return;
    }

    if(command.size() != 7){
        std::cout << "Wrong vector size" << std::endl;
        return;
    }
    else{
        motion_control_msgs::JointVelocities joint_vel_command;
        joint_vel_command.velocities = command;

        oport_joint_velocities.write(joint_vel_command);
    }
}

void FriExample::sendJointTorque(std::vector<double> &command){
    if(!requiresControlMode(30)){
        return;
    }

    if(command.size() != 7){
        std::cout << "Wrong vector size" << std::endl;
        return;
    }
    else{
        motion_control_msgs::JointEfforts joint_eff_command;
        joint_eff_command.efforts = command;

        oport_joint_efforts.write(joint_eff_command);
    }
}

void FriExample::sendJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
    if(stiffness.size() != 7 || damping.size() != 7){
        std::cout << "Wrong vector size, should be 7,7" << std::endl;
        return;
    }
    else{
        lwr_fri::FriJointImpedance joint_impedance_command;
        for(unsigned int i = 0; i < 7; i++){
            joint_impedance_command.stiffness[i] = stiffness[i];
            joint_impedance_command.damping[i] = damping[i];
        }

        oport_joint_impedance.write(joint_impedance_command);
    }
}

void FriExample::sendCartesianPose(std::vector<double> &position, std::vector<double> &orientation){
    if(!requiresControlMode(20)){
        return;
    }
    if(position.size() != 3 || orientation.size() != 4){
        std::cout << "Wrong vector size, should be [x, y, z], [qx, qy, qz, qw]" << std::endl;
        return;
    }
    else{
        geometry_msgs::Pose cart_pos_command;
        cart_pos_command.position.x = position[0];
        cart_pos_command.position.y = position[1];
        cart_pos_command.position.z = position[2];
        cart_pos_command.orientation.x = orientation[0];
        cart_pos_command.orientation.y = orientation[1];
        cart_pos_command.orientation.z = orientation[2];
        cart_pos_command.orientation.w = orientation[3];

        oport_cartesian_pose.write(cart_pos_command);
    }
}

void FriExample::sendCartesianVel(std::vector<double> &linear, std::vector<double> &angular){
    if(!requiresControlMode(20)){
        return;
    }
    if(linear.size() != 3 || angular.size() != 3){
        std::cout << "Wrong vector size, should be [x, y, z], [rx, ry, rz]" << std::endl;
        return;
    }
    else{
        geometry_msgs::Twist cart_vel_command;
        cart_vel_command.linear.x = linear[0];
        cart_vel_command.linear.y = linear[1];
        cart_vel_command.linear.z = linear[2];
        cart_vel_command.angular.x = angular[0];
        cart_vel_command.angular.y = angular[1];
        cart_vel_command.angular.z = angular[2];

        oport_cartesian_twist.write(cart_vel_command);
    }
}

void FriExample::sendCartesianWrench(std::vector<double> &force, std::vector<double> &torque){
    if(!requiresControlMode(20)){
        return;
    }
    if(force.size() != 3 || torque.size() != 3){
        std::cout << "Wrong vector size, should be [fx, fy, fz], [tx, ty, tz]" << std::endl;
        return;
    }
    else{
        geometry_msgs::Wrench cart_wrench_command;
        cart_wrench_command.force.x = force[0];
        cart_wrench_command.force.y = force[1];
        cart_wrench_command.force.z = force[2];
        cart_wrench_command.torque.x = torque[0];
        cart_wrench_command.torque.y = torque[1];
        cart_wrench_command.torque.z = torque[2];

        oport_cartesian_wrench.write(cart_wrench_command);
    }
}

void FriExample::sendCartesianImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
    if(!requiresControlMode(20)){
        return;
    }
    if(stiffness.size() != 6 || damping.size() != 6){
        std::cout << "Wrong vector size, should be [x, y, z, rx, ry, rz]" << std::endl;
        return;
    }
    else{
        lwr_fri::CartesianImpedance cart_impedance;
        cart_impedance.stiffness.linear.x = stiffness[0];
        cart_impedance.stiffness.linear.y = stiffness[1];
        cart_impedance.stiffness.linear.z = stiffness[2];
        cart_impedance.stiffness.angular.x = stiffness[3];
        cart_impedance.stiffness.angular.y = stiffness[4];
        cart_impedance.stiffness.angular.z = stiffness[5];
        cart_impedance.damping.linear.x = damping[0];
        cart_impedance.damping.linear.y = damping[1];
        cart_impedance.damping.linear.z = damping[2];
        cart_impedance.damping.angular.x = damping[3];
        cart_impedance.damping.angular.y = damping[4];
        cart_impedance.damping.angular.z = damping[5];

        oport_cartesian_impedance.write(cart_impedance);
    }
}

ORO_CREATE_COMPONENT(FriExample)
