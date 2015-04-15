// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#include <rtt/Component.hpp>
#include <sensor_msgs/JointState.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <urdf/model.h>
#include <rtt_roscomm/rtt_rostopic.h>

class LWRStatePublisher : public RTT::TaskContext{
public:
    LWRStatePublisher(std::string const& name):
        robot_name_("lwr"),
        n_joints_(0),
        peer(NULL),
        cnt_(0),
        RTT::TaskContext(name){
        this->ports()->addPort("JointPosition", port_JointPosition).doc("");
        this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
        this->ports()->addPort("JointTorque", port_JointTorque).doc("");
        this->ports()->addPort("JointState", port_JointState).doc("");
        this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");


    }
    virtual bool configureHook(){
        if(this->hasPeer(robot_name_)){
            this->peer = this->getPeer(robot_name_);
            this->port_JointPosition.connectTo(this->peer->getPort("JointPosition"));
            this->port_JointVelocity.connectTo(this->peer->getPort("JointVelocity"));
            this->port_JointTorque.connectTo(this->peer->getPort("JointTorque"));
            RTT::Attribute<int> njoints = this->peer->getAttribute("n_joints");
            this->n_joints_ = njoints.get();
            //std::string urdf_str = this->peer->getAttribute<std::string>("robot_description")->get();
            //this->urdf.initString(urdf_str);
            this->port_JointTorqueCommand.connectTo(this->peer->getPort("JointPositionCommand"));

        }else{
            RTT::log(RTT::Warning)<<"Couldn't find gazebo peer"<<RTT::endlog();
            //return false;
        }

        this->port_JointState.createStream(rtt_roscomm::topic(robot_name_+"/joint_states"));

        this->joint_position.resize(n_joints_);
        this->joint_velocity.resize(n_joints_);
        this->joint_torque.resize(n_joints_);
        this->joint_torque_command.resize(n_joints_);

        this->joint_position.setZero();
        this->joint_velocity.setZero();
        this->joint_torque.setZero();
        this->joint_torque_command.setZero();

        for(unsigned i=0;i<n_joints_;++i)
        {
            std::ostringstream ss;
            ss << i;
            this->joint_state.name.push_back(robot_name_+"/"+robot_name_+"_"+ss.str()+"_joint");
            this->joint_state.position.push_back(0.0);
            this->joint_state.velocity.push_back(0.0);
            this->joint_state.effort.push_back(0.0);
        }
        port_JointState.setDataSample(this->joint_state);
        return true;
    }

    virtual void updateHook() {
        cnt_++;
        this->joint_position_fs = port_JointPosition.read(joint_position);
        this->joint_velocity_fs = port_JointVelocity.read(joint_velocity);
        this->joint_torque_fs = port_JointTorque.read(joint_torque);
        this->joint_state.header.stamp = rtt_rosclock::host_now();

        if(this->joint_position_fs == RTT::NewData
                && this->joint_velocity_fs == RTT::NewData
                && this->joint_torque_fs == RTT::NewData)
        {
            for(unsigned i=0;i<n_joints_;++i)
            {
                this->joint_state.position[i] = joint_position[i];
                this->joint_state.velocity[i] = joint_velocity[i];
                this->joint_state.effort[i] = joint_torque[i];
                if(i==1  || i==3)
                    this->joint_torque_command[i] = 50*3.14/180.0*sin(cnt_*this->getPeriod()*double(i));
                else
                    this->joint_torque_command[i] = 0.0;
            }

        }


        if(this->joint_state.header.stamp.isZero()
                && (this->port_JointPosition.connected()
                    && this->joint_position_fs == RTT::NewData))
        {
            RTT::log(RTT::Error)<<"rtt_rosclock::host_now() returned 0, please check if /clock is published"<<RTT::endlog();
            this->joint_state.header.stamp = rtt_rosclock::rtt_now();
        }
        this->port_JointState.write(joint_state);

        this->port_JointTorqueCommand.write(joint_torque_command);


    }

public:
    sensor_msgs::JointState joint_state;
    RTT::OutputPort<sensor_msgs::JointState> port_JointState;
    RTT::InputPort<Eigen::VectorXd > port_JointPosition;
    RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
    RTT::InputPort<Eigen::VectorXd > port_JointTorque;

    RTT::OutputPort<Eigen::VectorXd > port_JointTorqueCommand;

    RTT::FlowStatus joint_position_fs;
    RTT::FlowStatus joint_velocity_fs;
    RTT::FlowStatus joint_torque_fs;
    Eigen::VectorXd joint_position;
    Eigen::VectorXd joint_velocity;
    Eigen::VectorXd joint_torque;

    Eigen::VectorXd joint_torque_command;

    RTT::TaskContext* peer;
    ros::Time now;
    int n_joints_;
    std::string robot_name_;
    int cnt_;
    urdf::Model urdf;
};

ORO_CREATE_COMPONENT(LWRStatePublisher)
