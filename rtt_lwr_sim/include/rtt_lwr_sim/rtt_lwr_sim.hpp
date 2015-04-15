// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#ifndef LWR_SIM_HPP
#define LWR_SIM_HPP
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include <tf_conversions/tf_kdl.h>

#include <Eigen/Dense>

#include <vector>

#include <sensor_msgs/JointState.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>
#include <rtt_roscomm/rtt_rostopic.h>
#include <urdf/model.h>
#include <rtt_rosparam/rosparam.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainidsolver_vereshchagin.hpp>
namespace lwr{
enum CommandType {
    EFFORT,
    VELOCITY,
    POSITION
};
    class LWRSim : public RTT::TaskContext{
    public:
        LWRSim(std::string const& name):RTT::TaskContext(name),
        n_joints_(0),
        peer(NULL),
        prop_joint_offset(7, 0.0),
        urdf_str_(""),
        kp_(2000.0),
        kd_(10.0),
        robot_name_("lwr"),
        velocity_smoothing_factor_(.95),
        command_type_(EFFORT)
        {

            this->addProperty("fri_port", prop_fri_port).doc("");
            this->addProperty("joint_offset", prop_joint_offset).doc("");
            this->addProperty("robot_description",urdf_str_)
              .doc("The URDF of the Kuka");
            this->addAttribute("n_joints",n_joints_);
            this->addAttribute("robot_name",robot_name_);

            this->ports()->addPort("JointPositionGazeboCommand", port_JointPositionGazeboCommand).doc("");
            this->ports()->addPort("JointVelocityGazeboCommand", port_JointVelocityGazeboCommand).doc("");
            this->ports()->addPort("JointTorqueGazeboCommand", port_JointTorqueGazeboCommand).doc("");

            this->ports()->addPort("JointPositionGazebo", port_JointPositionGazebo).doc("");
            this->ports()->addPort("JointVelocityGazebo", port_JointVelocityGazebo).doc("");
            this->ports()->addPort("JointTorqueGazebo", port_JointTorqueGazebo).doc("");

            this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
            this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
            this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
            this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
            this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
            this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

//this->ports()->addPort("KRL_CMD", port_KRL_CMD).doc("");

            this->ports()->addPort("CartesianWrench", port_CartesianWrench).doc("");
            this->ports()->addPort("RobotState", port_RobotState).doc("");
            this->ports()->addPort("FRIState", port_FRIState).doc("");
            this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
            this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");
            this->ports()->addPort("CartesianPosition", port_CartesianPosition).doc("");
            this->ports()->addPort("MassMatrix", port_MassMatrix).doc("");
            this->ports()->addPort("Jacobian", port_Jacobian).doc("");
            this->ports()->addPort("JointTorque", port_JointTorque).doc("");
            this->ports()->addPort("GravityTorque", port_GravityTorque).doc("");
            this->ports()->addPort("JointPosition", port_JointPosition).doc("");

            this->addProperty("kp",kp_);
            this->addProperty("kd",kd_);
            this->addProperty("smoothing_factor",velocity_smoothing_factor_);
            this->provides("debug")->addAttribute("read_start",this->read_start);
            this->provides("debug")->addAttribute("write_start",this->write_start);
            this->provides("debug")->addAttribute("read_duration",this->read_duration);
            this->provides("debug")->addAttribute("write_duration",this->write_duration);
            this->provides("debug")->addAttribute("updatehook_duration",this->updatehook_duration);

        }
        virtual bool configureHook();
        virtual void updateHook();

    public:
        RTT::InputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
        RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
        RTT::InputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
        RTT::InputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
        RTT::InputPort<Eigen::VectorXd > port_JointPositionCommand;
        RTT::InputPort<Eigen::VectorXd > port_JointTorqueCommand;
        RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;

        RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench;
        RTT::OutputPort<tFriRobotState > port_RobotState;
        RTT::OutputPort<tFriIntfState > port_FRIState;
        RTT::OutputPort<Eigen::VectorXd > port_JointVelocity;
        RTT::OutputPort<geometry_msgs::Twist > port_CartesianVelocity;
        RTT::OutputPort<geometry_msgs::Pose > port_CartesianPosition;
        RTT::OutputPort<Eigen::MatrixXd > port_MassMatrix;
        RTT::OutputPort<KDL::Jacobian > port_Jacobian;
        RTT::OutputPort<Eigen::VectorXd > port_JointTorque;
        RTT::OutputPort<Eigen::VectorXd > port_GravityTorque;
        RTT::OutputPort<Eigen::VectorXd > port_JointPosition;

        int prop_fri_port;
        std::vector<double> prop_joint_offset;

        RTT::InputPort<std::vector<double> > port_JointPositionGazebo;
        RTT::InputPort<std::vector<double> > port_JointVelocityGazebo;
        RTT::InputPort<std::vector<double> > port_JointTorqueGazebo;
        RTT::FlowStatus jnt_pos_fs;
        RTT::FlowStatus jnt_vel_fs;
        RTT::FlowStatus jnt_trq_fs;

        RTT::OutputPort<std::vector<double> > port_JointPositionGazeboCommand;
        RTT::OutputPort<std::vector<double> > port_JointVelocityGazeboCommand;
        RTT::OutputPort<std::vector<double> > port_JointTorqueGazeboCommand;

        std::vector<double> joint_position_gazebo;
        std::vector<double> joint_velocity_gazebo;
        std::vector<double> joint_torque_gazebo;

        std::vector<double> joint_position_gazebo_cmd;
        std::vector<double> joint_velocity_gazebo_cmd;
        std::vector<double> joint_torque_gazebo_cmd;

        Eigen::VectorXd jnt_pos_;
        Eigen::VectorXd jnt_pos_old_;
        Eigen::VectorXd jnt_trq_;
        Eigen::VectorXd grav_trq_;
        Eigen::VectorXd jnt_vel_;

        Eigen::VectorXd jnt_pos_cmd_;
        Eigen::VectorXd jnt_trq_cmd_;

        KDL::Jacobian jac_;

        KDL::Frame T_old_;
        geometry_msgs::Pose cart_pos_, cart_pos_cmd_;
        geometry_msgs::Wrench cart_wrench_, cart_wrench_cmd_;
        geometry_msgs::Twist cart_twist_;
        Eigen::MatrixXd mass_;
        lwr_fri::FriJointImpedance jnt_imp_cmd_;
        lwr_fri::CartesianImpedance cart_imp_cmd_;

        int m_control_mode;
        std::string joint_names_prefix;
        uint16_t counter, fri_state_last;

        tFriMsrData m_msr_data;
        tFriCmdData m_cmd_data;

        RTT::TaskContext* peer;
        ros::Time now,read_start,write_start;
        ros::Duration read_duration,write_duration,updatehook_duration;
        int n_joints_;
        std::string urdf_str_;
        urdf::Model urdf_model_;
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;
        KDL::Chain kukaLWR_DHnew;
        boost::shared_ptr<KDL::ChainIdSolver_RNE> id_rne_solver;
        boost::shared_ptr<KDL::ChainIdSolver_Vereshchagin> id_ver_solver;
        //! Control gains
        double kp_,kd_,velocity_smoothing_factor_;
        CommandType command_type_;
        std::string robot_name_;
        sensor_msgs::JointState joint_state_filtered_;
        sensor_msgs::JointState joint_state_;
        sensor_msgs::JointState joint_state_cmd_;
        RTT::OutputPort<sensor_msgs::JointState> port_JointState;
        RTT::OutputPort<sensor_msgs::JointState> port_JointStateFiltered;
        RTT::OutputPort<sensor_msgs::JointState> port_JointStateCommand;
    };
}
ORO_CREATE_COMPONENT(lwr::LWRSim)
#endif
