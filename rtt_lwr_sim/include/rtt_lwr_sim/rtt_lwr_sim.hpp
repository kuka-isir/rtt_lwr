// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#ifndef LWR_SIM_HPP
#define LWR_SIM_HPP
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
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

#include <rtt_ros_kdl_tools/tools.hpp>
#include <boost/scoped_ptr.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_msg.h>

#define TORAD 3.141592653589793/180.0

inline float clamp(const float& x, const float& a, const float& b)

{
    return x < a ? a : (x > b ? b : x);
}

namespace lwr{
    
    class LWRSim : public RTT::TaskContext{
    public:
        LWRSim(std::string const& name);
        bool configureHook();
        void updateHook();
        
    protected:
        bool connectToGazeboCORBA(const std::string& gazebo_deployer_name,const std::string& gazebo_robot_comp_name);
        bool waitForROSService(std::string service_name);
        void setJointImpedanceMode();
        void setCartesianImpedanceMode();
        void resetJointImpedanceGains();
        void setInitialJointPosition(const std::vector<double> j_init);
        void resetCartesianImpedanceGains();
        
        bool setGravityMode();
        bool setJointImpedance(const Eigen::VectorXd& stiffness, const Eigen::VectorXd& damping);
        bool setCartesianImpedance(const Eigen::Matrix<double,6,1> & cart_stiffness, const Eigen::Matrix<double,6,1> & cart_damping);
        
        RTT::InputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
        RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
        RTT::InputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
        RTT::InputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
        RTT::InputPort<Eigen::VectorXd > port_JointPositionCommand;
        RTT::InputPort<Eigen::VectorXd > port_JointTorqueCommand;
        //RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;
        RTT::InputPort<tFriKrlData > port_ToKRL;
        //RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;
        
        RTT::OutputPort<tFriKrlData > port_FromKRL;
        RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench;
        RTT::OutputPort<geometry_msgs::WrenchStamped > port_CartesianWrenchStamped;
        RTT::OutputPort<tFriRobotState > port_RobotState;
        RTT::OutputPort<tFriIntfState > port_FRIState;
        RTT::OutputPort<Eigen::VectorXd > port_JointVelocity;
        RTT::OutputPort<geometry_msgs::Twist > port_CartesianVelocity;
        RTT::OutputPort<geometry_msgs::Pose > port_CartesianPosition;
        RTT::OutputPort<geometry_msgs::PoseStamped > port_CartesianPositionStamped;
        RTT::OutputPort<Eigen::MatrixXd > port_MassMatrix;
        RTT::OutputPort<KDL::Jacobian > port_Jacobian;
        RTT::OutputPort<Eigen::VectorXd > port_JointTorque,
                                          port_GravityTorque,
                                          port_JointPosition,
                                          port_JointTorqueRaw,
                                          port_JointPositionFRIOffset;
        bool connect_to_rtt_gazebo_at_configure,using_corba;
        int prop_fri_port;
        double dr_max_;
        std::vector<double> prop_joint_offset;

        RTT::InputPort<std::vector<double> > port_JointPositionGazebo,
                                             port_JointVelocityGazebo,
                                             port_JointTorqueGazebo;
        RTT::FlowStatus jnt_pos_cmd_fs,
                        jnt_trq_cmd_fs,
                        cart_imp_cmd_fs,
                        cart_wrench_cmd_fs,
                        cart_pos_cmd_fs;

        RTT::OutputPort<std::vector<double> > port_JointPositionGazeboCommand,
                                              port_JointVelocityGazeboCommand,
                                              port_JointTorqueGazeboCommand;
                                              
        RTT::InputPort<sensor_msgs::JointState > port_JointStateGazebo;
        
        std::vector<double> joint_position_gazebo,
                            joint_velocity_gazebo,
                            joint_torque_gazebo,
                            joint_position_gazebo_cmd,
                            joint_velocity_gazebo_cmd,
                            joint_torque_gazebo_cmd;
        
        Eigen::VectorXd pos_limits_,
                        vel_limits_,
                        trq_limits_;
                            
        Eigen::VectorXd jnt_pos_,
                        jnt_pos_fri_offset,
                        jnt_pos_old_,
                        jnt_trq_,
                        jnt_trq_raw_,
                        grav_trq_,
                        jnt_vel_,
                        jnt_pos_cmd_,
                        jnt_trq_cmd_,
                        jnt_trq_gazebo_cmd_;
                        
        Eigen::Matrix<double,6,1>  X_,
                                   X_cmd_,
                                   Xd_,
                                   Xd_cmd_,
                                   X_err_,
                                   Xd_err_,
                                   F_cmd_;
        KDL::Twist ee_twist_kdl_,
                   ee_twist_des_kdl_,
                   ee_twist_diff_kdl_,
                   ee_frame_diff_kdl_;
        
        KDL::Frame ee_frame_kdl_,
                   ee_frame_des_kdl_;
                   
        KDL::FrameVel ee_framevel_kdl_;           
        tf::Matrix3x3 quat_m;
        tf::Quaternion quat_tf;
        double roll, pitch, yaw;
        KDL::Jacobian jac_;

        KDL::Frame T_old_;
        geometry_msgs::PoseStamped cart_pos_stamped_;
        geometry_msgs::Pose cart_pos_, cart_pos_cmd_;
        geometry_msgs::Wrench cart_wrench_, cart_wrench_cmd_;
        geometry_msgs::WrenchStamped cart_wrench_stamped_;
        geometry_msgs::Twist cart_twist_;
        Eigen::MatrixXd mass_;
        lwr_fri::FriJointImpedance jnt_imp_cmd_,jnt_imp_;
        lwr_fri::CartesianImpedance cart_imp_cmd_,cart_imp_;

        int m_control_mode;
        std::string joint_names_prefix;
        uint16_t counter, fri_state_last;

        tFriIntfState fri_state;
        tFriRobotState robot_state;
        
        tFriKrlData fri_from_krl;
        tFriKrlData fri_to_krl;
        
        ros::Time now;
        double read_start,write_start;
        double read_duration,write_duration,updatehook_duration;
        double id_duration,ik_duration,fk_duration;
        unsigned int n_joints_;
        std::string urdf_str_;
        urdf::Model urdf_model_;
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;        
        
        boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
        boost::scoped_ptr<KDL::ChainDynParam> id_dyn_solver;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
        boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_rne_solver;
        
        bool safety_checks_;

        KDL::Vector gravity_vector;
        
        //! Control gains
        Eigen::VectorXd kp_,
                        kd_,
                        kg_,
                        kp_default_,
                        kd_default_;
                        
        Eigen::Matrix<double,6,1>   kc_,
                                    kcd_,
                                    kc_default_,
                                    kcd_default_;        
        std::string robot_name_;
        
        sensor_msgs::JointState joint_state_,
                                joint_state_cmd_,
                                joint_state_gravity_,
                                joint_state_dyn_;
                                
        RTT::OutputPort<sensor_msgs::JointState> port_JointState,
                                                 port_JointStateCommand,
                                                 port_JointStateGravity,
                                                 port_JointStateDynamics;
                                                 
        std::string root_link;
        
        std::string tip_link;
        
        bool use_sim_clock;

        bool safetyChecks(const Eigen::VectorXd& position,const Eigen::VectorXd& velocity,const Eigen::VectorXd& torque);
        bool safetyCheck(const Eigen::VectorXd& v, const Eigen::VectorXd& limits,const std::string& name="");
        void updateJointImpedance(const lwr_fri::FriJointImpedance& impedance);
        void updateCartesianImpedance(const lwr_fri::CartesianImpedance& cart_impedance);
        
        //KDL Stuff
        KDL::Wrenches f_ext;
        KDL::JntArray G,qdot,qddot,jnt_trq_kdl_,jnt_trq_coriolis_kdl_;
        KDL::Wrench cart_wrench_kdl_;
        KDL::JntArrayVel q;
        KDL::JntSpaceInertiaMatrix H;
        bool init_pos_requested;
        double period_sim_;
        double service_timeout_s;
    };
}
ORO_CREATE_COMPONENT(lwr::LWRSim)
#endif
