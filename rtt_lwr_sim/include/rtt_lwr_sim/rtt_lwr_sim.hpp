// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#ifndef LWR_SIM_HPP
#define LWR_SIM_HPP
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

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

#include <boost/thread.hpp>
#include <std_srvs/Empty.h>

using namespace KDL;
using namespace RTT;
using namespace Eigen;
using namespace RTT::corba;


namespace lwr{
    
    class LWRSim : public RTT::TaskContext{
    public:
        LWRSim(std::string const& name);
        bool configureHook();
        void updateHook();
        
    protected:
        bool waitForROSService(std::string service_name);
        void setJointImpedanceMode();
        void setCartesianImpedanceMode();
        void resetJointImpedanceGains();
        void setInitialJointPosition(const std::vector<double>& joint_position_cmd);
        void resetCartesianImpedanceGains();
        void setLinkGravityMode(const std::string& link_name,bool gravity_mode);
        void gazeboUpdateHook(gazebo::physics::ModelPtr model);
        bool readyService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res);
        bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
        bool setGravityMode();
        bool setJointImpedance(const Eigen::VectorXd& stiffness, const Eigen::VectorXd& damping);
        bool setCartesianImpedance(const Eigen::Matrix<double,6,1> & cart_stiffness, const Eigen::Matrix<double,6,1> & cart_damping);
        bool isCommandMode();
        
        RTT::OutputPort<bool> port_sync_status;
        RTT::InputPort<bool> port_sync_cmd;
        
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
        
        bool set_joint_pos_no_dynamics_;
        bool set_brakes_;
        
        std::vector<double> prop_joint_offset;
                                              
        RTT::InputPort<sensor_msgs::JointState > port_JointStateGazebo;
                
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
                        jnt_trq_gazebo_cmd_,
                        jnt_pos_brakes_,
                        jnt_pos_no_dyn_;
                        
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
        KDL::Jacobian jac_;

        geometry_msgs::PoseStamped cart_pos_stamped_;
        geometry_msgs::Pose cart_pos_, cart_pos_cmd_;
        geometry_msgs::Wrench cart_wrench_, cart_wrench_cmd_;
        geometry_msgs::WrenchStamped cart_wrench_stamped_;
        geometry_msgs::Twist cart_twist_;
        lwr_fri::FriJointImpedance jnt_imp_cmd_,jnt_imp_;
        lwr_fri::CartesianImpedance cart_imp_cmd_,cart_imp_;

        std::string joint_names_prefix;

        tFriIntfState fri_state;
        tFriRobotState robot_state;
        
        tFriKrlData fri_from_krl;
        tFriKrlData fri_to_krl;
        
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
        
        sensor_msgs::JointState joint_states_,
                                joint_states_cmd_,
                                joint_states_dyn_;
                                
        RTT::OutputPort<sensor_msgs::JointState> port_JointStates,
                                                 port_JointStatesCommand,
                                                 port_JointStatesDynamics;
                                                 
        std::string root_link;
        
        std::string tip_link;
        
        bool use_sim_clock;

        bool safetyChecks(const Eigen::VectorXd& position,const Eigen::VectorXd& velocity,const Eigen::VectorXd& torque);
        bool safetyCheck(const Eigen::VectorXd& v, const Eigen::VectorXd& limits,const std::string& name="");
        void updateJointImpedance(const lwr_fri::FriJointImpedance& impedance);
        void updateCartesianImpedance(const lwr_fri::CartesianImpedance& cart_impedance);
        
        //KDL Stuff
        KDL::Wrenches f_ext_;
        
        KDL::JntArray jnt_pos_kdl_,
                      jnt_trq_grav_kdl_,
                      jnt_vel_kdl_,
                      jnt_acc_kdl_,
                      jnt_trq_kdl_,
                      jnt_trq_coriolis_kdl_;
                      
        KDL::Wrench cart_wrench_kdl_;
        KDL::JntSpaceInertiaMatrix mass_kdl_;
        bool init_pos_requested;
        double period_sim_;
        double service_timeout_s;
        std::vector<int> joints_idx_;
        std::map<gazebo::physics::LinkPtr,bool> gravity_mode_;
        gazebo::physics::Joint_V gazebo_joints_;
        gazebo::physics::Link_V model_links_;
        std::vector<std::string> joint_names_;
        unsigned int nb_no_data_;
    };
}

#endif
