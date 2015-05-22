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
namespace lwr{
    
    class LWRSim : public RTT::TaskContext{
    public:
        LWRSim(std::string const& name):RTT::TaskContext(name),
        n_joints_(0),
        peer(NULL),
        prop_joint_offset(LBR_MNJ, 0.0),
        urdf_str_(""),
        kp_(LBR_MNJ,250.0),
        kd_(LBR_MNJ,0.7),
        kg_(LBR_MNJ,1.0),
        robot_name_("lwr_sim"),
        velocity_smoothing_factor_(.95),
        root_link("lwr_sim/link_0"),
        tip_link("lwr_sim/link_7"),
        use_sim_clock(true)
        {

            this->addProperty("fri_port", prop_fri_port).doc("");
            this->addProperty("joint_offset", prop_joint_offset).doc("");
            
            this->addProperty("root_link", root_link).doc("");
            this->addProperty("tip_link", tip_link).doc("");
            
            this->addProperty("use_sim_clock", use_sim_clock).doc("");
            
            //this->addAttribute("fromKRL", m_fromKRL);
            //this->addAttribute("toKRL", m_toKRL);
            
            this->addProperty("robot_description",urdf_str_)
              .doc("The URDF of the Kuka");
            this->addProperty("n_joints",n_joints_);
            
            robot_name_ = this->getName();
            
            this->addProperty("robot_name",robot_name_).doc("The name of the robot lwr/lwr_sim");

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

            this->ports()->addPort("toKRL",port_ToKRL).doc("");
            //this->ports()->addPort("KRL_CMD", port_KRL_CMD).doc("");
            this->ports()->addPort("fromKRL",port_FromKRL).doc("");
            
            this->ports()->addPort("CartesianWrench", port_CartesianWrench).doc("");
            this->ports()->addPort("CartesianWrenchStamped", port_CartesianWrenchStamped).doc("");
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
            this->ports()->addPort("JointTorqueRaw", port_JointTorqueRaw).doc("");
            this->ports()->addPort("JointPositionFRIOffset", port_JointPositionFRIOffset).doc("");

            this->ports()->addPort("JointState",port_JointState).doc("");
            this->ports()->addPort("JointStateCommand",port_JointStateCommand).doc("");
            this->ports()->addPort("JointStateFiltered",port_JointStateFiltered).doc("");
            this->ports()->addPort("JointStateGravity",port_JointStateGravity).doc("");
            
            
            this->addProperty("kp",kp_);
            this->addProperty("kd",kd_);
            this->addProperty("kg",kg_);
            this->addProperty("smoothing_factor",velocity_smoothing_factor_);
            this->addOperation("setImpedance",&LWRSim::setImpedance,this,RTT::OwnThread);
            this->addOperation("setGravityMode",&LWRSim::setGravityMode,this,RTT::OwnThread);
            this->addOperation("resetImpedanceGains",&LWRSim::resetImpedanceGains,this,RTT::OwnThread);
            this->provides("debug")->addAttribute("read_start",this->read_start);
            this->provides("debug")->addAttribute("write_start",this->write_start);
            this->provides("debug")->addAttribute("read_duration",this->read_duration);
            this->provides("debug")->addAttribute("id_duration",this->id_duration);
            this->provides("debug")->addAttribute("fk_duration",this->fk_duration);
            this->provides("debug")->addAttribute("ik_duration",this->ik_duration);
            this->provides("debug")->addAttribute("write_duration",this->write_duration);
            this->provides("debug")->addAttribute("updatehook_duration",this->updatehook_duration);

        }
        bool configureHook();
        void updateHook();
        ~LWRSim(){};
    public:
        void resetImpedanceGains();
        bool setGravityMode(){
            return this->setImpedance(std::vector<double>(LBR_MNJ,0),std::vector<double>(LBR_MNJ,0));
        }
        bool setImpedance(const std::vector< double >& stiffness, const std::vector< double >& damping);
        bool torqueSafetyCheck(const std::vector<double>& torque);
        bool velocitySafetyCheck(const std::vector<double>& velocity);
        bool positionSafetyCheck(const std::vector<double>& position);
        bool safetyChecks(const std::vector<double>& position,const std::vector<double>& velocity,const std::vector<double>& torque);
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
        RTT::OutputPort<Eigen::VectorXd > port_JointTorque;
        RTT::OutputPort<Eigen::VectorXd > port_GravityTorque;
        RTT::OutputPort<Eigen::VectorXd > port_JointPosition;
        RTT::OutputPort<Eigen::VectorXd > port_JointTorqueRaw;
        RTT::OutputPort<Eigen::VectorXd > port_JointPositionFRIOffset;

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
        Eigen::VectorXd jnt_pos_fri_offset;
        Eigen::VectorXd jnt_pos_old_;
        Eigen::VectorXd jnt_trq_;
        Eigen::VectorXd jnt_trq_raw_;
        Eigen::VectorXd grav_trq_;
        Eigen::VectorXd jnt_vel_;

        Eigen::VectorXd jnt_pos_cmd_;
        Eigen::VectorXd jnt_trq_cmd_;

        KDL::Jacobian jac_;

        KDL::Frame T_old_;
        geometry_msgs::PoseStamped cart_pos_stamped_;
        geometry_msgs::Pose cart_pos_, cart_pos_cmd_;
        geometry_msgs::Wrench cart_wrench_, cart_wrench_cmd_;
        geometry_msgs::WrenchStamped cart_wrench_stamped_;
        geometry_msgs::Twist cart_twist_;
        Eigen::MatrixXd mass_;
        lwr_fri::FriJointImpedance jnt_imp_cmd_;
        lwr_fri::CartesianImpedance cart_imp_cmd_;

        int m_control_mode;
        std::string joint_names_prefix;
        uint16_t counter, fri_state_last;

        tFriIntfState fri_state;
        tFriRobotState robot_state;
        
        tFriKrlData fri_from_krl;
        tFriKrlData fri_to_krl;
        
        RTT::TaskContext* peer;
        ros::Time now;
        double read_start,write_start;
        double read_duration,write_duration,updatehook_duration;
        double id_duration,ik_duration,fk_duration;
        unsigned int n_joints_;
        std::string urdf_str_;
        urdf::Model urdf_model_;
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;
        KDL::Chain kukaLWR_DHnew;
        
        
        boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
        boost::scoped_ptr<KDL::ChainIkSolverVel_pinv_nso> ik_solver_vel;
        boost::scoped_ptr<KDL::ChainDynParam> id_dyn_solver;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
        boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_rne_solver;
         boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_rne_solver_add_;
        
        //! Control gains
        std::vector<double> kp_,kd_,kg_;
        double velocity_smoothing_factor_;
        std::string robot_name_;
        sensor_msgs::JointState joint_state_filtered_,joint_state_,joint_state_cmd_,joint_state_gravity_,joint_state_dyn_;
        RTT::OutputPort<sensor_msgs::JointState> port_JointState;
        RTT::OutputPort<sensor_msgs::JointState> port_JointStateFiltered;
        RTT::OutputPort<sensor_msgs::JointState> port_JointStateCommand;
        RTT::OutputPort<sensor_msgs::JointState> port_JointStateGravity;
        RTT::OutputPort<sensor_msgs::JointState> port_JointStateDynamics;
        std::string root_link;
        std::string tip_link;
        bool use_sim_clock;
    private:
    
    void initJointStateMsg(sensor_msgs::JointState& js,const unsigned int n_joints,const std::string& robot_name);
        //KDL Stuff
    KDL::Wrenches f_ext;
    KDL::Wrenches f_ext_add;
    KDL::JntArray G,qdot,qddot,jnt_trq_kdl_,jnt_trq_kdl_add_;
    KDL::Wrench cart_wrench_kdl_;
    KDL::JntArrayVel q;
    KDL::JntSpaceInertiaMatrix H;
    };
}
ORO_CREATE_COMPONENT(lwr::LWRSim)
#endif
