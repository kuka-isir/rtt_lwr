// Copyright 2015 ISIR-CNRS
// Author: Antoine Hoarau

#ifndef __RTT_LWR_ABSTRACT_HPP__
#define __RTT_LWR_ABSTRACT_HPP__

#include <rtt/RTT.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>
#include <rtt_rosparam/rosparam.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>
#include <rtt_roscomm/rtt_rostopic.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <tf_conversions/tf_kdl.h>

#include <Eigen/Dense>

#include <vector>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <rtt_ros_kdl_tools/tools.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>

namespace lwr{
    // Custom FRI/KRL Cmds
    static const fri_int32_t FRI_START = 1;
    static const fri_int32_t FRI_STOP = 2;
    static const fri_int32_t STOP_KRL_SCRIPT = 3;
    
class RTTLWRAbstract : public RTT::TaskContext{
  public:
      /**
     * @brief lwr or lwr_sim peer
     */
      RTT::TaskContext* peer;


    /**
     * @brief Shared arrays from the remote pc to the KRC
     */
    tFriKrlData fri_to_krl;

    /**
     * @brief Shared arrays from the KRC to the remote pc
     */
    tFriKrlData fri_from_krl;

    RTT::OutputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
    RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
    RTT::OutputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
    RTT::OutputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
    RTT::OutputPort<Eigen::VectorXd > port_JointPositionCommand;
    RTT::OutputPort<Eigen::VectorXd > port_JointTorqueCommand;
    //RTT::OutputPort<std_msgs::Int32 > port_KRL_CMD;
    RTT::InputPort<tFriKrlData > port_FromKRL;
    //RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;
        
    RTT::OutputPort<tFriKrlData > port_ToKRL;
    RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrench;
    RTT::InputPort<tFriRobotState > port_RobotState;
    RTT::InputPort<tFriIntfState > port_FRIState;
    RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
    RTT::InputPort<geometry_msgs::Twist > port_CartesianVelocity;
    RTT::InputPort<geometry_msgs::Pose > port_CartesianPosition;
    RTT::InputPort<Eigen::MatrixXd > port_MassMatrix;
    RTT::InputPort<KDL::Jacobian > port_Jacobian;
    RTT::InputPort<Eigen::VectorXd> port_JointTorque;
    RTT::InputPort<Eigen::VectorXd> port_GravityTorque;
    RTT::InputPort<Eigen::VectorXd> port_JointPosition;
    RTT::InputPort<Eigen::VectorXd> port_JointTorqueRaw;
    RTT::InputPort<Eigen::VectorXd> port_JointPositionFRIOffset;


    RTTLWRAbstract(std::string const& name);
    ~RTTLWRAbstract();
    tFriRobotState robot_state;
    tFriIntfState fri_state;
    geometry_msgs::Pose cart_pos, cart_pos_cmd;
    geometry_msgs::Wrench cart_wrench, cart_wrench_cmd;
    geometry_msgs::Twist cart_twist;
    Eigen::MatrixXd mass;
    lwr_fri::FriJointImpedance jnt_imp_cmd;
    lwr_fri::CartesianImpedance cart_imp_cmd;

    Eigen::VectorXd jnt_pos,
                    jnt_pos_old,
                    jnt_trq,
                    jnt_trq_raw,
                    jnt_pos_fri_offset,
                    jnt_grav,
                    jnt_vel,
                    jnt_pos_cmd,
                    jnt_trq_cmd,
                    jnt_vel_bdf;

    KDL::JntArray   jnt_pos_kdl,
                    jnt_vel_kdl,
                    coriolis_kdl,
                    gravity_kdl,
                    jnt_trq_kdl;
                    
    KDL::JntArrayVel jnt_pos_vel_kdl;
    KDL::Frame tool_in_base_frame;

    KDL::Wrenches f_ext_kdl;

    // Backward differentiation formula buffer : http://en.wikipedia.org/wiki/Backward_differentiation_formula
    boost::circular_buffer<Eigen::VectorXd> jnt_pos_bdf;

    KDL::Jacobian J;
    KDL::FrameVel tool_in_base_framevel;
    unsigned int n_joints;
    
    std::string root_link,tip_link,robot_name,robot_description;
    urdf::Model urdf_model;
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;        
        
    boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
    boost::scoped_ptr<KDL::ChainIkSolverVel_pinv_nso> ik_solver_vel;
    boost::scoped_ptr<KDL::ChainDynParam> id_dyn_solver;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_rne_solver;
    
    const unsigned int getNJoints()const{return n_joints;}

    bool sendJointCommand(RTT::OutputPort<Eigen::VectorXd>& port_cmd,const Eigen::VectorXd& jnt_cmd); 
    int getToolKRL();
    /** @brief Orocos Configure Hook
     * Initialization of the shared array between
     * KRC and remote pc
     * We choose by convention to trigger fri_start() in
     * the KRL program if $FRI_FRM_INT[1] == 1
     */
    bool configureHook();

    /** @brief Orocos Start Hook
     * Send arrays to KRC and call doStart()
     */
    bool startHook();

    /** @brief To implement if specific things have to be done when starting the component
     */
    virtual bool doStart();

    /** @brief Orocos Update hook
     */
    virtual void updateHook() = 0;

    /** @brief Orocos stop hook
     * Call doStop(), then put 2 in $FRI_FRM_INT[1]
     * which by our convention trigger fri_stop() in KRL program
     */
    void stopHook();

    /** @brief To implement if specific things have to be done when stoping the component
     */
    virtual void doStop();

    /** @brief Orocos Cleanup hook
     */
    virtual void cleanupHook();

    /** @brief define the lwr_fri peer name to share attributes toKrl and fromKrl
     * */
    void setPeer(std::string name);

    /** @brief Set control strategy
     */
    void setControlStrategy(const unsigned int mode);

    /** @brief Select the tool defined on the KRC
     */
    void setToolKRL(const unsigned int toolNumber);

    /** @brief Check if the selected control mode is the required one
     *  @param modeRequired : the required mode
     *  @return True : if the current mode match the required one
     */
    bool requiresControlMode(int modeRequired);

    /** @brief Get the FRI Mode
     *  @return : The value of the FRI_STATE enum
     */
    FRI_STATE getFRIMode();
    
    /** @brief Get the FRI Quality
     *  @return : The value of the FRI_QUALITY enum
     */
    FRI_QUALITY getFRIQuality();
    
    /** @brief Get the FRI Control mode
     *  @return: The current control mode from FRI_CTRL (joint impedance, cartesian impedance or joint position)
     */
    FRI_CTRL getFRIControlMode();
    
    /** @brief Ask KRL script for a friStop()
     */
    void friStop();

    /** @brief Ask KRL script for a friStart()
     */
    void friStart();

    /** @brief Estimate the velocity using the Backward Differentiation Formula
     */
    static void estimateVelocityBDF(unsigned int order,const double dt,const boost::circular_buffer<Eigen::VectorXd>& x_states,Eigen::VectorXd& xd);

    /** @brief Reset the array shared with the KRC
     */
    void friReset();

    /** @brief Ask KRL exit
     */
    void stopKrlScript();

    /** @brief Initialize the command that will be send to the robot
     */
    void initializeCommand();

    /** @brief Return the cartesian position of the tool center point in the robot base frame
     */
    bool getCartesianPosition(geometry_msgs::Pose& cart_position);

    /** @brief Return the Jacobian
     */
    bool getJacobian(KDL::Jacobian& jacobian);

    /** @brief Return the Mass Matrix
     */
    bool getMassMatrix(Eigen::MatrixXd& mass_matrix);

    /** @brief Return the gravity torque
     */
    bool getGravityTorque(Eigen::VectorXd& gravity_torque);
    /** @brief Return the estimated joint velocity
     */
    bool getJointVelocity(Eigen::VectorXd& joint_velocity);

    /** @brief Return the current configuration of the robot
     */
    bool getJointPosition(Eigen::VectorXd& joint_position);

    /** @brief Return the estimated external joint torque
     */
    bool getJointTorque(Eigen::VectorXd& joint_torque);
        /** @brief Return the actuator joint torque seem by the motor
     */
    bool getJointTorqueRaw(Eigen::VectorXd& joint_torque_raw);
    
    /** @brief Return the cartesian velocity of the Tooltip (Twist)
     */
    bool getCartesianVelocity(geometry_msgs::Twist& cart_twist);
    
    /** @brief Return the estimated external tool center point wrench
     */
    bool getCartesianWrench(geometry_msgs::Wrench& cart_wrench);

    /** @brief Send Joint position in radians
     */
    bool sendJointPosition(const Eigen::VectorXd& joint_position_cmd);
    /** @brief Send Joint Impedance gains
     */
    bool sendJointImpedance(const lwr_fri::FriJointImpedance& joint_impedance_cmd);
    /** @brief Send Joint Torque in N.m
     */
    bool sendJointTorque(const Eigen::VectorXd& joint_torque_cmd);
    /** @brief Send Cartesian position Command
     */
    bool sendCartesianPosition(const geometry_msgs::Pose& cartesian_pose);
    /** @brief Send Cartesian Wrench Command
     */
    bool sendCartesianWrench(const geometry_msgs::Wrench& cartesian_wrench);
    /** @brief Set the Position Control Mode 10
     */
    void setJointPositionControlMode(){setControlStrategy(10*FRI_CTRL_POSITION);}
        
    /** @brief Set the Impedance Control Mode 30
     */
    void setJointImpedanceControlMode(){setControlStrategy(10*FRI_CTRL_JNT_IMP);}
        
    /** @brief Set the Cartesian Impedance Control Mode 20
     */
    void setCartesianImpedanceControlMode(){setControlStrategy(10*FRI_CTRL_CART_IMP);}
    bool isCommandMode();
    bool isMonitorMode();
    bool isPowerOn();
    bool updateState(){
        bool res=true;
        res &= getJointPosition(jnt_pos);
        res &= getJointVelocity(jnt_vel);
        jnt_pos_kdl.data = jnt_pos;
        jnt_vel_kdl.data = jnt_vel;
        jnt_pos_vel_kdl.q.data = jnt_pos;
        jnt_pos_vel_kdl.qdot.data = jnt_vel;
        return res;
    }
    template<class T>
    RTT::FlowStatus getData(RTT::InputPort<T>& port,T& data)
    {
        if(port.connected() == false)
        {
            RTT::log(RTT::Error) << port.getName() << " not connected "<< RTT::endlog();
            return RTT::NoData;
        }
        return port.read(data);
    }
};
}
#endif
