// Copyright 2015 ISIR-CNRS
// Author: Antoine Hoarau

#ifndef __RTT_LWR_ABSTRACT_HPP__
#define __RTT_LWR_ABSTRACT_HPP__

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kuka_lwr_fri/friComm.h>
#include <rtt/Attribute.hpp>

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

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
#include <std_msgs/Int32.h>
#define BDFORDER 6

/*namespace Eigen{
typedef Matrix<double, LBR_MNJ, LBR_MNJ> MatrixXd;
typedef Matrix<double, LBR_MNJ, 1> VectorXd;
}*/

namespace lwr{
    static const fri_int32_t FRI_START = 1;
    static const fri_int32_t FRI_STOP = 2;
    static const fri_int32_t STOP_KRL_SCRIPT = 3;
    
class RTTLWRAbstract : public RTT::TaskContext{
  public:
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
    Eigen::MatrixXd M;
    lwr_fri::FriJointImpedance jnt_imp_cmd;
    lwr_fri::CartesianImpedance cart_imp_cmd;
    Eigen::VectorXd jnt_pos;
    Eigen::VectorXd jnt_pos_old;
    Eigen::VectorXd jnt_trq;
    Eigen::VectorXd jnt_trq_raw;
    Eigen::VectorXd jnt_pos_fri_offset;
    Eigen::VectorXd G;
    Eigen::VectorXd jnt_vel;
    std::string robot_name;
    // Backward differentiation formula buffer : http://en.wikipedia.org/wiki/Backward_differentiation_formula
    //std::vector<VectorXd> jnt_pos_bdf_;
    int BDFi;
    boost::circular_buffer<Eigen::VectorXd> jnt_pos_bdf;
    Eigen::VectorXd jnt_vel_bdf;
    static const double BDFCoeffs[];

    Eigen::VectorXd jnt_pos_cmd;
    Eigen::VectorXd jnt_trq_cmd;
    RTT::Attribute<tFriKrlData> m_fromKRL;
    RTT::Attribute<tFriKrlData> m_toKRL;
    KDL::Jacobian J;
    KDL::Frame T_old;

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
    void setControlStrategy(int mode);

    /** @brief Select the tool defined on the KRC
     */
    void setToolKRL(int toolNumber);

    /** @brief Check if the selected control mode is the required one
     *  @param modeRequired : the required mode
     *  @return True : if the current mode match the required one
     */
    bool requiresControlMode(int modeRequired);

    /** @brief Get the FRI Mode
     *  @return : The value of the FRI_STATE enum
     */
    FRI_STATE getFRIMode();

    /** @brief Ask KRL script for a friStop()
     */
    void friStop();

    /** @brief Ask KRL script for a friStart()
     */
    void friStart();

    /** @brief Estimate the velocity using the Backward Differentiation Formula
     */
    void estimateVelocityBDF(const double dt,const std::vector<double>& coeffs,const boost::circular_buffer<Eigen::VectorXd>& x,Eigen::VectorXd& xd)
    {
            for(size_t j=0;j<LBR_MNJ;++j)
            {
                double sum = 0.0 ;
                for(size_t i=0; i < coeffs.size(); ++i)
                {
                    size_t idx = (this->BDFi - i) % coeffs.size();
                    sum += coeffs[i]*x[idx][j];
                }
                xd[j] = sum/(dt*coeffs[0]);
            }
    }

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
    geometry_msgs::Pose getCartesianPosition();

    /** @brief Return the Jacobian
     */
    KDL::Jacobian getJacobian();

    /** @brief Return the Mass Matrix
     */
    Eigen::MatrixXd getMassMatrix();

    /** @brief Return the gravity torque
     */
    Eigen::VectorXd getGravityTorque();

    /** @brief Return the current configuration of the robot
     */
    Eigen::VectorXd getJointPosition();

    /** @brief Return the estimated external joint torque
     */
    Eigen::VectorXd getJointTorque();
        /** @brief Return the actuator joint torque
     */
    Eigen::VectorXd getJointTorqueAct();

    /** @brief Return the estimated external tool center point wrench
     */
    geometry_msgs::Wrench getCartesianWrench();

    bool connectRobotState();
    bool connectFRIState();
    bool connectJointPosition();
    bool connectJointPositionCommand();
    bool connectICmdJntPosFriOffset();
    bool connectIMsrCartPos();
    bool connectICmdCartPos();
    bool connectICmdCartPosFriOffset();
    bool connectIMsrJntVel();
    bool connectIMsrJntTrq();
    bool connectIEstExtJntTrq();
    bool connectIEstExtTcpWrench();
    bool connectIEvents();
    bool connectIMassMatrix();
    bool connectIJacobian();
    bool connectIGravity();
    

    /** @brief Connect joint position output ports with lwr_fri component
     */
    bool connectOJointPosition();

    /** @brief Connect joint velocities output ports with lwr_fri component
     */
    bool connectOJointVelocities();

    /** @brief Connect joint torque output ports with lwr_fri component
     */
    bool connectOJointTorque();

    /** @brief Connect cartesian pose output ports with lwr_fri component
     */
    bool connectOCartesianPose();

    /** @brief Connect cartesian pose output ports with lwr_fri component
     */
    bool connectOCartesianTwist();

    bool connectOCartesianWrench();
    bool connectODesJntImpedance();

    void disconnectPort(std::string portname);

    /** @brief Send Joint position in radians
     */
    void sendJointPosition(Eigen::VectorXd &qdes);

    /** @brief Send Joint velocities in radians/s
     */
    void sendJointVelocity(Eigen::VectorXd &qdotdes);

    /** @brief Send additionnal joint torque in Nm
     */
    void sendAddJointTorque(Eigen::VectorXd &tau);

    /** @brief Send desired tool center point cartesian pose: x, y, z(m), qw, qx, qy, qz
     */
    void sendCartesianPose(Eigen::VectorXd &pose);

    /** @brief Send desired tool center point cartesian twist: vx, vy, vz, wx, wy, wz
     */
    void sendCartesianTwist(Eigen::VectorXd &twist);

};
}
#endif
