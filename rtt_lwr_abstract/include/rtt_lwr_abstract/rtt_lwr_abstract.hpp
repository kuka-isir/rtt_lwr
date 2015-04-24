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

#define BDFORDER 6

typedef Eigen::Matrix<double, LBR_MNJ, LBR_MNJ> Matrix77d;
typedef Eigen::Matrix<double, LBR_MNJ, 1> Vector7d;
namespace lwr{
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
    tFriKrlData fri_frm_krl;

    /** @brief Store current control mode
     * controlMode = 10  : Joint position
     * controlMode = 20  : Cartesian stiffness
     * controlMode = 30  : Joint stiffness
     */
    int controlMode;

    unsigned int LWRDOF;
    unsigned int fri_desired_mode;

    /**
     * @brief Attribute to send shared arrays to the KRC
     */
  	RTT::Attribute<tFriKrlData> m_toFRI;

    /**
     * @brief Attribute to read shared arrays from the KRC
     */
	RTT::Attribute<tFriKrlData> m_fromFRI;

	/**
	 * @brief Property to get and set the control_mode (1 to 7)
	 */
	RTT::Property<int> control_mode_prop;

    RTT::OutputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
    RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
    RTT::OutputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
    RTT::OutputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
    RTT::OutputPort<Eigen::VectorXd > port_JointPositionCommand;
    RTT::OutputPort<Eigen::VectorXd > port_JointTorqueCommand;
    RTT::OutputPort<std_msgs::Int32 > port_KRL_CMD;

    RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrench;
    RTT::InputPort<tFriRobotState > port_RobotState;
    RTT::InputPort<tFriIntfState > port_FRIState;
    RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
    RTT::InputPort<geometry_msgs::Twist > port_CartesianVelocity;
    RTT::InputPort<geometry_msgs::Pose > port_CartesianPosition;
    RTT::InputPort<Matrix77d > port_MassMatrix;
    RTT::InputPort<KDL::Jacobian > port_Jacobian;
    RTT::InputPort<Vector7d> port_JointTorque;
    RTT::InputPort<Vector7d> port_GravityTorque;
    RTT::InputPort<Vector7d> port_JointPosition;


    RTTLWRAbstract(std::string const& name);
    ~RTTLWRAbstract();

    geometry_msgs::Pose cart_pos, cart_pos_cmd;
    geometry_msgs::Wrench cart_wrench, cart_wrench_cmd;
    geometry_msgs::Twist cart_twist;
    Matrix77d mass;
    lwr_fri::FriJointImpedance jnt_imp_cmd;
    lwr_fri::CartesianImpedance cart_imp_cmd;
    Vector7d jnt_pos_;
    Vector7d jnt_pos_old_;
    Vector7d jnt_trq_;
    Vector7d grav_trq_;
    Vector7d jnt_vel_;
    // Backward differentiation formula buffer : http://en.wikipedia.org/wiki/Backward_differentiation_formula
    //std::vector<Vector7d> jnt_pos_bdf_;
    int BDFi_;
    boost::circular_buffer<Vector7d> jnt_pos_bdf_;
    Vector7d jnt_vel_bdf_;
    static const double BDFcoeff[];//{60/147,-360/147,450/147,-400/147,225/147,-72/147,10/147};

    Vector7d jnt_pos_cmd_;
    Vector7d jnt_trq_cmd_;

    KDL::Jacobian jac_;
    KDL::Frame T_old;

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
    void setTool(int toolNumber);

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
    void estimateVelocityBDF(const double dt,const std::vector<double>& coeffs,const boost::circular_buffer<Vector7d>& x,Vector7d& xd)
    {
            for(size_t j=0;j<xd.size();++j)
            {
                double sum = 0.0 ;
                for(size_t i=0; i < coeffs.size(); ++i)
                {
                    size_t idx = (this->BDFi_ - i) % coeffs.size();
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
    Matrix77d getMassMatrix();

    /** @brief Return the gravity torque
     */
    Vector7d getGravityTorque();

    /** @brief Return the current configuration of the robot
     */
    Vector7d getJointPosition();

    /** @brief Return the estimated external joint torque
     */
    Vector7d getJointTorque();

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
    void sendJointPosition(Vector7d &qdes);

    /** @brief Send Joint velocities in radians/s
     */
    void sendJointVelocity(Vector7d &qdotdes);

    /** @brief Send additionnal joint torque in Nm
     */
    void sendAddJointTorque(Vector7d &tau);

    /** @brief Send desired tool center point cartesian pose: x, y, z(m), qw, qx, qy, qz
     */
    void sendCartesianPose(Vector7d &pose);

    /** @brief Send desired tool center point cartesian twist: vx, vy, vz, wx, wy, wz
     */
    void sendCartesianTwist(Vector7d &twist);

};
}
#endif
