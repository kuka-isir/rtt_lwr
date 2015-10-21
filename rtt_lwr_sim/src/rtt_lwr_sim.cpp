// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include<Eigen/Core>
#include<Eigen/SVD>
#include <Eigen/Dense>
#include <boost/assign.hpp>
#include <ros/service.h>
namespace Eigen{
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
} 
}
// Wrench ~ inv(J^t)*T (pseudo inverse of Jacobian transpose dot torques)
namespace KDL{
    void MultiplyJacobian(const KDL::Jacobian& jac, const KDL::JntArray& torques, KDL::Wrench& est_wrench)
    {
        Eigen::VectorXd trq = torques.data;
        
        Eigen::MatrixXd Jt = jac.data;
        //Jt.transposeInPlace();
        
        Eigen::VectorXd w(6);
        Eigen::MatrixXd pinvJ = Eigen::pseudoInverse(Jt);
        w = Jt*trq;
        est_wrench.force.x(w(0));
        est_wrench.force.y(w(1));
        est_wrench.force.z(w(2));
        
        est_wrench.torque.x(w(3));
        est_wrench.torque.y(w(4));
        est_wrench.torque.z(w(5));
    }
}

namespace lwr{
using namespace boost::assign;
    
KDL::Chain KukaLWR_DHnew(){
    using namespace KDL;
    Chain kukaLWR_DHnew;

    //joint 0
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::None),
                                     Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)
                                     ));
    //joint 1
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                              Vector::Zero(),
                                                                                                              RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));

    //joint 2
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,-0.3120511,-0.0038871),
                                                                                                               RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));

    //joint 3
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,-0.0015515,0.0),
                                                                                                               RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));

    //joint 4
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,0.5216809,0.0),
                                                                                                               RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));

    //joint 5
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                              Vector(0.0,0.0119891,0.0),
                                                                                                              RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));

    //joint 6
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,0.0080787,0.0),
                                                                                                               RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
    //joint 7
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::Identity(),
                                     RigidBodyInertia(2,
                                                      Vector::Zero(),
                                                      RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));
    return kukaLWR_DHnew;
}
bool LWRSim::waitForROSService(std::string service_name)
{
    std::cout << "*********************** "<<getName()<<" waiting for "<<service_name <<"**********************" << std::endl;
    return ros::service::waitForService(service_name, service_timeout_s*1E3);
}

void LWRSim::resetJointImpedanceGains()
{
    kp_default_[0] = 450.0;   kd_default_[0] = 1.0;
    kp_default_[1] = 450.0;   kd_default_[1] = 1.0;
    kp_default_[2] = 200.0;   kd_default_[2] = 0.7;
    kp_default_[3] = 200.0;   kd_default_[3] = 0.7;
    kp_default_[4] = 200.0;   kd_default_[4] = 0.7;
    kp_default_[5] = 20.0;    kd_default_[5] = 0.1;
    kp_default_[6] = 10.0;    kd_default_[6] = 0.0;
    this->setJointImpedance(kp_default_,kd_default_);
}
void LWRSim::resetCartesianImpedanceGains()
{
    kc_default_[0] = 1000.0;   kcd_default_[0] = 0.7;
    kc_default_[1] = 1000.0;   kcd_default_[1] = 0.7;
    kc_default_[2] = 1000.0;   kcd_default_[2] = 0.7;
    kc_default_[3] = 200.0;   kcd_default_[3] = 0.7;
    kc_default_[4] = 200.0;   kcd_default_[4] = 0.7;
    kc_default_[5] = 200.0;    kcd_default_[5] = 0.7;
    this->setCartesianImpedance(kc_default_,kcd_default_);
}
void LWRSim::setInitialJointPosition(const std::vector< double > j_init)
{
    if(!j_init.size() == LBR_MNJ){
        RTT::log(RTT::Error) << "Invalid size (should be " << LBR_MNJ << ")" << RTT::endlog();
        return;
    }
    joint_state_cmd_.position = j_init;
    
    if(using_corba)
    {
        RTT::log(RTT::Warning) << "Writing to CORBA " << joint_state_cmd_ << RTT::endlog();
        port_JointPositionGazeboCommand.write(j_init);
    }
    RTT::log(RTT::Warning) << getName() + " asking Joint Position to " << joint_state_cmd_ << RTT::endlog();
    init_pos_requested = true;
}

bool LWRSim::configureHook(){
    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
        return false;
    }

    // Set the Robot name (lwr or lwr_sim) For other components to know it
    rosparam->getRelative("robot_name");
    rosparam->getRelative("root_link");
    rosparam->getRelative("tip_link");

    RTT::log(RTT::Info)<<"root_link : "<<root_link<<RTT::endlog();
    RTT::log(RTT::Info)<<"tip_link : "<<tip_link<<RTT::endlog();
        
    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,kdl_tree_,kdl_chain_))
    {
        RTT::log(RTT::Error) << "Error while loading the URDF with params : "<<robot_name_<<" "<<root_link<<" "<<tip_link <<RTT::endlog();
        return false;
    }
    
    id_dyn_solver.reset(new KDL::ChainDynParam(kdl_chain_,gravity_vector));
    id_rne_solver.reset(new KDL::ChainIdSolver_RNE(kdl_chain_,gravity_vector));
    fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
            
      // Pointeur is not null
    kukaLWR_DHnew = KukaLWR_DHnew();
    f_ext_add.resize(kukaLWR_DHnew.getNrOfSegments());
    id_rne_solver_add_.reset(new KDL::ChainIdSolver_RNE(kukaLWR_DHnew,gravity_vector));
    // Overwrite kdl_chain_
    //kdl_chain_ = kukaLWR_DHnew;  
        
    // Store the number of degrees of freedom of the chain
    n_joints_ = kdl_chain_.getNrOfJoints();

    RTT::log(RTT::Debug)<<"LWR njoints : "<<LBR_MNJ<<" Segments : "<<kdl_chain_.getNrOfSegments()<<RTT::endlog();
    
    if(n_joints_ == 0)
    {
        RTT::log(RTT::Error)<<"Number of joints error : found "<<LBR_MNJ<<RTT::endlog();
        return false;
    }
    
    // In from gazebo
    joint_position_gazebo.resize(LBR_MNJ,0.0);
    joint_velocity_gazebo.resize(LBR_MNJ,0.0);
    joint_torque_gazebo.resize(LBR_MNJ,0.0);
    // Out to gazebo
    joint_position_gazebo_cmd.resize(LBR_MNJ,0.0);
    joint_velocity_gazebo_cmd.resize(LBR_MNJ,0.0);
    joint_torque_gazebo_cmd.resize(LBR_MNJ,0.0);

    port_JointPositionGazeboCommand.setDataSample(joint_position_gazebo_cmd);
    port_JointVelocityGazeboCommand.setDataSample(joint_velocity_gazebo_cmd);
    port_JointTorqueGazeboCommand.setDataSample(joint_torque_gazebo_cmd);
    // Fake LWR
    jnt_pos_.resize(LBR_MNJ);
    jnt_pos_fri_offset.resize(LBR_MNJ);
    jnt_pos_old_.resize(LBR_MNJ);
    jnt_vel_.resize(LBR_MNJ);
    jnt_trq_.resize(LBR_MNJ);
    jnt_trq_raw_.resize(LBR_MNJ);
    grav_trq_.resize(LBR_MNJ);
    jnt_pos_cmd_.resize(LBR_MNJ);
    jnt_trq_cmd_.resize(LBR_MNJ);
    jac_.resize(LBR_MNJ);
    jac_.data.setZero();
    mass_.resize(LBR_MNJ,LBR_MNJ);
    H.resize(LBR_MNJ);
    jnt_trq_gazebo_cmd_.resize(LBR_MNJ);
    prop_joint_offset.resize(LBR_MNJ);
    std::fill(prop_joint_offset.begin(),prop_joint_offset.end(),0.0);
    
    jnt_trq_gazebo_cmd_.setZero();
    jnt_pos_.setZero();
    jnt_pos_fri_offset.setZero();
    jnt_pos_old_.setZero();
    jnt_vel_.setZero();
    jnt_trq_.setZero();
    jnt_trq_raw_.setZero();
    grav_trq_.setZero();
    jnt_pos_cmd_.setZero();
    jnt_trq_cmd_.setZero();
    mass_.setZero();// = Eigen::MatrixXd::Zero(LBR_MNJ,LBR_MNJ);
    
    kp_.resize(LBR_MNJ);
    kd_.resize(LBR_MNJ);
    kg_.resize(LBR_MNJ);
    kp_default_.resize(LBR_MNJ);
    kd_default_.resize(LBR_MNJ);
    kc_.resize(6);
    kcd_.resize(6);
    kc_default_.resize(6);
    kcd_default_.resize(6);
     
    for(int i=0;i<FRI_USER_SIZE;i++)
    {
        fri_from_krl.realData[i]=0;
        fri_from_krl.intData[i]=0;
    }
    fri_from_krl.boolData=0;
    fri_from_krl.fill=0;

    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_cmd_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_filtered_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_gravity_);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(kdl_chain_,joint_state_dyn_);

    port_JointState.setDataSample(joint_state_);
    port_JointStateFiltered.setDataSample(joint_state_filtered_);
    port_JointStateCommand.setDataSample(joint_state_cmd_);
    port_JointStateGravity.setDataSample(joint_state_gravity_);
    port_JointStateDynamics.setDataSample(joint_state_dyn_);
    
    port_JointPosition.setDataSample(jnt_pos_);
    //port_JointPositionFRIOffset.setDataSample(jnt_pos_fri_offset);
    port_JointVelocity.setDataSample(jnt_vel_);
    port_JointTorque.setDataSample(jnt_trq_);
    //port_JointTorqueRaw.setDataSample(jnt_trq_raw_);
    port_GravityTorque.setDataSample(grav_trq_);
    port_Jacobian.setDataSample(jac_);
    port_MassMatrix.setDataSample(mass_);
    port_FromKRL.setDataSample(fri_from_krl);


    port_JointState.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/joint_states"));
    port_JointStateFiltered.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/joint_states_filtered"));
    
    port_JointStateGravity.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/joint_states_gravity"));
    port_JointStateDynamics.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/joint_states_dynamics"));
        
    port_CartesianPositionStamped.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/cartesian_pose"));
    //port_CartesianVelocity.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_twist"));
    //port_CartesianWrench.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_wrench"));
    
    port_CartesianWrenchStamped.createStream(rtt_roscomm::topic(/*"~"+*/this->getName()+"/cartesian_wrench"));
    
    q.resize(LBR_MNJ);
    f_ext.resize(kdl_chain_.getNrOfSegments());
    G.resize(LBR_MNJ);
    qdot.resize(LBR_MNJ);
    qddot.resize(LBR_MNJ);
    jnt_trq_kdl_.resize(LBR_MNJ);
    jnt_trq_kdl_add_.resize(LBR_MNJ);
    
    robot_state.control = static_cast<fri_uint16_t>(FRI_CTRL_POSITION);
    fri_state.quality = static_cast<fri_uint16_t>(FRI_QUALITY_PERFECT);
    fri_to_krl.intData[0] = FRI_STATE_MON;
    fri_from_krl.intData[0] = FRI_STATE_MON;
    
    pos_limits_ += 170,120,170,120,170,120,170;
    vel_limits_ += 112.5,112.5,112.5,112.5,180,112.5,112.5;
    trq_limits_ += 200,200,100,100,100,30,30;
    
       
    resetJointImpedanceGains();
    resetCartesianImpedanceGains();

    // This is necessary to plot the data as gazebo sets the clock
    // Note: Connections are still made and working, it's again just for plotting/recording
    if(use_sim_clock){
        RTT::Logger::Instance()->in(getName());
        RTT::log(RTT::Warning) << "Using ROS Sim Clock" << RTT::endlog();
        rtt_rosclock::use_ros_clock_topic();
        rtt_rosclock::enable_sim();
        rtt_rosclock::set_sim_clock_activity(this);
    }
}

bool LWRSim::connectToGazeboCORBA(const std::string& gazebo_deployer_name, const std::string& gazebo_robot_comp_name)
{
    if(this->hasPeer(gazebo_deployer_name)){
            RTT::TaskContext* peer = NULL;
            peer = this->getPeer(gazebo_deployer_name);
            if(peer == NULL) return false;
            // Getting information from gazebo
            RTT::ConnPolicy policy = RTT::ConnPolicy::data();
            port_JointPositionGazeboCommand.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointPositionCommand"),policy);
            port_JointVelocityGazeboCommand.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointVelocityCommand"),policy);
            port_JointTorqueGazeboCommand.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointTorqueCommand"),policy);

            port_JointPositionGazebo.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointPosition"),policy);
            port_JointVelocityGazebo.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointVelocity"),policy);
            port_JointTorqueGazebo.connectTo(peer->getPeer(gazebo_robot_comp_name)->getPort("JointTorque"),policy);
            return true;
        }else{
            RTT::log(RTT::Error)<<"Couldn't find "<<gazebo_deployer_name<<" deployer name"<<RTT::endlog();
            return false;
        }
}

bool LWRSim::safetyChecks(const std::vector<double>& position,const std::vector<double>& velocity,const std::vector<double>& torque)
{
    return safetyCheck(position,pos_limits_,"Position") &&
    safetyCheck(velocity,vel_limits_,"Velocity") &&
    safetyCheck(torque,trq_limits_,"Torque");
}

bool LWRSim::safetyCheck(const std::vector<double>& v, const std::vector<double>& limits,const std::string& name)
{
    if(v.size() != LBR_MNJ)
    {
        RTT::log(RTT::Error) << name<<" vector size error "<<v.size()<<"!="<<LBR_MNJ<<RTT::endlog();
        return false;
    }
    
    bool ret=true;
    for(unsigned i=0;i<v.size();i++)
    {
        if(std::abs(v[i]) > limits[i])
        {
           RTT::log(RTT::Error) << name<<" limit exceded at J"<<i<<" : "<<v[i]<<" / limit "<<limits[i]<<RTT::endlog();
           ret = false;
        }
    }
    return ret;
}

bool LWRSim::setJointImpedance(const Eigen::VectorXd& stiffness, const Eigen::VectorXd& damping)
{
    if(! (stiffness.size() == LBR_MNJ && damping.size() == LBR_MNJ))
    {
        RTT::log(RTT::Error) << "Size error, not setting impedance (stiff size "
                            <<stiffness.size()<<" damp size "<<damping.size()
                            <<", should be "<<LBR_MNJ<<")"<<RTT::endlog();
        return false;
    }
    
    for(unsigned j=0;j<LBR_MNJ;++j)
    {
        jnt_imp_.stiffness[j] = stiffness[j];
        jnt_imp_.damping[j] = damping[j];
    }
    
    kp_ = stiffness;
    kd_ = damping;
    
    return true;
}
bool LWRSim::setGravityMode()
{
    Eigen::VectorXd s(LBR_MNJ);
    s.setZero();
    Eigen::VectorXd d(LBR_MNJ);
    d.setZero();
    return this->setJointImpedance(s,d);
}

bool LWRSim::setCartesianImpedance(const Eigen::Matrix< double, 6, 1 >& cart_stiffness, const Eigen::Matrix< double, 6, 1 >& cart_damping)
{
    if(cart_damping.size() != 6 || cart_damping.size() != 6)
    {
        RTT::log(RTT::Error) << "Size error, not setting cartesian impedance (stiff size "
                            <<cart_stiffness.size()<<" damp size "<<cart_damping.size()
                            <<", should be 6 and 6)"<<RTT::endlog();
        return false;
    }

    cart_imp_.stiffness.linear.x = cart_stiffness[0];
    cart_imp_.stiffness.linear.y = cart_stiffness[1];
    cart_imp_.stiffness.linear.z = cart_stiffness[2];
    
    cart_imp_.stiffness.angular.x = cart_stiffness[3];
    cart_imp_.stiffness.angular.y = cart_stiffness[4];
    cart_imp_.stiffness.angular.z = cart_stiffness[5];
    
    cart_imp_.damping.linear.x = cart_damping[0];
    cart_imp_.damping.linear.y = cart_damping[1];
    cart_imp_.damping.linear.z = cart_damping[2];
    
    cart_imp_.damping.angular.x = cart_damping[3];
    cart_imp_.damping.angular.y = cart_damping[4];
    cart_imp_.damping.angular.z = cart_damping[5];
    
    kc_ = cart_stiffness;
    kcd_ = cart_damping;
    
    return true;
}

void LWRSim::updateJointImpedance(const lwr_fri::FriJointImpedance& impedance)
{
    kp_[0] = clamp(impedance.stiffness[0],0.0,450.0);   kd_[0] = clamp(impedance.damping[0],0.0,1.0);
    kp_[1] = clamp(impedance.stiffness[1],0.0,450.0);   kd_[1] = clamp(impedance.damping[1],0.0,1.0);
    kp_[2] = clamp(impedance.stiffness[2],0.0,200.0);   kd_[2] = clamp(impedance.damping[2],0.0,0.7);
    kp_[3] = clamp(impedance.stiffness[3],0.0,200.0);   kd_[3] = clamp(impedance.damping[3],0.0,0.7);
    kp_[4] = clamp(impedance.stiffness[4],0.0,200.0);   kd_[4] = clamp(impedance.damping[4],0.0,0.7);
    kp_[5] = clamp(impedance.stiffness[5],0.0,10.0);   kd_[5] = clamp(impedance.damping[5],0.0,0.1);
    kp_[6] = clamp(impedance.stiffness[6],0.0,10.0);   kd_[6] = clamp(impedance.damping[6],0.0,0.0);
    
}

void LWRSim::updateCartesianImpedance(const lwr_fri::CartesianImpedance& cart_impedance)
{
    kc_[0] = cart_impedance.stiffness.linear.x;
    kc_[1] = cart_impedance.stiffness.linear.y;
    kc_[2] = cart_impedance.stiffness.linear.z;
    
    kc_[3] = cart_impedance.stiffness.angular.x;
    kc_[4] = cart_impedance.stiffness.angular.y;
    kc_[5] = cart_impedance.stiffness.angular.z;
    
    kcd_[0] = cart_impedance.damping.linear.x;
    kcd_[1] = cart_impedance.damping.linear.y;
    kcd_[2] = cart_impedance.damping.linear.z;
    
    kcd_[3] = cart_impedance.damping.angular.x;
    kcd_[4] = cart_impedance.damping.angular.y;
    kcd_[5] = cart_impedance.damping.angular.z;
    
    cart_imp_ = cart_impedance;
}


void LWRSim::setJointImpedanceMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_JNT_IMP;
}
void LWRSim::setCartesianImpedanceMode()
{
    fri_to_krl.intData[1] = 10*FRI_CTRL_CART_IMP;
}

void LWRSim::updateHook() {
    RTT::log(RTT::Debug) << "LWR SIM Update at "<<rtt_rosclock::host_now()<< RTT::endlog();
    static double last_update_time_sim;
    double rtt_time_ = 1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
    period_sim_ = rtt_time_ - last_update_time_sim;
    last_update_time_sim = rtt_time_;
        
    read_start = rtt_rosclock::host_now().toSec();
    // Read From gazebo simulation
    if(using_corba && (port_JointPositionGazebo.connected() 
        || port_JointVelocityGazebo.connected() 
        || port_JointTorqueGazebo.connected()))
    {
        RTT::FlowStatus fs_p = port_JointPositionGazebo.read(joint_position_gazebo);
        RTT::FlowStatus fs_v = port_JointVelocityGazebo.read(joint_velocity_gazebo);
        RTT::FlowStatus fs_g = port_JointTorqueGazebo.read(joint_torque_gazebo);
        
        if(fs_g == RTT::NoData || fs_p == RTT::NoData || fs_v == RTT::NoData){
            RTT::log(RTT::Warning)  << getName() << ": NoData" << RTT::endlog();
            return;
        }
    }else{// NOTE: Using ROS instead
        if(RTT::NoData != port_JointStateGazebo.read(joint_state_))
        {
            joint_position_gazebo = joint_state_.position;
            joint_velocity_gazebo = joint_state_.velocity;
            joint_torque_gazebo = joint_state_.effort;
        }
        
    }
    
    joint_state_cmd_.header.frame_id = "";
    
    jnt_trq_cmd_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
    jnt_pos_cmd_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
    cart_pos_cmd_fs = port_CartesianPositionCommand.read(cart_pos_cmd_);
    cart_wrench_cmd_fs = port_CartesianWrenchCommand.read(cart_wrench_cmd_);
    RTT::FlowStatus fs_fri_to_krl = port_ToKRL.read(fri_to_krl);
    RTT::FlowStatus fs_jnt_imp_cmd = port_JointImpedanceCommand.read(jnt_imp_cmd_);
    RTT::FlowStatus fs_cart_imp_cmd = port_CartesianImpedanceCommand.read(cart_imp_cmd_);
    
    fri_state.timestamp = read_start;

    if(fs_fri_to_krl == RTT::NewData)
    {
        for(int i=0;i<FRI_USER_SIZE;i++)
        {
            fri_from_krl.realData[i]=fri_to_krl.realData[i];
            fri_from_krl.intData[i]=fri_to_krl.intData[i];
        }
        fri_from_krl.boolData=fri_to_krl.boolData;
    }
    switch(fri_to_krl.intData[1]){
        case 10:
            robot_state.control = FRI_CTRL_POSITION;
            break;
        case 30:
            robot_state.control = FRI_CTRL_JNT_IMP;
            break;
        case 20:
            robot_state.control = FRI_CTRL_CART_IMP;
            break;
        default:
            robot_state.control = FRI_CTRL_OTHER;
            break;
        }
    switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_POSITION:
            case FRI_CTRL_JNT_IMP:
            case FRI_CTRL_CART_IMP:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_CMD;
                robot_state.power = 1;
                break;
            case FRI_CTRL_OTHER:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_MON;
                robot_state.power = 0;
                break;
            default:
                fri_state.state = fri_from_krl.intData[0] = FRI_STATE_OFF;
                robot_state.power = 0;
                break;
        }
    
    if(safety_checks_)
        safetyChecks(joint_position_gazebo,joint_velocity_gazebo,joint_torque_gazebo);
    
    jnt_pos_ = Eigen::VectorXd::Map(joint_position_gazebo.data(),LBR_MNJ);
    jnt_vel_ = Eigen::VectorXd::Map(joint_velocity_gazebo.data(),LBR_MNJ);
    jnt_trq_ = Eigen::VectorXd::Map(joint_torque_gazebo.data(),LBR_MNJ);

    // Waiting for CartesianVelocityCommand
    Xd_cmd_.setZero();
    
    if(fs_jnt_imp_cmd == RTT::NewData)
        updateJointImpedance(jnt_imp_cmd_);
    
    if(fs_cart_imp_cmd == RTT::NewData)
        updateCartesianImpedance(cart_imp_cmd_);

    read_duration = (rtt_rosclock::host_now().toSec() - read_start);

    q.q.data = jnt_pos_;
    q.qdot.data = jnt_vel_;
    qdot.data = jnt_vel_;
    qddot.data.setConstant(0.0);
    
    id_dyn_solver->JntToMass(q.q,H);
    mass_ = H.data;
    
    std::fill(f_ext.begin(),f_ext.end(),KDL::Wrench::Zero());
    
    id_rne_solver->CartToJnt(q.q,qdot,qddot,f_ext,jnt_trq_kdl_);
    
    std::fill(f_ext_add.begin(),f_ext_add.end(),KDL::Wrench::Zero());
    
    id_rne_solver_add_->CartToJnt(q.q,qdot,qddot,f_ext_add,jnt_trq_kdl_add_);
    
    id_dyn_solver->JntToGravity(q.q,G);
    
    Eigen::Map<Eigen::VectorXd>(joint_state_dyn_.position.data(),LBR_MNJ) = jnt_trq_kdl_add_.data;
    Eigen::Map<Eigen::VectorXd>(joint_state_dyn_.velocity.data(),LBR_MNJ) = G.data;
    Eigen::Map<Eigen::VectorXd>(joint_state_dyn_.effort.data(),LBR_MNJ) = jnt_trq_kdl_.data;

    grav_trq_ = G.data;

    jnt_to_jac_solver->JntToJac(this->q.q,jac_,kdl_chain_.getNrOfSegments());
    fk_vel_solver->JntToCart(q,ee_framevel_kdl_,kdl_chain_.getNrOfSegments());
    ee_twist_kdl_ = ee_framevel_kdl_.GetTwist();
    ee_frame_kdl_ = ee_framevel_kdl_.GetFrame();

    tf::poseKDLToMsg(ee_frame_kdl_,cart_pos_);
    tf::twistKDLToMsg(ee_twist_kdl_,cart_twist_); 
    
    cart_pos_stamped_.header.frame_id = root_link;
    cart_pos_stamped_.pose = cart_pos_;

    
    //ee_twist_des_kdl_.SetToZero();
    KDL::SetToZero(ee_twist_des_kdl_);
    ee_twist_diff_kdl_ = KDL::diff(ee_twist_kdl_,ee_twist_des_kdl_);
    tf::twistKDLToEigen(ee_twist_diff_kdl_,Xd_err_);
    
    if(cart_pos_cmd_fs == RTT::NewData){
        tf::poseMsgToKDL(cart_pos_cmd_,ee_frame_des_kdl_);
        ee_frame_diff_kdl_ = KDL::diff(ee_frame_kdl_,ee_frame_des_kdl_);
        tf::twistKDLToEigen(ee_frame_diff_kdl_,X_err_);
    }
    
    
    F_cmd_.setZero();
    if(cart_wrench_cmd_fs == RTT::NewData)
        tf::wrenchMsgToEigen(cart_wrench_cmd_,F_cmd_);
    
    if(jnt_trq_cmd_fs != RTT::NewData)
        jnt_trq_cmd_.setZero();    
    
    cart_wrench_stamped_.header.frame_id = tip_link;
    cart_wrench_stamped_.wrench = cart_wrench_;
    
    // Send at least Gravity

      switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_JNT_IMP:
                // Joint Impedance Control
                jnt_trq_gazebo_cmd_ = G.data;
                
                // Joint Impedance part
                if(jnt_pos_cmd_fs == RTT::NewData){
                    jnt_trq_gazebo_cmd_ += kp_.asDiagonal()*(jnt_pos_cmd_ - jnt_pos_) - kd_.asDiagonal()*jnt_vel_ ;
                }
                // Additional torque
                if(jnt_trq_cmd_fs != RTT::NoData)
                    jnt_trq_gazebo_cmd_ += jnt_trq_cmd_;
                    
                Eigen::Map<Eigen::VectorXd>(joint_torque_gazebo_cmd.data(),LBR_MNJ) = jnt_trq_gazebo_cmd_;
                port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                joint_state_cmd_.header.frame_id = "EFFORT_CMD";
                
                break;
            case FRI_CTRL_POSITION:
                //Position Control
                jnt_trq_gazebo_cmd_ = G.data;
                
                if(jnt_pos_cmd_fs == RTT::NewData){
                    jnt_trq_gazebo_cmd_ += kp_default_.asDiagonal()*(jnt_pos_cmd_-jnt_pos_) - kd_default_.asDiagonal()*jnt_vel_ ;
                }
                
                Eigen::Map<Eigen::VectorXd>(joint_torque_gazebo_cmd.data(),LBR_MNJ) = jnt_trq_gazebo_cmd_;
                port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                joint_state_cmd_.header.frame_id = "EFFORT_CMD";
                
                break;
            case FRI_CTRL_CART_IMP:
                //Cartesian Impedance Control
                jnt_trq_gazebo_cmd_ = G.data;
                
                if(cart_pos_cmd_fs == RTT::NewData){
                    jnt_trq_gazebo_cmd_ += jac_.data.transpose()*(kc_.asDiagonal()*(X_err_) + F_cmd_ + kcd_.asDiagonal()*(Xd_err_));
                }
                
                Eigen::Map<Eigen::VectorXd>(joint_torque_gazebo_cmd.data(),LBR_MNJ) = jnt_trq_gazebo_cmd_;
                port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                joint_state_cmd_.header.frame_id = "EFFORT_CMD";
                
                break;
            default:
                break;
      }
    
    now = rtt_rosclock::host_now();
    write_start = now.toSec();
    //Update status
    cart_pos_stamped_.header.stamp = now;
    cart_wrench_stamped_.header.stamp = now;
    joint_state_.header.stamp = now;
    joint_state_filtered_.header.stamp = now;
    
    joint_state_cmd_.header.stamp = now;
    joint_state_gravity_.header.stamp = now;
    joint_state_dyn_.header.stamp = now;

    Eigen::Map<Eigen::VectorXd>(joint_state_gravity_.effort.data(),LBR_MNJ) = grav_trq_;
    
    Eigen::Map<Eigen::VectorXd>(joint_state_.position.data(),LBR_MNJ) = jnt_pos_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.velocity.data(),LBR_MNJ) = jnt_vel_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.effort.data(),LBR_MNJ) = jnt_trq_;

    if(jnt_pos_cmd_fs == RTT::NewData /*&& init_pos_requested==false*/)
        Eigen::Map<Eigen::VectorXd>(joint_state_cmd_.position.data(),LBR_MNJ) = jnt_pos_cmd_;
    
    joint_state_cmd_.effort = joint_torque_gazebo_cmd;

    for(unsigned int j=0;j<joint_position_gazebo.size();++j)
    {
        joint_state_filtered_.velocity[j] = velocity_smoothing_factor_*(jnt_pos_[j]-joint_state_filtered_.position[j])/this->getPeriod() + (1.0-velocity_smoothing_factor_)*joint_state_filtered_.velocity[j];
    }
    Eigen::Map<Eigen::VectorXd>(joint_state_filtered_.position.data(),LBR_MNJ) = jnt_pos_;
    //Eigen::Map<Eigen::VectorXd>(joint_state_filtered_.effort.data(),LBR_MNJ) = jnt_trq_;
    
    port_JointState.write(joint_state_);
    port_JointStateFiltered.write(joint_state_filtered_);
    port_JointStateGravity.write(joint_state_gravity_);
    port_JointStateDynamics.write(joint_state_dyn_);

    port_JointPosition.write(jnt_pos_);
    port_JointVelocity.write(jnt_vel_);
    port_JointTorque.write(jnt_trq_);
    port_GravityTorque.write(grav_trq_);

    port_CartesianPosition.write(cart_pos_);
    port_CartesianPositionStamped.write(cart_pos_stamped_);
    port_CartesianVelocity.write(cart_twist_);
    port_CartesianWrench.write(cart_wrench_);
    port_CartesianWrenchStamped.write(cart_wrench_stamped_);
    
    port_Jacobian.write(jac_);
    port_MassMatrix.write(mass_);
    
    if(init_pos_requested){
        joint_state_cmd_.header.frame_id="POSITION_CMD";
        init_pos_requested=false;
    }
    port_JointStateCommand.write(joint_state_cmd_);
    port_FromKRL.write(fri_from_krl);
    
    port_RobotState.write(robot_state);
    port_FRIState.write(fri_state);
    
    write_duration = (rtt_rosclock::host_now().toSec() - write_start);
    updatehook_duration = (rtt_rosclock::host_now().toSec() - read_start);

}
}
