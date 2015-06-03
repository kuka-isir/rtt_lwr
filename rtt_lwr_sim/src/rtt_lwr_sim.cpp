// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include<Eigen/Core>
#include<Eigen/SVD>
#include <Eigen/Dense>
#include <boost/assign.hpp>

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
bool LWRSim::configureHook(){
    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
        return false;
    }

    // Set the Robot name (lwr or lwr_sim) For other components to know it
    rosparam->setComponentPrivate("robot_name");
    rosparam->getComponentPrivate("root_link");
    rosparam->getComponentPrivate("tip_link");
    //rosparam->getParam(getName()+"/"+"use_sim_clock",use_sim_clock);
    
    rosparam->setComponentPrivate("root_link");
    rosparam->setComponentPrivate("tip_link");
    
    RTT::log(RTT::Info)<<"root_link : "<<root_link<<RTT::endlog();
    RTT::log(RTT::Info)<<"tip_link : "<<tip_link<<RTT::endlog();
    
    KDL::Vector gravity_vector(0.,0.,-9.81289);
    
    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,
                                                     root_link,
                                                     tip_link,
                                                     kdl_tree_,
                                                     kdl_chain_,
                                                     getName()+"/"+"robot_description"
                                                    ))
    {
        RTT::log(RTT::Error) << "Error while loading the URDF with params : "<<robot_name_<<" "<<root_link<<" "<<tip_link <<RTT::endlog();
        return false;
    }
    
    ik_solver_vel.reset(new KDL::ChainIkSolverVel_pinv_nso(kdl_chain_));
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

    PeerList lpeers = this->getPeerList();
    for(PeerList::iterator p=lpeers.begin();p!=lpeers.end();++p)
        RTT::log(RTT::Debug)<<" Peer : "<<(*p)<<RTT::endlog();

    if(this->hasPeer("gazebo")){
        this->peer = this->getPeer("gazebo");
        // Getting information from gazebo
        std::string lwr_gazebo("lwr_gazebo");
        RTT::ConnPolicy policy = RTT::ConnPolicy::data();
        port_JointPositionGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointPositionCommand"),policy);
        port_JointVelocityGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointVelocityCommand"),policy);
        port_JointTorqueGazeboCommand.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointTorqueCommand"),policy);

        port_JointPositionGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointPosition"),policy);
        port_JointVelocityGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointVelocity"),policy);
        port_JointTorqueGazebo.connectTo(this->peer->getPeer(lwr_gazebo)->getPort("JointTorque"),policy);

        // The number of joints is 8 because it counts base_link, even though it's a fixed joint
        RTT::Property<unsigned int> njoints = this->peer->getPeer(lwr_gazebo)->getProperty("n_joints");
        RTT::log(RTT::Debug)<<"LWR njoints GAZEBO : "<<njoints.get()<<RTT::endlog();

    }else{
        RTT::log(RTT::Error)<<"Couldn't find gazebo and lwr_gazebo peer"<<RTT::endlog();
        return false;
    }

    RTT::log(RTT::Debug)<<"LWR njoints : "<<n_joints_<<" Segments : "<<kdl_chain_.getNrOfSegments()<<RTT::endlog();
    
    if(n_joints_ != LBR_MNJ)
    {
        RTT::log(RTT::Error)<<"Number of joints error : found "<<n_joints_<<" but should be "<<LBR_MNJ<<RTT::endlog();
        return false;
    }
    
    // In from gazebo
    joint_position_gazebo.resize(n_joints_,0.0);
    joint_velocity_gazebo.resize(n_joints_,0.0);
    joint_torque_gazebo.resize(n_joints_,0.0);
    // Out to gazebo
    joint_position_gazebo_cmd.resize(n_joints_,0.0);
    joint_velocity_gazebo_cmd.resize(n_joints_,0.0);
    joint_torque_gazebo_cmd.resize(n_joints_,0.0);
    // Fake LWR
    jnt_pos_.resize(n_joints_);
    jnt_pos_fri_offset.resize(n_joints_);
    jnt_pos_old_.resize(n_joints_);
    jnt_vel_.resize(n_joints_);
    jnt_trq_.resize(n_joints_);
    jnt_trq_raw_.resize(n_joints_);
    grav_trq_.resize(n_joints_);
    jnt_pos_cmd_.resize(n_joints_);
    jnt_trq_cmd_.resize(n_joints_);
    jac_.resize(n_joints_);
    jac_.data.setZero();
    mass_.resize(n_joints_,n_joints_);
    jnt_trq_gazebo_cmd_.resize(n_joints_);
    
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
    mass_.setZero();// = Eigen::MatrixXd::Zero(n_joints_,n_joints_);
    
    kp_.resize(n_joints_);
    kd_.resize(n_joints_);
    kg_.resize(n_joints_);
    kp_default_.resize(n_joints_);
    kd_default_.resize(n_joints_);
    kc_.resize(6);
    kcd_.resize(6);
    kc_default_.resize(6);
    kcd_default_.resize(6);
     
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
    port_JointPositionFRIOffset.setDataSample(jnt_pos_fri_offset);
    port_JointVelocity.setDataSample(jnt_vel_);
    port_JointTorque.setDataSample(jnt_trq_);
    port_JointTorqueRaw.setDataSample(jnt_trq_raw_);
    port_GravityTorque.setDataSample(grav_trq_);
    port_Jacobian.setDataSample(jac_);
    port_MassMatrix.setDataSample(mass_);
    
    port_JointState.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states"));
    port_JointStateFiltered.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states_filtered"));
    port_JointStateCommand.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states_cmd"));
    port_JointStateGravity.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states_gravity"));
    port_JointStateDynamics.createStream(rtt_roscomm::topic("/"+this->getName()+"/joint_states_dynamics"));
        
    port_CartesianPositionStamped.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_pose"));
    //port_CartesianVelocity.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_twist"));
    //port_CartesianWrench.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_wrench"));
    
    port_CartesianWrenchStamped.createStream(rtt_roscomm::topic("/"+this->getName()+"/cartesian_wrench"));
    
    q.resize(n_joints_);
    f_ext.resize(kdl_chain_.getNrOfSegments());
    G.resize(n_joints_);
    qdot.resize(n_joints_);
    qddot.resize(n_joints_);
    jnt_trq_kdl_.resize(n_joints_);
    jnt_trq_kdl_add_.resize(n_joints_);
    
    robot_state.control = static_cast<FRI_CTRL>(FRI_CTRL_POSITION);
    fri_state.quality = static_cast<FRI_QUALITY>(FRI_QUALITY_PERFECT);
    fri_to_krl.intData[0] = FRI_STATE_MON;
    fri_from_krl.intData[0] = FRI_STATE_MON;
    
    pos_limits_ += 170,120,170,120,170,120,170;
    vel_limits_ += 112.5,112.5,112.5,112.5,180,112.5,112.5;
    trq_limits_ += 200,200,100,100,100,30,30;
    
       
    resetJointImpedanceGains();
    resetCartesianImpedanceGains();
    
    if(use_sim_clock){
        rtt_rosclock::use_ros_clock_topic();
        rtt_rosclock::enable_sim();
    }
    return true;
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
    read_start = rtt_rosclock::host_now().toSec();
    // Read From gazebo simulation
    RTT::FlowStatus fs_p = port_JointPositionGazebo.read(joint_position_gazebo);
    RTT::FlowStatus fs_v = port_JointVelocityGazebo.read(joint_velocity_gazebo);
    RTT::FlowStatus fs_g = port_JointTorqueGazebo.read(joint_torque_gazebo);

    fri_state.timestamp = read_start;

    if(port_ToKRL.read(fri_to_krl) == RTT::NewData)
    {
        fri_from_krl = fri_to_krl;
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
    port_FromKRL.write(fri_from_krl);
    
    port_RobotState.write(robot_state);
    port_FRIState.write(fri_state);
    
    if(fs_g == RTT::NoData || fs_p == RTT::NoData || fs_v == RTT::NoData)
        return;
    
    safetyChecks(joint_position_gazebo,joint_velocity_gazebo,joint_torque_gazebo);
    
    jnt_pos_ = Eigen::VectorXd::Map(joint_position_gazebo.data(),n_joints_);
    jnt_vel_ = Eigen::VectorXd::Map(joint_velocity_gazebo.data(),n_joints_);
    jnt_trq_ = Eigen::VectorXd::Map(joint_torque_gazebo.data(),n_joints_);

    // Read Commands from users
    jnt_trq_cmd_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
    jnt_pos_cmd_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
    cart_pos_cmd_fs = port_CartesianPositionCommand.read(cart_pos_cmd_);
    cart_wrench_cmd_fs = port_CartesianWrenchCommand.read(cart_wrench_cmd_);
    
    
    
    // Waiting for JointVelocityCommand
    Xd_cmd_.setZero();
    
    /*if(jnt_trq_cmd_fs != RTT::NewData)
        jnt_trq_cmd_.setZero();

    if(jnt_pos_cmd_fs != RTT::NewData)
        jnt_pos_cmd_.setZero();*/
    
    if(port_JointImpedanceCommand.read(jnt_imp_cmd_) == RTT::NewData)
        updateJointImpedance(jnt_imp_cmd_);
    
    if(port_CartesianImpedanceCommand.read(cart_imp_cmd_) == RTT::NewData)
        updateCartesianImpedance(cart_imp_cmd_);

    read_duration = (rtt_rosclock::host_now().toSec() - read_start);

    
    for(unsigned int j=0; j < n_joints_; j++){
        q.q(j) = jnt_pos_[j];
        q.qdot(j) = jnt_vel_[j];
        qdot(j) = jnt_vel_[j];
        qddot(j) = 0.0;
        //(j) = jnt_trq_[j];
    }
    
    id_dyn_solver->JntToMass(q.q,H);
    mass_ = H.data;
    
    for(unsigned int j=0;j<kdl_chain_.getNrOfSegments();++j)
        f_ext[j] = KDL::Wrench::Zero();
    
    int ret_rne = id_rne_solver->CartToJnt(q.q,qdot,qddot,f_ext,jnt_trq_kdl_);
    if(ret_rne<0)
        RTT::log(RTT::Error)<<"ERROR on id_rne_solver : "<<ret_rne<<RTT::endlog();

    
    for(unsigned int j=0;j<kukaLWR_DHnew.getNrOfSegments();++j)
        f_ext_add[j] = KDL::Wrench::Zero();
    
    int ret_rne_add = id_rne_solver_add_->CartToJnt(q.q,qdot,qddot,f_ext_add,jnt_trq_kdl_add_);
    if(ret_rne_add<0)
        RTT::log(RTT::Error)<<"ERROR on ret_rne_add : "<<ret_rne_add<<RTT::endlog();
    
    int ret_dyn = id_dyn_solver->JntToGravity(q.q,G);
    if(ret_dyn<0)
        RTT::log(RTT::Error)<<"ERROR on id_dyn_solver : "<<ret_dyn<<RTT::endlog();
    
    Eigen::Map<Eigen::VectorXd>(joint_state_dyn_.position.data(),n_joints_) = jnt_trq_kdl_add_.data;
    Eigen::Map<Eigen::VectorXd>(joint_state_dyn_.velocity.data(),n_joints_) = G.data;
    Eigen::Map<Eigen::VectorXd>(joint_state_dyn_.effort.data(),n_joints_) = jnt_trq_kdl_.data;
    


    grav_trq_ = G.data;

    jnt_to_jac_solver->JntToJac(this->q.q,jac_);
    KDL::FrameVel endToBase;
    fk_vel_solver->JntToCart(q,endToBase);
    KDL::Twist endTwist = endToBase.GetTwist();
    KDL::Frame endFrame = endToBase.GetFrame();
    KDL::Vector endPos = endFrame.p;
    KDL::Rotation endRot = endFrame.M;

    cart_pos_.position.x = endPos[0];
    cart_pos_.position.y = endPos[1];
    cart_pos_.position.z = endPos[2];
    
    cart_pos_stamped_.header.frame_id = root_link;
    cart_pos_stamped_.pose = cart_pos_;
    
    endRot.GetQuaternion(cart_pos_.orientation.x,
                         cart_pos_.orientation.y,
                         cart_pos_.orientation.z,
                         cart_pos_.orientation.w
                        );
    
    cart_twist_.linear.x = endTwist.vel[0];
    cart_twist_.linear.y = endTwist.vel[1];
    cart_twist_.linear.z = endTwist.vel[2];
    
    cart_twist_.angular.x = endTwist.rot[0];
    cart_twist_.angular.y = endTwist.rot[1];
    cart_twist_.angular.z = endTwist.rot[2];
    
    KDL::MultiplyJacobian(jac_,jnt_trq_kdl_,cart_wrench_kdl_);
    cart_wrench_.force.x = cart_wrench_kdl_.force.x();
    cart_wrench_.force.y = cart_wrench_kdl_.force.y();
    cart_wrench_.force.z = cart_wrench_kdl_.force.z();
    cart_wrench_.torque.x = cart_wrench_kdl_.torque.x();
    cart_wrench_.torque.y = cart_wrench_kdl_.torque.y();
    cart_wrench_.torque.z = cart_wrench_kdl_.torque.z();
    //cart_wrench_; // TODO: GET DATA FROM ATI SENSOR
    
    quat_tf.setX(cart_pos_.orientation.x);
    quat_tf.setY(cart_pos_.orientation.y);
    quat_tf.setZ(cart_pos_.orientation.z);
    quat_tf.setW(cart_pos_.orientation.w);
    
    quat_m.setRotation(quat_tf);
    quat_m.getRPY(roll, pitch, yaw);
    
    X_[0] = cart_pos_.position.x;
    X_[1] = cart_pos_.position.y;
    X_[2] = cart_pos_.position.z;
    X_[3] = roll;
    X_[4] = pitch;
    X_[5] = yaw;
    
    X_cmd_.setZero();
    if(cart_pos_cmd_fs == RTT::NewData){
        quat_tf.setX(cart_pos_cmd_.orientation.x);
        quat_tf.setY(cart_pos_cmd_.orientation.y);
        quat_tf.setZ(cart_pos_cmd_.orientation.z);
        quat_tf.setW(cart_pos_cmd_.orientation.w);
        
        quat_m.setRotation(quat_tf);
        quat_m.getRPY(roll, pitch, yaw);
        
        X_cmd_[0] = cart_pos_cmd_.position.x;
        X_cmd_[1] = cart_pos_cmd_.position.y;
        X_cmd_[2] = cart_pos_cmd_.position.z;
        X_cmd_[3] = roll;
        X_cmd_[4] = pitch;
        X_cmd_[5] = yaw;
    }
    
    tf::twistMsgToEigen(cart_twist_,Xd_);
    
    F_cmd_.setZero();
    if(cart_wrench_cmd_fs == RTT::NewData)
        tf::wrenchMsgToEigen(cart_wrench_cmd_,F_cmd_);
    
    
    cart_wrench_stamped_.header.frame_id = tip_link;
    cart_wrench_stamped_.wrench = cart_wrench_;
    
    
      switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_JNT_IMP:
                // Joint Impedance Control
                
                // If no new command received, then don't send additional torque
                if(jnt_trq_cmd_fs != RTT::NewData)
                    jnt_trq_cmd_.setZero();
                
                if(jnt_pos_cmd_fs == RTT::NewData){
                    //Impedance Control
                    jnt_trq_gazebo_cmd_ = kp_.asDiagonal()*(jnt_pos_cmd_ - jnt_pos_) - kd_.asDiagonal()*jnt_vel_ + jnt_trq_cmd_ + G.data;
                }else 
                    ////////////////////////////// TODO : remove that
                    if(jnt_pos_cmd_fs != RTT::NewData && jnt_trq_cmd_fs == RTT::NewData)
                            jnt_trq_gazebo_cmd_ = jnt_trq_cmd_ + G.data;
                    
                    Eigen::Map<Eigen::VectorXd>(joint_torque_gazebo_cmd.data(),n_joints_) = jnt_trq_gazebo_cmd_;
                    port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                
                break;
            case FRI_CTRL_POSITION:
                if(jnt_pos_cmd_fs == RTT::NewData){
                    //Position Control
                    jnt_trq_gazebo_cmd_ = kp_default_.asDiagonal()*(jnt_pos_cmd_-jnt_pos_) - kd_default_.asDiagonal()*jnt_vel_ + G.data;
                    
                    Eigen::Map<Eigen::VectorXd>(joint_torque_gazebo_cmd.data(),n_joints_) = jnt_trq_gazebo_cmd_;
                    port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                }
                break;
            case FRI_CTRL_CART_IMP:
                if(cart_pos_cmd_fs == RTT::NewData){
                    //Cartesian Impedance Control
                    jnt_trq_gazebo_cmd_ = jac_.data.transpose()*(kc_.asDiagonal()*(X_cmd_-X_) + F_cmd_ + kcd_.asDiagonal()*(Xd_cmd_ - Xd_)) + G.data;
                    
                    RTT::log(RTT::Debug) << kc_<<(X_cmd_-X_) << kcd_<<(Xd_cmd_ - Xd_) << F_cmd_<<RTT::endlog();
                    
                    
                    Eigen::Map<Eigen::VectorXd>(joint_torque_gazebo_cmd.data(),n_joints_) = jnt_trq_gazebo_cmd_;
                    port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);
                }
                break;
            default:
                break;
      }
    
    now = rtt_rosclock::host_now();
       
    write_start = now.toNSec();
    //Update status
    cart_pos_stamped_.header.stamp = now;
    cart_wrench_stamped_.header.stamp = now;
    joint_state_.header.stamp = now;
    joint_state_filtered_.header.stamp = now;
    joint_state_cmd_.header.stamp = now;
    joint_state_gravity_.header.stamp = now;
    joint_state_dyn_.header.stamp = now;

    Eigen::Map<Eigen::VectorXd>(joint_state_gravity_.effort.data(),n_joints_) = grav_trq_;
    
    Eigen::Map<Eigen::VectorXd>(joint_state_.position.data(),n_joints_) = jnt_pos_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.velocity.data(),n_joints_) = jnt_vel_;
    Eigen::Map<Eigen::VectorXd>(joint_state_.effort.data(),n_joints_) = jnt_trq_;

    Eigen::Map<Eigen::VectorXd>(joint_state_cmd_.position.data(),n_joints_) = jnt_pos_cmd_;
    joint_state_cmd_.effort = joint_torque_gazebo_cmd;

    for(unsigned int j=0;j<joint_position_gazebo.size();++j)
    {
        joint_state_filtered_.velocity[j] = velocity_smoothing_factor_*(jnt_pos_[j]-joint_state_filtered_.position[j])/this->getPeriod() + (1.0-velocity_smoothing_factor_)*joint_state_filtered_.velocity[j];
    }
    Eigen::Map<Eigen::VectorXd>(joint_state_filtered_.position.data(),n_joints_) = jnt_pos_;
    //Eigen::Map<Eigen::VectorXd>(joint_state_filtered_.effort.data(),n_joints_) = jnt_trq_;
    
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
    
    port_JointStateCommand.write(joint_state_cmd_);
    
    
    write_duration = (rtt_rosclock::host_now().toSec() - write_start);
    updatehook_duration = (rtt_rosclock::host_now().toSec() - read_start);

}
}
