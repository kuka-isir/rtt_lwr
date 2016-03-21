// Copyright ISIR 2015
// Antoine Hoarau <hoarau.robotics@gmail.com>
#include <rtt_lwr_sim/rtt_lwr_sim.hpp>
#include <rtt_ros_kdl_tools/mqueue_connpolicy.hpp>
#include <Eigen/Dense>
#include <ros/service.h>

using namespace lwr;
using namespace KDL;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void LWRSim::updateHook()
{
    log(RTT::Debug) << getName() << " UpdateHook() BEGIN "<< TimeService::Instance()->getNSecs() << endlog();
    RTT::os::MutexLock lock(gazebo_mutex_);
    log(RTT::Debug) << getName() << " UpdateHook() START "<< TimeService::Instance()->getNSecs() << endlog();
    static TimeService::nsecs last_tstart,tstart,tstart_wait;
    tstart = TimeService::Instance()->getNSecs();
    // Reset commands from users
    jnt_trq_cmd_.setZero();
    if(set_brakes_ == false) jnt_pos_cmd_ = jnt_pos_;
    Xd_cmd_.setZero();
    F_cmd_.setZero();

    // Do set pos if asked
    if(set_joint_pos_no_dynamics_)
    {
        for(unsigned j=0; j<joints_idx_.size(); j++)
#ifdef GAZEBO_6
            gazebo_joints_[joints_idx_[j]]->SetPosition(0,jnt_pos_no_dyn_[j]);
#else
	    gazebo_joints_[joints_idx_[j]]->SetAngle(0,jnt_pos_no_dyn_[j]);
#endif
        // Set jnt pos
        jnt_pos_cmd_ = jnt_pos_ = jnt_pos_no_dyn_;
        set_joint_pos_no_dynamics_ = false;
    }


    FlowStatus jnt_trq_cmd_fs
                    ,jnt_pos_cmd_fs
                    ,cart_pos_cmd_fs
                    ,cart_wrench_cmd_fs
                    ,fri_to_krl_cmd_fs
                    ,jnt_imp_cmd_fs
                    ,cart_imp_cmd_fs;

    fri_to_krl_cmd_fs = port_ToKRL.readNewest(fri_to_krl);
    fri_state.timestamp = rtt_rosclock::host_now().toNSec();

    // Update Robot Internal State to mimic KRC
    // Update cmds to FRI
    if(fri_to_krl_cmd_fs == NewData)
    {
        for(int i=0;i<FRI_USER_SIZE;i++)
        {
            fri_from_krl.realData[i]=fri_to_krl.realData[i];
            fri_from_krl.intData[i]=fri_to_krl.intData[i];
        }
        fri_from_krl.boolData=fri_to_krl.boolData;
    }
    // Update robot status
    switch(fri_to_krl.intData[1])
    {
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
    // Update robot power state + fri state
    switch(static_cast<FRI_CTRL>(robot_state.control))
    {
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
        safetyChecks(jnt_pos_,jnt_vel_,jnt_trq_);

    bool exit_loop = false;
    int n_wait=0;
    tstart_wait = TimeService::Instance()->getNSecs();
    do
    {
        jnt_trq_cmd_fs = port_JointTorqueCommand.readNewest(jnt_trq_cmd_);
        jnt_pos_cmd_fs = port_JointPositionCommand.readNewest(jnt_pos_cmd_);
        cart_pos_cmd_fs = port_CartesianPositionCommand.readNewest(cart_pos_cmd_);

        if(jnt_pos_cmd_fs == NoData && jnt_trq_cmd_fs == NoData && cart_pos_cmd_fs == NoData)
            break;

        switch(static_cast<FRI_CTRL>(robot_state.control)){
            case FRI_CTRL_JNT_IMP:
                if(jnt_trq_cmd_fs == NewData || jnt_pos_cmd_fs == NewData)
                {
                    exit_loop = true;
                    nb_loops_++;
                }
                break;
            case FRI_CTRL_OTHER:
                exit_loop = true;
                break;
            case FRI_CTRL_POSITION:
                if(jnt_pos_cmd_fs == NewData || jnt_pos_cmd_fs == NoData)
                {
                    exit_loop = true;
                    nb_loops_++;
                }
                break;
            case FRI_CTRL_CART_IMP:
                if(cart_pos_cmd_fs == NewData || cart_pos_cmd_fs == NoData)
                {
                    exit_loop = true;
                    nb_loops_++;
                }
                break;
            default:
                break;
        }
        if(!sync_with_cmds_)
            break;

        if(false && n_wait % 10 == 0 && !exit_loop)
        {
            log(RTT::Debug) << getName() << " UpdateHook() : waiting "<< TimeService::Instance()->getNSecs()
            <<" - pos:"<<jnt_pos_cmd_fs
            <<" - trq:"<<jnt_trq_cmd_fs
            <<" - crt:"<<cart_pos_cmd_fs
            <<" - set_joint_pos_no_dynamics_:"<<set_joint_pos_no_dynamics_
            << endlog();
        }
        if(!exit_loop){
            n_wait++;
            usleep(250);
            if(n_wait * 250. > timeout_s*1e6)
            {
                log(RTT::Debug) << getName() << " Timeout at "<< TimeService::Instance()->getNSecs() << endlog();
                break;
            }
        }
    }while(!exit_loop);

    TimeService::nsecs tduration_wait = TimeService::Instance()->getNSecs(tstart_wait);

    cart_wrench_cmd_fs = port_CartesianWrenchCommand.readNewest(cart_wrench_cmd_);
    jnt_imp_cmd_fs = port_JointImpedanceCommand.readNewest(jnt_imp_cmd_);
    cart_imp_cmd_fs = port_CartesianImpedanceCommand.readNewest(cart_imp_cmd_);

    // Update joint impedance gains
    if(jnt_imp_cmd_fs == NewData)
        updateJointImpedance(jnt_imp_cmd_);

    // Update cartesian impedance gains
    if(cart_imp_cmd_fs == NewData)
        updateCartesianImpedance(cart_imp_cmd_);

    // Copy State to KDL
    jnt_pos_kdl_.data = jnt_pos_;
    jnt_vel_kdl_.data = jnt_vel_;
    jnt_acc_kdl_.data.setConstant(0.0);

    // Compute MassMatrix
    id_dyn_solver->JntToMass(jnt_pos_kdl_,mass_kdl_);

    // Compute Inverse dynamics [q,qdot] => T
    std::fill(f_ext_.begin(),f_ext_.end(),Wrench::Zero());
    id_rne_solver->CartToJnt(jnt_pos_kdl_,jnt_vel_kdl_,jnt_acc_kdl_,f_ext_,jnt_trq_kdl_);

    // Compute Gravity terms G(q)
    id_dyn_solver->JntToGravity(jnt_pos_kdl_,jnt_trq_grav_kdl_);

    // Compute Coriolis terms b(q,qdot)
    id_dyn_solver->JntToCoriolis(jnt_pos_kdl_,jnt_vel_kdl_,jnt_trq_coriolis_kdl_);

    Map<VectorXd>(joint_states_dyn_.position.data(),joints_idx_.size()) = jnt_trq_grav_kdl_.data;
    Map<VectorXd>(joint_states_dyn_.velocity.data(),joints_idx_.size()) = jnt_trq_coriolis_kdl_.data;
    Map<VectorXd>(joint_states_dyn_.effort.data(),joints_idx_.size()) = jnt_trq_kdl_.data;

    jnt_to_jac_solver->JntToJac(jnt_pos_kdl_,jac_,kdl_chain_.getNrOfSegments());
    fk_vel_solver->JntToCart(JntArrayVel(jnt_pos_kdl_,jnt_vel_kdl_),ee_framevel_kdl_,kdl_chain_.getNrOfSegments());
    ee_twist_kdl_ = ee_framevel_kdl_.GetTwist();
    ee_frame_kdl_ = ee_framevel_kdl_.GetFrame();

    tf::poseKDLToMsg(ee_frame_kdl_,cart_pos_);
    tf::twistKDLToMsg(ee_twist_kdl_,cart_twist_);

    cart_pos_stamped_.header.frame_id = root_link_;
    cart_pos_stamped_.pose = cart_pos_;


    //ee_twist_des_kdl_.SetToZero();
    SetToZero(ee_twist_des_kdl_);
    ee_twist_diff_kdl_ = diff(ee_twist_kdl_,ee_twist_des_kdl_);
    tf::twistKDLToEigen(ee_twist_diff_kdl_,Xd_err_);

    if(cart_pos_cmd_fs == NewData){
        tf::poseMsgToKDL(cart_pos_cmd_,ee_frame_des_kdl_);
        ee_frame_diff_kdl_ = diff(ee_frame_kdl_,ee_frame_des_kdl_);
        tf::twistKDLToEigen(ee_frame_diff_kdl_,X_err_);
    }


    tf::wrenchMsgToEigen(cart_wrench_cmd_,F_cmd_);


    cart_wrench_stamped_.header.frame_id = tip_link_;
    cart_wrench_stamped_.wrench = cart_wrench_;

    jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * jnt_trq_grav_kdl_.data;

    switch(static_cast<FRI_CTRL>(robot_state.control)){
        case FRI_CTRL_JNT_IMP:
            // Joint Impedance part
            jnt_trq_gazebo_cmd_ += kp_.asDiagonal()*(jnt_pos_cmd_ - jnt_pos_) - kd_.asDiagonal()*jnt_vel_ ;
            // Additional torque
            jnt_trq_gazebo_cmd_ += jnt_trq_cmd_;

            set_brakes_ = (jnt_trq_cmd_fs != NewData);

            break;
        case FRI_CTRL_OTHER:
        case FRI_CTRL_POSITION:
            // Joint Position (Joint Imp with kd fixed)
            jnt_trq_gazebo_cmd_ += kp_default_.asDiagonal()*(jnt_pos_cmd_-jnt_pos_) - kd_default_.asDiagonal()*jnt_vel_ ;

            set_brakes_ = (jnt_pos_cmd_fs != NewData);
            break;
        case FRI_CTRL_CART_IMP:
            //Cartesian Impedance Control
            jnt_trq_gazebo_cmd_ += jac_.data.transpose()*(kcp_.asDiagonal()*(X_err_ + F_cmd_) + kcd_.asDiagonal()*(Xd_err_));

            set_brakes_ = (cart_pos_cmd_fs != NewData);

            break;
        default:
            break;
    }

    ros::Time now = rtt_rosclock::host_now();
    //Update status
    cart_pos_stamped_.header.stamp = now;
    cart_wrench_stamped_.header.stamp = now;
    joint_states_.header.stamp = now;

    joint_states_cmd_.header.stamp = now;
    joint_states_dyn_.header.stamp = now;

    Map<VectorXd>(joint_states_.position.data(),joints_idx_.size()) = jnt_pos_;
    Map<VectorXd>(joint_states_.velocity.data(),joints_idx_.size()) = jnt_vel_;
    Map<VectorXd>(joint_states_.effort.data(),joints_idx_.size()) = jnt_trq_;

    Map<VectorXd>(joint_states_cmd_.position.data(),joints_idx_.size()) = jnt_pos_cmd_;
    Map<VectorXd>(joint_states_cmd_.effort.data(),joints_idx_.size()) = jnt_trq_gazebo_cmd_ - jnt_trq_grav_kdl_.data;

    port_JointStates.write(joint_states_);
    port_JointStatesCommand.write(joint_states_cmd_);
    port_JointStatesDynamics.write(joint_states_dyn_);

    port_JointPosition.write(jnt_pos_);
    port_JointVelocity.write(jnt_vel_);
    port_JointTorque.write(jnt_trq_);
    port_GravityTorque.write(jnt_trq_grav_kdl_.data);

    port_CartesianPosition.write(cart_pos_);
    port_CartesianPositionStamped.write(cart_pos_stamped_);
    port_CartesianVelocity.write(cart_twist_);
    port_CartesianWrench.write(cart_wrench_);
    port_CartesianWrenchStamped.write(cart_wrench_stamped_);

    port_Jacobian.write(jac_);
    port_MassMatrix.write(mass_kdl_.data);

    port_FromKRL.write(fri_from_krl);

    port_RobotState.write(robot_state);
    port_FRIState.write(fri_state);

    rtt_sem_.signal();


    TimeService::nsecs tduration = TimeService::Instance()->getNSecs(tstart);
    log(RTT::Debug) << getName() << " UpdateHook()  END "
    <<tstart
    <<"\n - jnt_trq_cmd_fs:" << jnt_trq_cmd_fs
    <<"\n - jnt_pos_cmd_fs:" << jnt_pos_cmd_fs
    <<"\n - jnt_trq_cmd_:"<<jnt_trq_cmd_.transpose()
    <<"\n - jnt_pos_cmd_:"<<jnt_pos_cmd_.transpose()
    <<"\n - sync_with_cmds:"<<sync_with_cmds_
    <<"\n - Waited for cmds :"<<n_wait
    <<"\n - Waited for cmds ns :"<<tduration_wait
    <<"\n - set_brakes:"<<set_brakes_
    <<"\n - robot_state.control:"<<robot_state.control
    <<"\n -- kp: "<<kp_.transpose()
    <<"\n -- kd: "<<kd_.transpose()
    <<"\n -- duration: "<<tduration<< endlog();

}
void LWRSim::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    /*RTT::os::MutexTryLock trylock(gazebo_mutex_);
    if(trylock.isSuccessful() == false) {
        log(RTT::Debug) << getName() << " gazeboUpdateHook() : mutex locked, waiting.. "<< TimeService::Instance()->getNSecs() << endlog();
        rtt_sem_.wait();
        log(RTT::Debug) << getName() << " gazeboUpdateHook() : let's go!"<< TimeService::Instance()->getNSecs() << endlog();
    }*/
    // Locking gazebo should be safe as UpdateHook() waits on updateworld end ?
    log(RTT::Debug) << getName() << " gazeboUpdateHook() BEGIN "<< TimeService::Instance()->getNSecs() << endlog();
    RTT::os::MutexLock lock(gazebo_mutex_);
    // Checking if model is correct
    if(model.get() == NULL){
        log(RTT::Debug) << getName() << " gazeboUpdateHook() : model is NULL "<< TimeService::Instance()->getNSecs() << endlog();
        return;
    }
    log(RTT::Debug) << getName() << " gazeboUpdateHook() START "<< TimeService::Instance()->getNSecs() << endlog();

    // Read From gazebo simulation
    for(unsigned j=0; j<joints_idx_.size(); j++) {
        jnt_pos_[j] = gazebo_joints_[joints_idx_[j]]->GetAngle(0).Radian();
        jnt_vel_[j] = gazebo_joints_[joints_idx_[j]]->GetVelocity(0);
        jnt_trq_[j] = gazebo_joints_[joints_idx_[j]]->GetForce(0u);
    }


    if(port_JointTorqueCommand.connected() || port_JointPositionCommand.connected())
    {
        // Enable gravity for everyone
       for(gazebo::physics::Link_V::iterator it = model_links_.begin();
             it != model_links_.end();++it)
             (*it)->SetGravityMode(true);

        // Set Gravity Mode or specified links
        for(std::map<gazebo::physics::LinkPtr,bool>::iterator it = this->gravity_mode_.begin();
            it != this->gravity_mode_.end();++it)
                    it->first->SetGravityMode(it->second);

        for(unsigned j=0; j<joints_idx_.size(); j++)
            gazebo_joints_[joints_idx_[j]]->SetForce(0,jnt_trq_gazebo_cmd_[j]);
    }
    else
    {
        // If no one is connected, stop gravity
       for(gazebo::physics::Link_V::iterator it = model_links_.begin();
             it != model_links_.end();++it)
             (*it)->SetGravityMode(false);
    }
    log(RTT::Debug) << getName() << " gazeboUpdateHook() END "<< TimeService::Instance()->getNSecs() << endlog();
    this->trigger();
}
