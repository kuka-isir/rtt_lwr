#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <Eigen/Dense>
#include <rtt/os/Timer.hpp>
#include <rtt_rosclock/rtt_rosclock.h>

class LWRGazeboComponent : public RTT::TaskContext
{
public:


    LWRGazeboComponent(std::string const& name) :
        RTT::TaskContext(name),
        steps_rtt_(0),
        steps_gz_(0),
        n_joints_(0),
        last_gz_update_time_(0,0)
    {
        // Add required gazebo interfaces
        this->provides("gazebo")->addOperation("configure",&LWRGazeboComponent::gazeboConfigureHook,this,RTT::ClientThread);
        this->provides("gazebo")->addOperation("update",&LWRGazeboComponent::gazeboUpdateHook,this,RTT::ClientThread);

        this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
        this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
        this->ports()->addPort("JointVelocityCommand", port_JointVelocityCommand).doc("");

        this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
        this->ports()->addPort("JointTorque", port_JointTorque).doc("");
        this->ports()->addPort("JointPosition", port_JointPosition).doc("");

        this->provides("debug")->addAttribute("jnt_pos",jnt_pos_);
        this->provides("debug")->addAttribute("jnt_vel",jnt_vel_);
        this->provides("debug")->addAttribute("jnt_trq",jnt_trq_);
        this->provides("debug")->addAttribute("gz_time",gz_time_);
        this->provides("debug")->addAttribute("write_duration",write_duration_);
        this->provides("debug")->addAttribute("read_duration",read_duration_);
        this->provides("debug")->addAttribute("rtt_time",rtt_time_);
        this->provides("debug")->addAttribute("steps_rtt",steps_rtt_);
        this->provides("debug")->addAttribute("steps_gz",steps_gz_);
        this->addAttribute("n_joints",n_joints_);

    }

    //! Called from gazebo
    virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model)
    {
        if(model.get() == NULL) {
            RTT::log(RTT::Error)<<"No model could be loaded"<<RTT::endlog();
            return false;
        }

        // Get the joints
        gazebo_joints_ = model->GetJoints();
        n_joints_ = gazebo_joints_.size() -1 ;//!\\ This contains base_joint

        // Get the joint names
        for(std::vector<gazebo::physics::JointPtr>::iterator it=gazebo_joints_.begin();
            it != gazebo_joints_.end();
            ++it)
        {
            joint_names_.push_back((**it).GetName());
            RTT::log(RTT::Info)<<"Adding joint "<<(**it).GetName()<<RTT::endlog();
        }
        jnt_pos_.resize(n_joints_);
        jnt_vel_.resize(n_joints_);
        jnt_trq_.resize(n_joints_);

        jnt_pos_cmd_.resize(n_joints_);
        jnt_vel_cmd_.resize(n_joints_);
        jnt_trq_cmd_.resize(n_joints_);

        for(unsigned j=0; j < n_joints_; j++){
            jnt_pos_cmd_[j] = 0.0;
            jnt_vel_cmd_[j] = 0.0;
            jnt_trq_cmd_[j] = 0.0;
            jnt_pos_[j] = 0.0;
            jnt_vel_[j] = 0.0;
            jnt_trq_[j] = 0.0;
        }

        port_JointPosition.setDataSample(jnt_pos_);
        port_JointVelocity.setDataSample(jnt_vel_);
        port_JointTorque.setDataSample(jnt_trq_);

        RTT::log(RTT::Info)<<"Done configuring gazebo"<<RTT::endlog();
        last_update_time_ = rtt_rosclock::rtt_now();
        return true;
    }

    //! Called from Gazebo
    virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model)
    {
        if(model.get() == NULL) {return;}

        // Synchronize with update()
        RTT::os::MutexTryLock trylock(gazebo_mutex_);
#ifdef XENOMAI
        if(true){
#else
        if(trylock.isSuccessful()) {
#endif
            // Increment simulation step counter (debugging)
            steps_gz_++;

            // Get the RTT and gazebo time for debugging purposes
            rtt_time_ = 1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
            gazebo::common::Time gz_time = model->GetWorld()->GetSimTime();
            gz_time_ = (double)gz_time.sec + ((double)gz_time.nsec)*1E-9;

            // Get the wall time
            gazebo::common::Time gz_wall_time = gazebo::common::Time::GetWallTime();
            wall_time_ = (double)gz_wall_time.sec + ((double)gz_wall_time.nsec)*1E-9;

            // Get state
            for(unsigned j=0; j < n_joints_; j++) {
                jnt_pos_[j] = gazebo_joints_[j+1]->GetAngle(0).Radian();
                jnt_vel_[j] = gazebo_joints_[j+1]->GetVelocity(0);
                //jnt_trq_[j] = gazebo_joints_[j+1]->GetForce(0u);//jnt_trq_cmd_[j];
                jnt_trq_[j] = jnt_trq_cmd_[j];
            }



            // Write command
            if(gazebo_joints_.size())
                gazebo_joints_[0]->SetForce(0,gazebo_joints_[0]->GetForce(0u));
            for(unsigned int j=0; j < n_joints_; j++)
                gazebo_joints_[j+1]->SetForce(0,jnt_trq_cmd_[j]);

        }else{
            RTT::log(RTT::Error)<< "gazeboUpdateHook locked" <<RTT::endlog();
        }

    }


    virtual bool configureHook()
    {
        RTT::log(RTT::Info)<<"Done configuring hook"<<RTT::endlog();
        return true;
    }

    virtual void updateHook()
    {
        // Synchronize with gazeboUpdate()
        RTT::os::MutexLock lock(gazebo_mutex_);

        ros::Time gz_time = rtt_rosclock::rtt_now();
        RTT::Seconds gz_period = (gz_time - last_gz_update_time_).toSec();
        gz_period_ = gz_period;

        // Increment simulation step counter (debugging)
        steps_rtt_++;

        // Get command from ports

        RTT::os::TimeService::ticks read_start = RTT::os::TimeService::Instance()->getTicks();

        //jnt_pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
        //jnt_vel_fs = port_JointVelocityCommand.read(jnt_vel_cmd_);
        // Our only input is Torque on Kuka LWR
        jnt_trq_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);

        read_duration_ = RTT::os::TimeService::Instance()->secondsSince(read_start);

        // Write state to ports
        RTT::os::TimeService::ticks write_start = RTT::os::TimeService::Instance()->getTicks();
        /*if(port_JointPosition.connected()
                && port_JointVelocity.connected()
                && port_JointPosition.connected())
        {*/
            port_JointVelocity.write(jnt_vel_);
            port_JointPosition.write(jnt_pos_);
            port_JointTorque.write(jnt_trq_);
        /*}*/

        write_duration_ = RTT::os::TimeService::Instance()->secondsSince(write_start);
    }

protected:

    //! Synchronization
    RTT::os::MutexRecursive gazebo_mutex_;

    //! The Gazebo Model
    //! The gazebo
    std::vector<gazebo::physics::JointPtr> gazebo_joints_;
    std::vector<std::string> joint_names_;

    RTT::InputPort<std::vector<double> > port_JointVelocityCommand;
    RTT::InputPort<std::vector<double> > port_JointTorqueCommand;
    RTT::InputPort<std::vector<double> > port_JointPositionCommand;

    RTT::OutputPort<std::vector<double> > port_JointVelocity;
    RTT::OutputPort<std::vector<double> > port_JointTorque;
    RTT::OutputPort<std::vector<double> > port_JointPosition;
    RTT::FlowStatus jnt_pos_fs,jnt_vel_fs,jnt_trq_fs;

    std::vector<double> jnt_vel_,jnt_trq_,jnt_pos_;
    std::vector<double> jnt_vel_cmd_,jnt_trq_cmd_,jnt_pos_cmd_;

    //! RTT time for debugging
    double rtt_time_;
    //! Gazebo time for debugging
    double gz_time_;
    double wall_time_;

    ros::Time last_gz_update_time_;
    RTT::Seconds gz_period_;
    RTT::Seconds gz_duration_;

    ros::Time last_update_time_;
    RTT::Seconds rtt_period_;
    RTT::Seconds read_duration_;
    RTT::Seconds write_duration_;

    int steps_gz_;
    int steps_rtt_;
    int n_joints_;
    double period_sim_;
    double period_wall_;
};

ORO_LIST_COMPONENT_TYPE(LWRGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();
