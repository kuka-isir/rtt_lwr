#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>

#include <Eigen/Dense>
#include <boost/graph/graph_concepts.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt_rosclock/rtt_rosclock.h>
#include <boost/thread/mutex.hpp>
class LWRGazeboComponent : public RTT::TaskContext
{
public:


    LWRGazeboComponent(std::string const& name) :
        RTT::TaskContext(name),
        steps_rtt_(0),
        steps_gz_(0),
        n_joints_(0),
        rtt_done(true),
        gazebo_done(false),
        new_data(true),
        cnt_lock_(100),
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
        this->provides("debug")->addAttribute("period_sim",period_sim_);
        this->provides("debug")->addAttribute("period_wall",period_wall_);
        this->addProperty("n_joints",n_joints_);
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
        model_links_ = model->GetLinks();
        n_joints_ = gazebo_joints_.size() -1 ;//!\\ This contains base_joint

        // Get the joint names
        for(std::vector<gazebo::physics::JointPtr>::iterator it=gazebo_joints_.begin();
            it != gazebo_joints_.end();
            ++it)
        {
            (*it)->SetProvideFeedback(true);
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

        port_JointPosition.keepLastWrittenValue(true);
        port_JointVelocity.keepLastWrittenValue(true);
        port_JointTorque.keepLastWrittenValue(true);
        
        RTT::log(RTT::Info)<<"Done configuring gazebo"<<RTT::endlog();
        last_update_time_ = rtt_rosclock::rtt_now();
        return true;
    }

    //! Called from Gazebo
    virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model)
    {
        if(model.get() == NULL) {return;}

        // Synchronize with update()
#ifdef XENOMAI
        //gazebo_mutex_.lock();
        if(rtt_done){
            gazebo_done=false;
#else
        RTT::os::MutexTryLock trylock(gazebo_mutex_);
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
            gazebo::physics::JointWrench jw;
            for(unsigned j=0; j < n_joints_; j++) {
                jnt_pos_[j] = gazebo_joints_[j+1]->GetAngle(0).Radian();
                jnt_vel_[j] = gazebo_joints_[j+1]->GetVelocity(0);
                jnt_trq_[j] = gazebo_joints_[j+1]->GetForce(0u);//jnt_trq_cmd_[j];
                //jw = gazebo_joints_[j+1]->GetForceTorque(0u);
                //RTT::log(RTT::Error)<<"j"<<j<<" "<<jw.body1Torque.x<<" "<<jw.body1Torque.y<<" "<<jw.body1Torque.z<<RTT::endlog(); 
                //jnt_trq_[j] = jw.body1Torque.x;
                //jnt_trq_[j] = jnt_trq_cmd_[j];
            }


            if(jnt_trq_fs != RTT::NewData)
            {
                
                // Simulates the breaks
                /*for(gazebo::physics::Link_V::iterator it = model_links_.begin();
                    it != model_links_.end();++it)
                    (*it)->SetGravityMode(false);
            }else{
                for(gazebo::physics::Link_V::iterator it = model_links_.begin();
                    it != model_links_.end();++it)
                    (*it)->SetGravityMode(true);*/
            }
            // Write command
            if(gazebo_joints_.size())
                gazebo_joints_[0]->SetForce(0,gazebo_joints_[0]->GetForce(0u));
            for(unsigned int j=0; j < n_joints_; j++)
                gazebo_joints_[j+1]->SetForce(0,jnt_trq_cmd_[j]);
        }else{
            RTT::log(RTT::Error)<< "gazeboUpdateHook locked" <<RTT::endlog();
        }
     #ifdef XENOMAI
     gazebo_done = true;
#endif
    }


    virtual bool configureHook()
    {
        return true;
    }

    virtual void updateHook()
    {
        // Synchronize with gazeboUpdate()
#ifndef XENOMAI
       RTT::os::MutexLock lock(gazebo_mutex_);
#else
       if(true){
           rtt_done=false;
           
#endif

        static double last_update_time_sim;
        period_sim_ = rtt_time_ - last_update_time_sim;
        last_update_time_sim = rtt_time_;

        // Compute period in wall clock
        static double last_update_time_wall;
        period_wall_ = wall_time_ - last_update_time_wall;
        last_update_time_wall = wall_time_;


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

        port_JointVelocity.write(jnt_vel_);
        port_JointPosition.write(jnt_pos_);
        port_JointTorque.write(jnt_trq_);
        
        write_duration_ = RTT::os::TimeService::Instance()->secondsSince(write_start);
#ifdef XENOMAI
        //gazebo_mutex_.unlock();
        rtt_done=true;
       }else{
           RTT::log(RTT::Error)<< "gazeboUpdateHook not done" <<RTT::endlog();
       }
#endif
    }
protected:

    //! Synchronization
#ifndef XENOMAI
    RTT::os::MutexRecursive gazebo_mutex_;
#else
    boost::mutex gazebo_mutex_;
#endif
    //! The Gazebo Model
    //! The gazebo
    std::vector<gazebo::physics::JointPtr> gazebo_joints_;
    gazebo::physics::Link_V model_links_;
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
    unsigned int n_joints_;
    int cnt_lock_;
    double period_sim_;
    double period_wall_;
    boost::atomic<bool> new_data;
    boost::atomic<bool> rtt_done,gazebo_done;
};

ORO_LIST_COMPONENT_TYPE(LWRGazeboComponent)
ORO_CREATE_COMPONENT_LIBRARY();
