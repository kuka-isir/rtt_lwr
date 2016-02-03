#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <ros/service.h>
#include <ros/param.h>
#include <ros/this_node.h>
#include <ocl/DeploymentComponent.hpp>

using namespace RTT;
using namespace std;

class RosHelper: public RTT::Service
{
public:
    RosHelper(OCL::DeploymentComponent* deployer) :
    Service("ros_helper", static_cast<RTT::TaskContext*>(deployer)),
    deployer_(deployer)
    {
        this->addOperation("connectPeerCORBA", &RosHelper::connectPeerCORBA, this);
        this->addOperation("waitForROSService", &RosHelper::waitForROSService, this);
        this->addOperation("getParam", &RosHelper::getParam, this);
        this->addOperation("getRobotName", &RosHelper::getRobotName, this);
        this->addOperation("isSim", &RosHelper::isSim, this);
        this->addOperation("getThisNodeName", &RosHelper::getThisNodeName, this);
        this->addOperation("getThisNodeNamespace", &RosHelper::getThisNodeNamespace, this);
    }
    string getRobotName()
    {
        string robot_name;
        std::cout << "I am "<<getThisNodeName()<<" living in "<<getThisNodeNamespace()<<std::endl;
        ros::param::get("robot_name", robot_name);
        return robot_name;
    }
    string getThisNodeName()
    {
        return ros::this_node::getName();
    }
    bool isSim()
    {
        bool is_sim = false;
        ros::param::get("/use_sim_time", is_sim);
        return is_sim;
    }
    string getThisNodeNamespace()
    {
        string ns(ros::this_node::getNamespace());
        size_t pos = ns.find( "//" );
        if ( pos != string::npos ) {
           ns.replace( pos, 2, "/" );   // 5 = length( $name )
        }
        return ns;
    }
    string getParam(const string& res_param_name)
    {
        string param;
        ros::param::get(res_param_name, param);
        return param;
    }
    bool waitForROSService(std::string service_name, double service_timeout_s)
    {
        return ros::service::waitForService(service_name, service_timeout_s*1E3);
    }
    bool connectPeerCORBA(const std::string& interface_name,const std::string& peer_name)
    {
        if(getOwner() == NULL) return false;

        if(peer_name.empty() || interface_name.empty()) return false;

        if(this->getOwner()->hasPeer(peer_name) == true) return false;

        if(this->getOwner()->hasPeer(interface_name) == false) return false;

        if(this->getOwner()->getPeer(interface_name)->hasPeer(peer_name) == false) return false;

        return this->getOwner()->connectPeers(this->getOwner()->getPeer(interface_name)->getPeer(peer_name));
    }
private:
    OCL::DeploymentComponent *deployer_;
};

bool loadROSHelper(RTT::TaskContext *tc) {
  if(tc == 0)
  {
      RTT::log(RTT::Error) << "RTT::TaskContext *tc is NULL" <<RTT::endlog();
      return false;
  }
  OCL::DeploymentComponent *deployer = dynamic_cast<OCL::DeploymentComponent*>(tc);

  if(!deployer) {
    RTT::log(RTT::Error) << "The ros_helper service must be loaded on a valid OCL::DeploymentComponent" <<RTT::endlog();
    return false;
  }

  deployer->import("rtt_rosnode");

  if(!ros::isInitialized()) {
    RTT::log(RTT::Error) << "The rtt_rosdeployment plugin cannot be used without the rtt_rosnode plugin. Please load rtt_rosnode." << RTT::endlog();

    return false;
  }

  RTT::Service::shared_ptr sp( new RosHelper( deployer ) );
  return tc->provides()->addService( sp );
}
extern "C" {
  RTT_EXPORT bool loadRTTPlugin(RTT::TaskContext* tc);
  bool loadRTTPlugin(RTT::TaskContext* tc) {
    if(tc == 0) return true;
    return loadROSHelper(tc);
  }
  RTT_EXPORT RTT::Service::shared_ptr createService();
  RTT::Service::shared_ptr createService() {
    RTT::Service::shared_ptr sp;
    return sp;
  }
  RTT_EXPORT std::string getRTTPluginName();
  std::string getRTTPluginName() {
    return "ros_helper";
  }
  RTT_EXPORT std::string getRTTTargetName();
  std::string getRTTTargetName() {
    return OROCOS_TARGET_NAME;
  }
}
