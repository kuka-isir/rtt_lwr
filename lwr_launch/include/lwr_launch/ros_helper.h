#ifndef __ROS_HELPER_H__
#define __ROS_HELPER_H__
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <ros/service.h>
#include <ros/param.h>
#include <ros/this_node.h>
#include <ocl/DeploymentComponent.hpp>

#include <boost/smart_ptr.hpp>
#include <rtt/scripting/Scripting.hpp>

#include <rtt/ServiceRequester.hpp>

class ROSHelper : public RTT::ServiceRequester
{

public:
    ROSHelper(RTT::TaskContext *owner);
    RTT::OperationCaller<RTT::TaskContext *(void)> getServiceOwner;
    RTT::OperationCaller<RTT::TaskContext *(const std::string &)> getTaskContext;
    RTT::OperationCaller<RTT::TaskContext *(void)> getOwner;
    RTT::OperationCaller<bool(void)> isSim;
    RTT::OperationCaller<std::string(void)> getTfPrefix;
    RTT::OperationCaller<std::string(void)> getEnvName;
    RTT::OperationCaller<bool(RTT::TaskContext *)> addComponent;
    RTT::OperationCaller<bool(const std::string&)> rosServiceCall;
    RTT::OperationCaller<void(void)> printArgv;
    RTT::OperationCaller<std::string(void)> getRobotNs;
    RTT::OperationCaller<std::string(void)> getRobotName;
    RTT::OperationCaller<std::string(void)> getThisNodeName;
    RTT::OperationCaller<std::string(void)> getThisNodeNamespace;
    RTT::OperationCaller<std::string(const std::string &)> getParam;
    RTT::OperationCaller<bool(const std::string &,const std::string &)> connectPeerCORBA;
    RTT::OperationCaller<bool(const std::string &,double)> waitForROSService;
    RTT::OperationCaller<bool(const std::vector<std::string>&)> parseArgv;

};

class OS : public RTT::ServiceRequester
{

public:
    OS(RTT::TaskContext* owner ):
    RTT::ServiceRequester("os",owner),
    argv("argv"),
    argc("argc"),
    getenv("getenv"),
    isenv("isenv"),
    setenv("setenv")
    {
        this->addOperationCaller(argv);
        this->addOperationCaller(argc);
        this->addOperationCaller(getenv);
        this->addOperationCaller(isenv);
        this->addOperationCaller(setenv);
    }
    RTT::OperationCaller<std::vector<std::string>(void)> argv;
    RTT::OperationCaller<int(void)> argc;
    RTT::OperationCaller<std::string(const std::string&)> getenv;
    RTT::OperationCaller<bool(const std::string&)> isenv;
    RTT::OperationCaller<bool(const std::string&, const std::string&)> setenv;
};

#endif
