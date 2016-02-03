#ifndef __ROS_HELPER_H__
#define __ROS_HELPER_H__

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

namespace rtt_rosparam {

  class ROSParam : public RTT::ServiceRequester
  {

  public:
    ROSParam(RTT::TaskContext *owner) :
      RTT::ServiceRequester("ros_helper",owner),
      connectSim("getParam"),
      connectPeers("connectPeerCORBA"),
      connectPeers("getThisNodeName"),
      connectPeers("isSim"),
      connectPeers("getThisNodeNamespace"),
      connectPeers("waitForROSService"),
      connectPeers("getRobotName")
    {
      this->addOperationCaller(getParam);
      this->addOperationCaller(connectPeerCORBA);
      this->addOperationCaller(waitForROSService);
      this->addOperationCaller(getThisNodeName);
      this->addOperationCaller(getThisNodeNamespace);
      this->addOperationCaller(isSim);
      this->addOperationCaller(getRobotName);
    }
    RTT::OperationCaller<bool(void)> isSim;
    RTT::OperationCaller<std::string(void)> getRobotName;
    RTT::OperationCaller<std::string(void)> getThisNodeName;
    RTT::OperationCaller<std::string(void)> getThisNodeNamespace;
    RTT::OperationCaller<std::string(const std::string &)> getParam;
    RTT::OperationCaller<bool(const std::string &,const std::string &)> connectPeerCORBA;
    RTT::OperationCaller<bool(const std::string &,double)> waitForROSService;

  };
}

#endif
