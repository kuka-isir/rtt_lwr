// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_example/rtt_lwr_example.hpp"


lwr::RTTLWRExample::RTTLWRExample(const std::string& name): RTTLWRAbstract(name)
{
    RTT::log(RTT::Info) << "Example Created !" << RTT::endlog();
}
lwr::RTTLWRExample::~RTTLWRExample()
{

}

void lwr::RTTLWRExample::updateHook()
{

}
bool lwr::RTTLWRExample::configureHook()
{
return lwr::RTTLWRAbstract::configureHook();
}

