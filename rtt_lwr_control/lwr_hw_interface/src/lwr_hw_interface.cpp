///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, Stefan Kohlbrecher, TU Darmstadt
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of TU Darmstadt nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <lwr_hw_interface/lwr_hw_interface.h>
#include <controller_manager/controller_manager.h>

namespace lwr_ros_control
{

  LWRHwInterface::LWRHwInterface()
  {

    // Could read joint names from API or URDF to name two examples,
    // here we just make something up.
    std::vector<std::string> joint_names;
    joint_names.push_back("lwr_0_joint");
    joint_names.push_back("lwr_1_joint");
    joint_names.push_back("lwr_2_joint");
    joint_names.push_back("lwr_3_joint");
    joint_names.push_back("lwr_4_joint");
    joint_names.push_back("lwr_5_joint");
    joint_names.push_back("lwr_6_joint");

    cmd_joint_vel_.resize(joint_names.size());
    cmd_joint_position_.resize(joint_names.size());
    state_joint_position_.resize(joint_names.size());
    state_joint_velocity_.resize(joint_names.size());
    state_joint_effort_.resize(joint_names.size());

    for(size_t i=0; i<joint_names.size(); ++i)
    {


      hardware_interface::JointStateHandle state_handle(joint_names[i], &state_joint_position_[i], &state_joint_velocity_[i], &state_joint_effort_[i]);
      joint_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_names[i]), &state_joint_position_[i]);
      position_joint_interface_.registerHandle(pos_handle);

      hardware_interface::JointHandle vel_handle(joint_state_interface_.getHandle(joint_names[i]), &state_joint_velocity_[i]);
      velocity_joint_interface_.registerHandle(vel_handle);

    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
  }

  void LWRHwInterface::cleanup()
  {

  }

  void LWRHwInterface::read()
  {
    // Read data from hardware here. In this example, nothing is done for the moment   
  }

  void LWRHwInterface::write()
  {
     // Write data to hardware here. In this example, nothing is done for the moment
  }

}
