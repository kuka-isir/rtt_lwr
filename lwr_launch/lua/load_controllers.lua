
function load_controllers(depl, scheme, prefix)

  --[[ default arguments --]]
  local prefix = prefix or ""

  --[[ convenience vars --]]
  cp = rtt.Variable("ConnPolicy")
  depl:import("rtt_ros")
  gs:provides("ros"):import("lcsr_controllers")

  --[[ get required components --]]
  barrett_manager = depl:getPeer(prefix.."barrett_manager")
  effort_sum = depl:getPeer(prefix.."effort_sum")
  tf = depl:getPeer("tf")

  local wam = barrett_manager:getName()..".wam"

  --[[ get a port name from a task/port --]]
  local function port_name(task, port_name)
    if type(task) == "string" then
      task_name = task
    else
      task_name = task:getName()
    end

    return task_name.."."..port_name
  end

  --[[ simpler connect function --]]
  local function connect(from_task, from_port,to_task, to_port, policy)
    policy = policy or cp
    return depl:connect(
      port_name(from_task, from_port),
      port_name(to_task, to_port),
      policy);
  end

  --[[ load and get a component --]]
  local function loadComponent(name, component_type)
    depl:loadComponent(name, component_type);
    return depl:getPeer(name)
  end

  --[[ Create joint-space PID controller loop --]]
  pid_name = prefix.."pid"
  depl:loadComponent(pid_name, "lcsr_controllers::JointPIDController");
  pid = depl:getPeer(pid_name)
  connect(wam, "position_out", pid, "joint_position_in");
  connect(wam, "velocity_out", pid, "joint_velocity_in");
  connect(                     pid, "joint_effort_out", effort_sum, "feedback_in");

  --[[ Create joint-space RML trajectory generator --]]
  traj_rml_name = prefix.."traj_rml"
  depl:loadComponent(traj_rml_name,"lcsr_controllers::JointTrajGeneratorRML");
  traj_rml = depl:getPeer(traj_rml_name)
  connect(wam, "position_out", traj_rml, "joint_position_in");
  connect(wam, "velocity_out", traj_rml, "joint_velocity_in");
  connect(                     traj_rml, "joint_position_out", pid, "joint_position_cmd_in");
  connect(                     traj_rml, "joint_velocity_out", pid, "joint_velocity_cmd_in");
  --connect(                     traj_rml, "joint_acceleration_out", pid, "joint_acceleration_cmd_in");

  --[[ Create a cartesian impedance controller (jacobian transpose) --]]
  jtns_name = prefix.."jtns"
  depl:loadComponent(jtns_name,"lcsr_controllers::JTNullspaceController");
  jtns = depl:getPeer(jtns_name)
  connect(wam, "position_out", jtns, "joint_position_in");
  connect(wam, "velocity_out", jtns, "joint_velocity_in");
  connect(                     jtns, "joint_effort_out", effort_sum, "feedback_in");
  jtns:connectPeers(tf)
  jtns:connectServices(tf)

  --[[ Create an IK controller --]]
  ik_name = prefix.."ik"
  depl:loadComponent(ik_name, "lcsr_controllers::IKController");
  ik = depl:getPeer(ik_name)
  connect(wam, "position_out", ik, "positions_in");
  connect(                     ik, "trajectories_out", traj_rml, "joint_traj_cmd_in");
  ik:connectPeers(tf)

  --[[ Create a cartesian interpolator --]]
  cart_servo_name = prefix.."cart_servo"
  depl:loadComponent(cart_servo_name,"lcsr_controllers::CartesianLogisticServo");
  cart_servo = depl:getPeer(cart_servo_name)
  connect(wam, "position_out", cart_servo, "positions_in");
  connect(                     cart_servo, "framevel_out", jtns, "framevel_in");
  cart_servo:connectPeers(tf)

  --[[ Create a coulomb friction compensator --]]
  coulomb_name = prefix.."coulomb"
  depl:loadComponent(coulomb_name, "lcsr_controllers::CoulombCompensator");
  coulomb = depl:getPeer(coulomb_name)
  connect(wam, "position_out", coulomb, "joint_position_in");
  connect(jtns, "wrench_out",  coulomb, "wrench_des_in");
  connect(                     coulomb, "joint_effort_out", effort_sum, "feedforward_in");

  --[[ Create a setpoint controller for tuning PID --]]
  joint_setpoint_name = prefix.."joint_setpoint"
  depl:loadComponent(joint_setpoint_name, "lcsr_controllers::JointSetpoint");
  joint_setpoint = depl:getPeer(joint_setpoint_name)
  connect(wam, "position_out", joint_setpoint, "joint_position_in");
  connect(wam, "velocity_out", joint_setpoint, "joint_velocity_in");
  connect(                     joint_setpoint, "joint_position_out", pid, "joint_position_cmd_in");
  connect(                     joint_setpoint, "joint_velocity_out", pid, "joint_velocity_cmd_in");

  --[[ Create a setpoint controller for testing traj generation --]]
  traj_setpoint_name = prefix.."traj_setpoint"
  loadComponent("traj_setpoint","lcsr_controllers::JointSetpoint");
  traj_setpoint = depl:getPeer(traj_setpoint_name)
  connect(wam, "position_out", traj_setpoint, "joint_position_in");
  connect(wam, "velocity_out", traj_setpoint, "joint_velocity_in");
  connect(                     traj_setpoint, "joint_position_out", traj_rml, "joint_position_cmd_in");

  --[[ Configure all components --]]
  pid:configure();
  traj_rml:configure();
  jtns:configure();
  ik:configure();
  cart_servo:configure();
  coulomb:configure();
  joint_setpoint:configure();
  traj_setpoint:configure();

  --[[ Add to the conman scheme --]]
  scheme:addPeer(pid);
  scheme:addPeer(traj_rml);
  scheme:addPeer(jtns);
  scheme:addPeer(ik);
  scheme:addPeer(cart_servo);
  scheme:addPeer(coulomb);
  scheme:addPeer(joint_setpoint);
  scheme:addPeer(traj_setpoint);

  --[[ Add blocks to the scheme --]]
  scheme:addBlock(pid_name);
  scheme:addBlock(traj_rml_name);
  scheme:addBlock(jtns_name);
  scheme:addBlock(ik_name);
  scheme:addBlock(cart_servo_name);
  scheme:addBlock(coulomb_name);
  scheme:addBlock(joint_setpoint_name);
  scheme:addBlock(traj_setpoint_name);

  --[[ Create joint control group --]]
  joint_control = prefix.."joint_control"
  scheme:addGroup(joint_control);
  scheme:addToGroup(pid_name, joint_control);
  scheme:addToGroup(traj_rml_name, joint_control);

  --[[ Create cart impedance control group --]]
  cart_imp_control = prefix.."cart_imp_control"
  scheme:addGroup(cart_imp_control);
  scheme:addToGroup(jtns_name, cart_imp_control);
  scheme:addToGroup(cart_servo_name, cart_imp_control);

  --[[ Create an IK control group --]]
  ik_control = prefix.."ik_control"
  scheme:addGroup(ik_control);
  scheme:addToGroup(ik_name, ik_control);
  scheme:addToGroup(traj_rml_name, ik_control);
  scheme:addToGroup(pid_name, ik_control);
end

