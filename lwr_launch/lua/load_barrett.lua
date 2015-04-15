
function load_barrett(depl, scheme, prefix, sim, urdf_param)

  --[ default arguments ]--
  local prefix = prefix or ""
  local sim = sim or false
  local urdf_param = urdf_param or "/robot_description"

  --[ required imports ]--
  depl:import("rtt_ros")
  gs:provides("ros"):import("rtt_rosparam")
  gs:provides("ros"):import("conman_blocks")
  gs:provides("ros"):import("lcsr_controllers")
  
  --[ convenience vars ]--
  cp = rtt.Variable("ConnPolicy")

  --[ sim or hardware barrett_manager component ]--
  if sim then
    rtt.logl("Info","Creating simulated barrett manager...")
    gs:provides("ros"):import("oro_barrett_sim")
    manager_type = "oro_barrett_sim::BarrettSimManager"
  else
    rtt.logl("Info", "Creating real barrett manager...")
    gs:provides("ros"):import("oro_barrett_hw")
    manager_type = "oro_barrett_hw::BarrettHWManager"
  end

  --[[ load and configure barrett manager --]]
  manager_name = prefix.."barrett_manager"

  --[[ check if the manager already exists --]]
  manager_exists = false
  for key,peer_name in pairs(depl:getPeers()) do
    if peer_name == manager_name then
      manager_exists = true
    end
  end

  if not manager_exists then
    loaded = depl:loadComponent(manager_name, manager_type)
    if not loaded then
      rtt.logl("Error", "Could not load manager: "..manager_type)
      return nil, nil
    end
  end

  manager = depl:getPeer(manager_name)
  manager:loadService("rosparam")
  manager:provides("rosparam"):getAll()
  manager:provides("rosparam"):getParam(urdf_param,"robot_description")
  if not manager:configure() then
    return nil, nil
  end

  --[[ configure effort sum --]]
  effort_sum_name = prefix.."effort_sum"
  depl:loadComponent(effort_sum_name, "conman_blocks::FeedForwardFeedBack")
  depl:connect( effort_sum_name..".sum_out", manager_name..".wam.effort_in", cp)
  effort_sum = depl:getPeer(effort_sum_name)
  effort_sum:configure()
  
  --[[ Create Feed-Forward inverse dynamics component --]]
  inverse_dynamics_name = prefix.."inverse_dynamics"
  depl:loadComponent(inverse_dynamics_name, "lcsr_controllers::IDControllerKDL")
  inverse_dynamics = depl:getPeer(inverse_dynamics_name)
  depl:connect(manager_name..".wam.position_out", inverse_dynamics_name..".joint_position_in", cp)
  depl:connect(manager_name..".wam.velocity_out", inverse_dynamics_name..".joint_velocity_in", cp)
  depl:connect(                                   inverse_dynamics_name..".joint_effort_out", effort_sum_name..".feedforward_in", cp)

  --[[ Connect hand masses to inverse dynamics if necessary --]]
  if manager:getProperty("auto_configure_hand"):get() then
    depl:connect(manager_name..".hand.center_of_mass_out", inverse_dynamics_name..".end_effector_masses_in", cp)
  end

  inverse_dynamics:configure()

  --[[ create "devices" group --]]
  devices = prefix.."devices"
  scheme:addGroup(devices)

  --[[ peer components to scheme  --]]
  --[[ NOTE: this could be skipped if an "add" operation which accepted a
  taskcontext existed, but the conman interface is designed for ops scripts. In
  the future maybe we could have two interfaces exposed. --]]
  scheme:addPeer(manager)
  scheme:addPeer(effort_sum)
  scheme:addPeer(inverse_dynamics)

  --[[ add maanger and configure latching --]]
  scheme:addToGroup(manager_name, devices)
  scheme:latchInputs(manager_name, true)

  scheme:addToGroup(effort_sum_name, devices)
  scheme:addToGroup(inverse_dynamics_name, devices)

  return manager, effort_sum
end
