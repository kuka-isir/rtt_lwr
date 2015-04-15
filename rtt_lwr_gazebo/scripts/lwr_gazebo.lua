--[[ conventional globals --]]
gs = rtt.provides()
tc = rtt.getTC()
depl = tc;--:getPeer("Deployer")

--rtt.log("Loading lwr_gazebo !")
--[[ get ROS global service --]]
--depl:import("rtt_ros")
--ros = gs:provides("ros")
--ros:import("rtt_lwr_sim")
--require("rtt_lwr_sim")



depl:setActivity("lwr_gazebo",0.001,depl:getAttribute("HighestPriority"):get(),rtt.globals.ORO_SCHED_OTHER)

lwr_gazebo = depl:getPeer("lwr_gazebo")

if(lwr_gazebo) then
	lwr_gazebo:configure()
	lwr_gazebo:start()
else
	rtt.log("gazebo not found")
end