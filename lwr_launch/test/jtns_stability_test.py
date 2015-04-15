#!/usr/bin/env python

import math

import rospy

import actionlib
import control_msgs.msg
import controller_manager_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg

def main():
    rospy.init_node('test_jtns_stability')

    # Connect to the controller switching service
    rospy.loginfo("Connecting to controller manager...")
    rospy.wait_for_service('controller_manager/switch_controller')
    switch_controllers = rospy.ServiceProxy(
            'controller_manager/switch_controller',
            controller_manager_msgs.srv.SwitchController)
    rospy.loginfo("Connected to controller manager.")

    # Enable joint control group
    switch_controllers(['joint_control',],['ik_control','cart_imp_control'],1)

    # Connect to the trajectory action client
    traj_client = actionlib.SimpleActionClient(
            '/gazebo/traj_rml/action', control_msgs.msg.FollowJointTrajectoryAction)
    traj_client.wait_for_server()
    rospy.loginfo("Connected to traj action.")

    # Crate a traj goal
    traj_goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj_goal.trajectory.header.stamp = rospy.Time(0,0)
    
    # Create the first test point
    point_pi4 = trajectory_msgs.msg.JointTrajectoryPoint()
    point_pi4.positions = [
            0,
            -math.pi/4.0,
            0,
            math.pi/4.0,
            math.pi/4.0,
            0,
            math.pi/4.0]

    traj_goal.trajectory.points.append(point_pi4)

    traj_client.send_goal(traj_goal)
    traj_client.wait_for_result()

    print(str(traj_client.get_result()))

    #switch_controllers(['cart_imp_control',],['traj_control',],1)

if __name__ == '__main__':
    main()
