#!/usr/bin/env python

from __future__ import print_function

import time

import rospy

def main():
    rospy.init_node('time_warp_monitor')

    while not rospy.is_shutdown():
        last_sim = None
        last_wall = None

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            sim = rospy.Time.now()
            wall = rospy.Time.from_sec(time.time())

            if last_sim and sim < last_sim:
                last_sim = sim

            if not last_sim or (sim - last_sim).to_sec() > 1.0:
                if last_sim:
                    print("%g%% realtime" % (100.0 * (sim-last_sim).to_sec() / (wall-last_wall).to_sec()))
                last_wall = wall
                last_sim = sim

            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException as exc:
                break

if __name__ == '__main__':
    main()
