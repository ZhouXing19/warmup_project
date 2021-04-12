#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from math import inf
from math import pi

class PersonFollower(object):
    def __init__(self, goal_dist = 0.5, prop = 0.15):
        '''
        Initialize the instance
        '''
        rospy.init_node("person_follower")
        rospy.sleep(2) # sleep for 2 seconds to have the system inited
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_dist = goal_dist # The closest distance we would like to set between the turtle and the wall
        self.prop = prop # For proportional control of the speed

    def get_closest_ang_dist(self, scan):
        min_dist = float("inf")
        min_idx = -1
        for idx, dist in enumerate(scan.ranges):
            if dist < min_dist:
                min_idx = idx
                min_dist = dist
        return (min_idx, min_dist)
    
    def __idx_2_radiant(self, idx):
        '''
        The degree range: (-180 ~ 180)
        '''
        degree = idx if idx < 180 else idx - 360
        return pi * degree / 180
    
    def set_vel(self, min_idx, min_dist):
        new_vel = Twist()
        if min_dist == float("inf"):
            print("=====I can't see it!=====")
            pass
        elif min_dist < self.goal_dist:
            print("=====I got you.=====")
            pass
        else:
            print("=====Rushing Rushing=====")
            new_vel.linear.x = self.prop * min_dist
            new_ang = self.__idx_2_radiant(min_idx)
            new_vel.angular.z = new_ang
        
        self.speed_pub.publish(new_vel)


    def scan_callback(self, data):
        min_idx, min_dist = self.get_closest_ang_dist(data)
        self.set_vel(min_idx, min_dist)


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = PersonFollower()
        node.run()
    except rospy.ROSInterruptException:
        pass