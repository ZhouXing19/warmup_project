#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import inf


class WallFollower(object):
    def __init__(self, goal_dist = 0.5, prop = 0.15):
        '''
        Initialize the instance
        '''
        rospy.init_node("wall_follower")
        rospy.sleep(2) # sleep for 2 seconds to have the system inited
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_dist = goal_dist # The closest distance we would like to set between the turtle and the wall
        self.prop = prop # For proportional control of the speed

    def get_cur_distance(self, scan):
        '''
        Get the distance in 5 directions: front, left, right, front right, and back right.
        Consider 5 direction ranges:
            front: 0
            left: 85,95
            right: 265,275
            front_right: 310,320
            back_right: 220,230
        It turns out we only need to use the front.
        '''
        dists = [0 for _ in range(5)]
        dist_ranges = [(0, 10), (85, 95), (265, 275),(310, 320), (220, 230)]

        for idx in range(5): # loop over the five directions
            range_l, range_h = dist_ranges[idx]
            range_diff = range_h - range_l
            # For each range, each scan will be listed and averaged.
            dists[idx] = sum(scan.ranges[range_l:range_h])/range_diff 
        return dists

    def set_vel(self, dists):
        front, left, right, fright, bright = dists
        
        print("=============")
        print(f"front: {front}")
        print(f"left: {left}")
        print(f"right: {right}")
        print(f"front_right: {fright}")
        print(f"back_right: {bright}")
        new_vel = Twist()
        
        # ====== My draft that tried to work with more complicated settings 

        # if front < self.goal_dist and self.turtle_status == 0:
        #     new_vel.linear.x = 0.0
        #     new_vel.angular.z = 2.5  # + => turn left / - => turn right
        #     self.turtle_status = 1
        #     print("front wall! turn left!")
        
        # elif right < self.goal_dist and front < self.goal_dist and self.turtle_status == 1:
        #     new_vel.linear.x = 0.0
        #     new_vel.angular.z = 2.5  # + => turn left / - => turn right
        #     print("There's something in front of me! turn left!")
        # elif right < self.goal_dist and front > self.goal_dist and self.turtle_status == 1:
        #     new_vel.linear.x = 2.0
        #     new_vel.angular.z = 0.0  # + => turn left / - => turn right
            
        #     print("Follow the wall!")
        # elif right > self.goal_dist + 0.1 and front > self.goal_dist + 0.1 and self.turtle_status == 1 and fright > self.goal_dist + 0.3:
        #     new_vel.linear.x = 2.0
        #     new_vel.angular.z = 0  # + => turn left / - => turn right
            
        #     print("No wall! turn turn right!")
        
        # else: # Otherwise, going straight
        #     new_vel.linear.x = 2.0
        #     new_vel.angular.z = 0.0
#=========
        if front >= self.goal_dist:
            # If it's still far, just rush ahead
            print("====rush rush rush=====")
            new_vel.linear.x = self.prop * front
        else:
            print("=====turn turn turn =====")
            # Otherwise, turn left, since the wall is a square
            new_vel.angular.z = 1
            new_vel.linear.x = 0.1

        self.speed_pub.publish(new_vel)

    def scan_callback(self, data):
        self.set_vel(self.get_cur_distance(data))

    def run(self):
        rospy.spin()
    
if __name__ == "__main__":
    try:
        node = WallFollower()
        node.run()
    except rospy.ROSInterruptException:
        pass