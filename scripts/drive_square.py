#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class DriveSquare(object):
    def __init__(self):
        rospy.init_node('drive_square')
        rospy.sleep(2) # sleep for 2 seconds to have the system inited
        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    
    def move_forward(self, duration, f_speed):
        forward_cmd = Twist()
        forward_cmd.linear.x = f_speed
        forward_cmd.linear.y = 0.0
        forward_cmd.angular.z = 0.0


        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1) 

        start_time = rospy.Time.now()

        # rospy.Rate specifies the rate of the loop (in this case 2 Hz)
        r = rospy.Rate(1)
        
        while rospy.Time.now() - start_time < rospy.Duration(duration) and not rospy.is_shutdown():
            self.speed_pub.publish(forward_cmd)
            r.sleep()
        
        # stop the robot
        self.speed_pub.publish(Twist())
        rospy.sleep(2)
        print("forward")
    
    def turn_to(self, duration, a_speed):
        turn_cmd = Twist()
        turn_cmd.angular.z = a_speed

        rospy.sleep(1)

        start_time = rospy.Time.now()

        r = rospy.Rate(1)
        while rospy.Time.now() - start_time < rospy.Duration(duration) and not rospy.is_shutdown():
            self.speed_pub.publish(turn_cmd)
            r.sleep()
        
        self.speed_pub.publish(Twist())
        rospy.sleep(2)
        print("turn")

if __name__ == "__main__":
    try:
        node = DriveSquare()
        for _ in range(4):
            node.move_forward(2, 0.5)
            node.turn_to(3, 0.5)
        # node.move_forward(2)
        # node.move_forward(3)
    except rospy.ROSInterruptException:
        pass
