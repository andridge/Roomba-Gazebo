#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FollowWall:
   
    def __init__(self):
        self.active = False
        self.state = 0
        self.pub_vel = rospy.Publisher('/your_robot/cmd_vel', Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber('/your_robot/laser/scan', LaserScan, self.callback_laser)
   
        rospy.spin()

    def callback_laser(self, msg):
        if not self.active:
            return

        if self.state == 0:
            self.find_wall(msg)
        elif self.state == 1:
            self.turn_left()
        elif self.state == 2:
            self.follow_the_wall(msg)
        else:
            rospy.logerr('Unknown state!')

    def find_wall(self, msg):
        min_dist = min(msg.ranges)
        if min_dist < 1.0:
            self.state = 1

    def turn_left(self):
        vel_msg = Twist()
        vel_msg.angular.z = 0.5  # Adjust the angular velocity as needed
        self.pub_vel.publish(vel_msg)

        rospy.sleep(1.0)  # Adjust the duration of turning as needed

        self.state = 2

    def follow_the_wall(self, msg):
        # Implement wall following logic here
        pass

if __name__ == '__main__':
    rospy.init_node('follow_wall')
    FollowWall()
