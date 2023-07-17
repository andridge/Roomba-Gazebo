#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math

class GoToPoint:

    def __init__(self):
        self.active = False
        self.position = Point()
        self.yaw = 0
        self.state = 0
        self.desired_position = Point(-4, -4, 0)  # Docking station position
        self.yaw_threshold = math.radians(rospy.get_param('th_yaw'))  # Yaw threshold in radians
        self.dist_threshold = rospy.get_param('th_dist')  # Distance threshold

        self.pub_vel = rospy.Publisher('/your_robot/cmd_vel', Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber('/your_robot/odom', Odometry, self.callback_odom)

        rospy.spin()

    def callback_odom(self, msg):
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]  # Yaw angle

        if not self.active:
            return

        if self.state == 0:
            self.fix_heading(self.desired_position)
        elif self.state == 1:
            self.go_straight(self.desired_position)
        elif self.state == 2:
            self.done()
        else:
            rospy.logerr('Unknown state!')

    def fix_heading(self, desired_pos):
        target_angle = math.atan2(desired_pos.y - self.position.y, desired_pos.x - self.position.x)
        angle_diff = target_angle - self.yaw

        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        angular_vel = 0.5 * angle_diff

        vel_msg = Twist()
        vel_msg.angular.z = angular_vel
        self.pub_vel.publish(vel_msg)

        if abs(angle_diff) < self.yaw_threshold:
            self.state = 1

    def go_straight(self, desired_pos):
        distance = math.sqrt((desired_pos.x - self.position.x) ** 2 + (desired_pos.y - self.position.y) ** 2)

        linear_vel = 0.2 * distance

        vel_msg = Twist()
        vel_msg.linear.x = linear_vel
        self.pub_vel.publish(vel_msg)

        if distance < self.dist_threshold:
            self.state = 2

    def done(self):
        vel_msg = Twist()
        self.pub_vel.publish(vel_msg)
        rospy.loginfo("Robot reached the docking station.")

if __name__ == '__main__':
    rospy.init_node('go_to_point')
    GoToPoint()
