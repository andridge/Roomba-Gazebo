#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from GoToPoint import GoToPoint
from FollowWall import FollowWall
from com760_Suhael_Ahmed.srv import b00856266HomingSignal


def is_on_M_line(x_current, y_current, x0, y0, threshold=0.05):
    goal_point_X = 1.3
    goal_point_Y = 6.15
   
    if x0 - goal_point_X != 0:
        dist = abs(y_current - ((x_current - x0) * (y0 - goal_point_Y) / (x0 - goal_point_X) + y0))
        return dist < 1
    else:
        dist = abs(x_current - goal_point_X)
        return dist < threshold

def homing_signal_client():
    rospy.wait_for_service('b00856266_homing_signal')
    try:
        homing_signal = rospy.ServiceProxy('b00856266_homing_signal', b00856266HomingSignal)

        response = homing_signal()
        return response.flag, response.des_x, response.des_y, response.message
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: " + str(e))
        return False, 0.0, 0.0, ""

if __name__ == "__main__":
    rospy.init_node('bug0_algorithm')

    # Initialize the robot and laser scanner
    robot = init_robot(time_step=32)
    laser = robot.getLaser("laser")
    laser.enable(TIME_STEP)
   
    goal_position = [1.3, 6.15]
    start_position = [1.3, -9.74]
    visited_points = []

    go_to_point = GoToPoint(goal_position)
    follow_wall = FollowWall()

    state = 'start'

    while robot.step(32) != -1:
        # Read laser sensor values
        ranges = laser.getRangeImage()

        # Update robot state
        update_robot_state()

        # State machine
        if state == 'start':
            flag, start_x, start_y, message = homing_signal_client()
            if flag:
                start_position = [start_x, start_y]
                if is_on_M_line(gps_values[0], gps_values[1], start_position[0], start_position[1]):
                    state = 'align_robot_heading'
                    go_to_point.reset()
                    go_to_point.set_target(goal_position)
            else:
                rospy.loginfo("Service disabled. Message: " + message)

        elif state == 'align_robot_heading':
            if go_to_point.align_to_goal_heading(get_bearing_in_degrees(compass_val)):
                state = 'move_to_goal'

        elif state == 'move_to_goal':
            go_to_point.move_to_goal()

            if calculate_euclidean_distance(gps_values[0], gps_values[1], goal_position[0], goal_position[1]) < 0.5:
                state = 'follow_wall'

        elif state == 'follow_wall':
            follow_wall.follow_wall()

            if calculate_euclidean_distance(gps_values[0], gps_values[1], goal_position[0], goal_position[1]) < 0.5:
                state = 'end'

        elif state == 'end':
            if go_to_point.align_to_goal_heading(get_bearing_in_degrees(compass_val), threshold=5):
                update_motor_speed(input_omega=[0, 0, 0])  # Stop the robot
                rospy.loginfo("Robot reached the docking station.")
                break

    pass
