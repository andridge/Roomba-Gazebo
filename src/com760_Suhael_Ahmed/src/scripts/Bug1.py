#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from GoToPoint import GoToPoint
from FollowWall import FollowWall
from com760_Suhael_Ahmed.srv import b00856266HomingSignal
import numpy as np
from initialization import *
import math

def is_on_M_line(x_current, y_current, x0, y0, threshold=0.05):
    goal_point_X = 1.3
    goal_point_Y = 6.15

    if x0 - goal_point_X != 0:
        dist = abs(y_current - ((x_current - x0) * (y0 - goal_point_Y) / (x0 - goal_point_X) + y0))
        return dist < 1
    else:
        dist = abs(x_current - goal_point_X)
        return dist < threshold


def align_to_M(heading, theta, turn_left=False):
    threshold = 5
    ts = 3  # turning speed
    if turn_left:
        ts *= -1  # turn left
    if abs(heading - theta % 360) < threshold:
        return True
    else:
        update_motor_speed(input_omega=[ts, ts, ts])
        return False


def calculate_euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def calculate_theta(x1, y1, x0, y0):
    if x1 == x0:
        return 180
    else:
        return (math.degrees(math.atan((y1 - y0) / (x1 - x0))) % 360) - 90


if __name__ == "__main__":

    rospy.init_node('bug1_algorithm')

    robot = init_robot(time_step=32)
    laser = robot.getLaser("laser")
    laser.enable(32)

    prev = ""
    goal_postition = [1.3, 6.15]
    start_position = [1.3, -9.74]
    state = 'start'
    robot_speed = 8
    forward_left_speeds = [-1 * robot_speed, -1 * robot_speed, 2 * robot_speed]
    far_from_wall_counter = 0
    close_to_wall_counter = 0
    hit_point = []  # x, y
    closest_point = None  # x, y
    x0, y0 = start_position
    is_finding_closest_point = False

    while robot.step(32) != -1:
        ranges = laser.getRangeImage()

        gps_values = robot_position
        compass_val = robot_omega[2]

        front_left_distance = ranges[270]
        front_right_distance = ranges[90]

        update_robot_state()

        # DEFINE STATE MACHINE HERE!

        if state == 'start':
            distance_threshold = 0.0001
            if is_on_M_line(gps_values[0], gps_values[1], x0, y0):
                prev = state
                state = 'align_robot_heading'

        elif state == 'align_robot_heading':
            theta = calculate_theta(goal_postition[0], goal_postition[1], x0, y0)
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=theta)
            if is_aligned:
                prev = state
                state = 'move_to_goal'

        elif state == 'move_to_goal':
            update_motor_speed(input_omega=[-1 * robot_speed, robot_speed, 0])
            if front_left_distance < 1.0 or front_right_distance < 1.0:
                prev = state
                state = 'wall_following'
                hit_point.append([gps_values[0], gps_values[1]])
                closest_point = [gps_values[0], gps_values[1]]
                update_motor_speed(input_omega=[-1 * robot_speed, -1 * robot_speed, 2 * robot_speed])

            elif calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0],
                                              goal_postition[1]) < 0.5:
                state = 'end'

        elif state == 'wall_following':
            if not is_finding_closest_point and calculate_euclidean_distance(gps_values[0], gps_values[1],
                                                                             goal_postition[0],
                                                                             goal_postition[1]) < calculate_euclidean_distance(
                    closest_point[0], closest_point[1], goal_postition[0], goal_postition[1]):
                closest_point = gps_values[0], gps_values[1]

            difference = front_left_distance - front_right_distance

            if difference > 0.1:
                update_motor_speed(input_omega=[robot_speed, robot_speed, 0])
            elif difference < -0.1:
                update_motor_speed(input_omega=[-1 * robot_speed, -1 * robot_speed, 0])
            elif max(front_left_distance, front_right_distance) > 0.8:
                far_from_wall_counter += 1
                if far_from_wall_counter == 10:
                    prev = state
                    state = 'go_close'
                    far_from_wall_counter = 0
                else:
                    update_motor_speed(input_omega=[-1 * robot_speed, -1 * robot_speed, 2 * robot_speed])
                    forward_left_speeds = [-1 * robot_speed, -1 * robot_speed, 2 * robot_speed]

            else:
                update_motor_speed(input_omega=[-1 * robot_speed, -1 * robot_speed, 2 * robot_speed])
                forward_left_speeds = [-1 * robot_speed, -1 * robot_speed, 2 * robot_speed]

            if front_left_distance < 0.3 or front_right_distance < 0.3:
                close_to_wall_counter += 1
                if close_to_wall_counter >= 20:
                    prev = state
                    state = 'go_reverse'

            if is_finding_closest_point:
                x0, y0 = closest_point
                dist = calculate_euclidean_distance(gps_values[0], gps_values[1], x0, y0)
                if dist < 0.2:
                    if is_on_M_line(gps_values[0], gps_values[1], x0, y0):
                        state = 'align_robot_heading'
                        hit_point = []
                        closest_point = []
                        is_finding_closest_point = False

        elif state == 'go_reverse':
            update_motor_speed(input_omega=[robot_speed, -1 * robot_speed, 0])
            if front_left_distance > 0.8 or front_right_distance > 0.8:
                prev = state
                state = 'wall_following'
                close_to_wall_counter = 0

        elif state == 'go_close':
            update_motor_speed(input_omega=[-1 * robot_speed, robot_speed, 0])
            if front_left_distance < 1.0 or front_right_distance < 1.0:
                prev = state
                state = 'wall_following'

        elif state == 'end':
            is_aligned = align_to_M(get_bearing_in_degrees(compass_val), theta=90)
            if is_aligned:
                update_motor_speed(input_omega=[0, 0, 0])  # end

        elif calculate_euclidean_distance(gps_values[0], gps_values[1], goal_postition[0], goal_postition[1]) < 0.5:
            state = 'end'
            update_motor_speed(input_omega=[0, 0, 0])

    pass
