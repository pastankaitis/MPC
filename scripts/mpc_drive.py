#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import sys
import os
#import tf


import math
import numpy as np
import scipy as sp


import pdb
import sys
sys.path.append('../../')
import do_mpc
from do_mpc.tools.timer import Timer


from template_model import template_model
from template_mpc import template_mpc
from solving import find_constraint

from constants import LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX
from constants import FTG_IGNORE_RANGE, SAFETY_RADIUS, POSITION_PREDICTION_TIME
from constants import LIDAR_MINIMUM_ANGLE, LIDAR_ANGLE_INCREMENT, LIDAR_MAX_INDEX
from point import LidarPoint, CartesianPoint



lidar = None



def find_sequence(points, ignore_range):

    current_left_index = 0
    current_right_index = 0
    current_longest_sequence = 0
    final_left_index = 0
    final_right_index = 0
    is_sequence_started = False

    for i, point in enumerate(points):

            # Lengthen the sub-sequence or start a new one if non-ignore number found
        if point.range != ignore_range:
            if is_sequence_started:
                current_right_index += 1
            else:
                is_sequence_started = True
                current_left_index = i
                current_right_index = i

            # End current sub-sequence and see if it was any bigger than the previous one
        else:
            if is_sequence_started:
                length = current_right_index - current_left_index
                if length > current_longest_sequence:
                    final_left_index = current_left_index
                    final_right_index = current_right_index
                    current_longest_sequence = length
                is_sequence_started = False

       # Finally, make sure that if the last point was also within the sub-sequence, the sub-sequence is also counted
    if is_sequence_started:
        length = current_right_index - current_left_index
        if length > current_longest_sequence:
            final_left_index = current_left_index
            final_right_index = current_right_index

    return points[final_left_index:final_right_index]

def get_target(points):

    if not points:
        return CartesianPoint(mpc.target_x, mpc.target_y)

    return points[len(points)//2].cartesian



def lidar_to_cart(ranges, position_x, position_y, heading_angle, starting_index):

    points = []

    for index, lidar_range in enumerate(ranges):
        laser_beam_angle = ((starting_index + index) * LIDAR_ANGLE_INCREMENT) + LIDAR_MINIMUM_ANGLE
        rotated_angle = laser_beam_angle + heading_angle
        x_coordinate = lidar_range * math.cos(rotated_angle) + position_x
        y_coordinate = lidar_range * math.sin(rotated_angle) + position_y
        points.append(CartesianPoint(x_coordinate, y_coordinate))


    return points

def adjust_target_position(position_x, position_y, heading_angle):
   

    global lidar 

    lidar_data = lidar ##

    ranges = lidar_data.ranges[RIGHT_DIVERGENCE_INDEX:(LEFT_DIVERGENCE_INDEX+1)]

    cartesian_points = lidar_to_cart(
            ranges=ranges,
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle,
            starting_index=RIGHT_DIVERGENCE_INDEX
        )

        # Build a list of relevant Point instances
    points = list()
    for i in range(LEFT_DIVERGENCE_INDEX - RIGHT_DIVERGENCE_INDEX + 1):
        cartesian_point = cartesian_points[i]
        lidar_index = i + RIGHT_DIVERGENCE_INDEX
        lidar_range = ranges[i]

        if lidar_range <= SAFETY_RADIUS:
            lidar_range = FTG_IGNORE_RANGE

        points.append(LidarPoint(lidar_index, lidar_range, cartesian_point))

    tarx, tary = get_target(find_sequence(points, FTG_IGNORE_RANGE))

    return tarx, tary




def odometry_update(data):

    global lidar
    
    lidar = data

def pose_callback(pose_msg):
 
    global lidar

    quaternion = np.array([pose_msg.pose.pose.orientation.x,
                           pose_msg.pose.pose.orientation.y,
                           pose_msg.pose.pose.orientation.z,
                           pose_msg.pose.pose.orientation.w])

    position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z]

    #euler = tf.transformations.euler_from_quaternion(quaternion)

    quaternion_z = pose_msg.pose.pose.orientation.z
    quaternion_w = pose_msg.pose.pose.orientation.w

    head_angle = math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))

    tarx = adjust_target_position(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, head_angle)[0]
    tary = adjust_target_position(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, head_angle)[1]
  

    mpc_drive(position[0], position[1], head_angle, tarx, tary)
    

   

# Pass relevant information to publisher
def mpc_drive(posx, posy, head_angle, tarx, tary):


    drive_publish = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    model = template_model()
    mpc = template_mpc(model, tarx, tary)
    x0 = np.array([posx, posy, head_angle]).reshape(-1, 1)
    mpc.x0 = x0


    mpc.set_initial_guess()

    u0 = mpc.make_step(x0)

    


    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.header.frame_id = "drive"
    drive_msg.drive.steering_angle = float(u0[1])
    drive_msg.drive.speed = 5*float(u0[0])
    drive_publish.publish(drive_msg)




if __name__ == '__main__':
    rospy.init_node('mpc_node')
    rospy.Subscriber('/scan', LaserScan, odometry_update, queue_size=1)
    rospy.Subscriber('/odom', Odometry, pose_callback, queue_size=1)
    rospy.spin()
    
    
    
    
    
    
