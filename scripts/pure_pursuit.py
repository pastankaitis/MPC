#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

import numpy as np
import math
import sys
import os
import tf

from template_model import template_model
from template_mpc import template_mpc


def pose_callback(pose_msg):
    quaternion = np.array([pose_msg.pose.pose.orientation.x,
                           pose_msg.pose.pose.orientation.y,
                           pose_msg.pose.pose.orientation.z,
                           pose_msg.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]

    quaternion_z = pose_msg.pose.pose.orientation.z
    quaternion_w = pose_msg.pose.pose.orientation.w
    head_angle = math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))

    set_drive(position[0], position[1], head_angle,  2.0, 0.5)
    #set_drive(0.0, 0.0, )

   

# Pass relevant information to publisher
def set_drive(posx, posy, head_angle, tarx, tary):

    drive_publish = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    model = template_model()
    mpc = template_mpc(model, tarx, tary)
    x0 = np.array([posx, posy, head_angle]).reshape(-1,1)
    mpc.x0 = x0
    mpc.set_initial_guess()
    u0 = mpc.make_step(x0)
    print(u0[0], u0[1])
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.header.frame_id = "drive"
    drive_msg.drive.steering_angle = u0[1]
    drive_msg.drive.speed = u0[0]
    drive_publish.publish(drive_msg)




if __name__ == '__main__':
    rospy.init_node('ftg')
    rospy.Subscriber('/odom', Odometry, pose_callback, queue_size=1)
    rospy.spin()
    
    
    
    
    
    
