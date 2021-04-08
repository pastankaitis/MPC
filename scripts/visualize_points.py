#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

import os
import csv
import pdb
import numpy as np

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

rospy.init_node('view_node')

while not rospy.is_shutdown():
    markerArray = MarkerArray()
    x = 2.0
    y = 0.5
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.11
    marker.scale.y = 0.11
    marker.scale.z = 0.11
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.7
    marker.color.b = 0.8
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.id = 1
    markerArray.markers.append(marker)

    publisher.publish(markerArray)

    rospy.sleep(0.3)
