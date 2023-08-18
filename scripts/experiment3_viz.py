#!/usr/bin/env python3

import rospy
import math
import numpy as np
import traceback
import copy
from skimage.util import invert
from skimage.transform import rescale
from master_project.srv import RequestPath,RequestPathRequest,RequestPathResponse
# from tf2_ros import Buffer,TransformListener
import tf2_ros
# import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from move_base_msgs.msg import MoveBaseAction,MoveBaseFeedback,MoveBaseGoal,MoveBaseResult,MoveBaseActionGoal
from mbf_msgs.msg import ExePathAction,ExePathGoal,ExePathFeedback,ExePathResult

from geometry_msgs.msg import Pose,Point,PoseStamped,TransformStamped

from actionlib import SimpleActionClient

from sensor_msgs.msg import LaserScan

from csv import writer
from rospy_message_converter import message_converter
import json

from std_msgs.msg import Float32

class Ex3ExePath:
    def __init__(self):


        self.min_range_pub = rospy.Publisher("/min_range",Path,queue_size=10)
        self.min_value_pub = rospy.Publisher("/min_value",Float32,queue_size=10)
        # 1. subscribe to lidar
        self.lidar_sub = rospy.Subscriber('/scan_multi',LaserScan,self.scan_cb)
        

        rospy.loginfo("Experiment3 vizualization is running")

    def scan_cb(self,msg:LaserScan):
        # find index of min range
        min_value = min(msg.ranges)
        min_index = msg.ranges.index(min_value)
        th = (min_index * msg.angle_increment) + msg.angle_min
        print(th)
        min_range_msg = Path()
        x = min_value * math.cos(th)
        y = min_value * math.sin(th)

        min_range_msg.header.frame_id = "base_footprint"
        min_range_msg.header.stamp = rospy.Time.now()
        min_range_msg.poses.append(PoseStamped())
        temp = PoseStamped()
        temp.pose.position.x = x
        temp.pose.position.y = y
        min_range_msg.poses.append(temp)

        self.min_range_pub.publish(min_range_msg)
        self.min_value_pub.publish(data=min_value)
        




if __name__ == '__main__':
    rospy.init_node('experiment3')
    server = Ex3ExePath()
    rospy.spin()