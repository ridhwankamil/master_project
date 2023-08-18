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

class Ex3ExePath:
    def __init__(self):
        # self.path_planner_ser_name = '/gng/path_planner'
        # self.path_planner_ser_name = '/gng_biased/path_planner'
        self.file_name = rospy.get_param('~file_name',"/home/ridhwan/experiment3/set2/prm_halton/path_topic.json")

        # 1. subscribe to lidar
        self.lidar_sub = rospy.Subscriber('/scan_multi',LaserScan,self.scan_cb)

        # 3. Local Planner client
        self.controller_client = SimpleActionClient('/move_base_flex/exe_path',ExePathAction)

        rospy.loginfo("Experiment3 is running")


    def send_goal(self):
        
        if self.controller_client.wait_for_server(rospy.Duration(3.0)):
            print("action available")
            with open(self.file_name) as json_file:
                path_dict = json.load(json_file)
            path_msg:Path = message_converter.convert_dictionary_to_ros_message("nav_msgs/Path",path_dict)
            path_msg.header.frame_id = 'map'

            last_pose:PoseStamped = path_msg.poses[-1]
            last_pose.pose.orientation.z = 1.0
            path_msg.poses.append(last_pose)
            goal = ExePathGoal()
            goal.path = path_msg
            
            self.controller_client.send_goal(goal)

            self.controller_client.wait_for_result()
            print("done")
        else:
            print("action not available")
        return

    def scan_cb(self,msg:LaserScan):
        self.laser_scan = msg
        # Open our existing CSV file in append mode
        # Create a file object for this file
        with open('/home/ridhwan/experiment3/scan.csv', 'a') as f_object:

            # Pass this file object to csv.writer()
            # and get a writer object
            writer_object = writer(f_object)
            writer_object.writerow(msg.ranges)
            # Close the file object
            f_object.close()


if __name__ == '__main__':
    rospy.init_node('experiment3')
    server = Ex3ExePath()

    path_pub = rospy.Publisher("/experiment3/path",Path,queue_size=10,latch=True)
    node_marker_pub = rospy.Publisher("/experiment3/node",Marker,queue_size=10,latch=True)
    edge_marker_pub = rospy.Publisher("/experiment3/edge",Marker,queue_size=10,latch=True)

    with open("/home/ridhwan/experiment3/set2/prm_halton/path_topic.json") as json_file:
        path_dict = json.load(json_file)
        path_msg:Path = message_converter.convert_dictionary_to_ros_message("nav_msgs/Path",path_dict)
        path_msg.header.frame_id = 'map'
        path_pub.publish(path_msg)

    with open("/home/ridhwan/experiment3/set2/prm_halton/node_marker_topic.json") as json_file:
        node_dict = json.load(json_file)
        node_marker_msg:Marker = message_converter.convert_dictionary_to_ros_message("visualization_msgs/Marker",node_dict)
        node_marker_pub.publish(node_marker_msg)

    with open("/home/ridhwan/experiment3/set2/prm_halton/edge_marker_topic.json") as json_file:
        edge_dict = json.load(json_file)
        edge_marker_msg:Marker = message_converter.convert_dictionary_to_ros_message("visualization_msgs/Marker",edge_dict)
        edge_marker_pub.publish(edge_marker_msg)

    

    

    server.send_goal()
    rospy.spin()