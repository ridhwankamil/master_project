#!/usr/bin/env python3

import rospy
import json
import numpy as np
import math
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from master_project.srv import RequestPath,RequestPathRequest,RequestPathResponse
from rospy_message_converter import message_converter
from geometry_msgs.msg import PoseStamped

def node_sub_cb(msg:Marker):
  node_marker_dict = message_converter.convert_ros_message_to_dictionary(msg)
  print("Started writing node marker dict to a file")
  with open("/home/ridhwan/experiment3/node_marker_topic.json", "w") as fp:
      json.dump(node_marker_dict, fp)  # encode dict into JSON
  rospy.loginfo("receive nodes marker")


def edge_sub_cb(msg:Marker):
  edge_marker_dict = message_converter.convert_ros_message_to_dictionary(msg)
  print("Started writing edge marker dict to a file")
  with open("/home/ridhwan/experiment3/edge_marker_topic.json", "w") as fp:
      json.dump(edge_marker_dict, fp)  # encode dict into JSON

  rospy.loginfo("receive edges marker")


def path_sub_cb(msg:Path):
  rospy.loginfo(len(msg.poses))
  print("interpolating pose")
  interpolated_path = Path()

  for index in range(len(msg.poses)-2):
      temp_path = interpolate_straight(msg.poses[index],msg.poses[index+1])
      for item in temp_path:
          interpolated_path.poses.append(item)
  
  interpolated_path.poses.append(msg.poses[-1])

  print(len(interpolated_path.poses))

  interpolated_path_dict = message_converter.convert_ros_message_to_dictionary(interpolated_path)
  print("Started writing interpolated path dict to a file")
  with open("/home/ridhwan/experiment3/path_topic.json", "w") as fp:
      json.dump(interpolated_path_dict, fp)  # encode dict into JSON

  rospy.loginfo("receive shortest path")

def interpolate_straight(source:PoseStamped,target:PoseStamped):
    injection_pose_list = []
    #measure distance to from start to target
    dist = calc_eucl_dist(source,target)
    # print("distance: ", dist)

    #from distance we estimate interval count between source to target with 0.01 metre interval
    injection_count = round(dist/0.005)

    for t in np.arange(0.0,1.0,1/injection_count):
        pose:PoseStamped = pose_interpolate(source,target,t)
        injection_pose_list.append(pose)

    return injection_pose_list 
    
def pose_interpolate(source:PoseStamped,target:PoseStamped,t):

    source_x = source.pose.position.x
    source_y = source.pose.position.y

    target_x = target.pose.position.x
    target_y = target.pose.position.y


    x = source_x + (target_x - source_x) * t
    y = source_y + (target_y - source_y) * t

    pose = PoseStamped()
    pose.header = source.header
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = x
    pose.pose.position.y = y

    return pose 

def calc_eucl_dist(source:PoseStamped,target:PoseStamped):
    x0 = source.pose.position.x
    x1 = target.pose.position.x

    y0 = source.pose.position.y
    y1 = target.pose.position.y

    sum = math.sqrt(math.pow(x1-x0,2) + math.pow(y1-y0,2))
    # sum= math.sqrt(abs((B[0]-A[0]) + (B[1]-A[1])))
    return sum      
    



if __name__ == '__main__':
    rospy.init_node('experiment3_save_path')
    # global node_done
    # global edge_done
    # global path_done

    start_pos = [25.55003398853261,4.156202836737073]
    start_orientation = [-0.6887385259492977,0.7250098226046241]


    # set 1
    # goal_pos = [14.200572706563799, -16.011745970199463]
    # goal_orientation = [0.718051074337617, 0.6959904127519244]

    # set 2
    goal_pos = [6.203347255591034, 11.787475688787042]
    goal_orientation = [-0.02020640499228135, 0.9997958297558996]
    

    # 1st get topic name of generated roadmap path (node&edge)
    node_topic_name = rospy.get_param('~node_topic_name','node_marker')
    edge_topic_name = rospy.get_param('~edge_topic_name','edge_marker')

    # 2nd get topic name of generated shortest path
    path_topic_name = rospy.get_param('~path_topic_name','shortest_path')
    planner_service_name = rospy.get_param('~planner_name','path_planner') 

    # 3rd subscribe to all those topics
    node_sub = rospy.Subscriber(node_topic_name,Marker,node_sub_cb)
    edge_sub = rospy.Subscriber(edge_topic_name,Marker,edge_sub_cb)
    path_sub = rospy.Subscriber(path_topic_name,Path,path_sub_cb)


    rospy.wait_for_service(planner_service_name,timeout=5.0)
    print("planner service available")
    service = rospy.ServiceProxy(planner_service_name, RequestPath)
    req = RequestPathRequest()
    req.start.pose.position.x = start_pos[0]
    req.start.pose.position.y = start_pos[1]

    req.goal.pose.position.x = goal_pos[0]
    req.goal.pose.position.y = goal_pos[1]
    req.custom_weight_function = False
    res:RequestPathResponse = service(req)

    print(len(res.path.poses))

    rospy.spin()
    # rate = rospy.Rate(1)
    # while not node_done and not edge_done and not path_done:
    #     rospy.loginfo("waiting for others topics to receive")
    #     rate.sleep()

    