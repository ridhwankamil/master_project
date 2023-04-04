#!/usr/bin/env python3
import rospy
from scipy.spatial import KDTree
import numpy as np
from occ_map import OccupancyMap
import random

import networkx as nx
import math
import copy
from skimage.morphology import skeletonize
from skimage.util import invert
from scipy import ndimage

from nav_msgs.msg import OccupancyGrid,Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from master_project.srv import RequestPath,RequestPathRequest,RequestPathResponse

from pprint import pprint


class EXPERIMENT1():
  def __init__(self):
    self.obstacle_kd_tree:KDTree # kd-tree for obstacle array
    self.skel_kd_tree:KDTree # kd-tree for nodes array

    self.ori_map = OccupancyMap() #original map
    self.skel_map = OccupancyMap()

    self.done_map_callback = False

    self.node_marker_msg = Marker()

    # subscribe to map topic
    self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)


    # publisher
    self.skel_inflated_map_pub = rospy.Publisher('skel_inflated_map', OccupancyGrid, latch=True, queue_size=1)
    self.obs_marker_pub = rospy.Publisher('obstacle_marker',Marker,latch=True,queue_size=10)
    

    print("done init experiment path success rate")

  def map_callback(self,map_msg):
    #every time receive map reinitializa
    self.done_map_callback = False
    # 1. Initialize original map every time new map receive
    self.ori_map.init_map(map_msg)
    # obtain obstacle xy data
    data = self.obstacle_to_data(self.ori_map.np_map)
    self.obs_xy = np.apply_along_axis(self.ori_map.grid_to_world,1,data)

    obs_marker = self.generate_obs_marker()
    self.obs_marker_pub.publish(obs_marker)
    # test_points = [[1,1],[-1,1],[-1,-1]]
    self.obstacle_kd_tree = KDTree(self.obs_xy)

    # 2. Generate Skeletonized map
    skeletonized_map = self.generate_skeletonized_map(self.ori_map.map)
    self.skel_map.init_map(skeletonized_map)
    
    self.done_map_callback = True
    print('map_callback done')

  def generate_skeletonized_map(self,map:OccupancyGrid):
    # 1.0 copy originalmap object
    skel_map = copy.deepcopy(map)

    # 2.0 create numpy array of map data
    np_map = np.array(skel_map.data)
    # 2.1 convert to binary array
    np_map[np_map == -1] = 1
    np_map[np_map == 100] = 1
    # 2.2 convert numpy array to boolean type
    np_map = np_map.astype(bool)
    # 2.3 change the shape of numpy array into 2d
    np_map.shape = (skel_map.info.height,skel_map.info.width)

    # 3.0 skeletonized
    np_map:np.ndarray = skeletonize(invert(np_map))

    # 4.1 generate map data array from numpy map array
    np_map = np_map.astype(np.uint8)
    np_map[np_map == 1] = 100

    skel_map.data = np_map.flatten().tolist()

    return skel_map
    
  def generate_kdtree_marker(self,arr) -> Marker:
    # 1. initialize node marker
    node_marker = Marker()
    node_marker.type = 8
    node_marker.ns = 'nodes'
    node_marker.id = 0
    node_marker.header.frame_id = 'map'
    node_marker.header.stamp = rospy.Time.now()
    node_marker.lifetime = rospy.Duration(0)
    node_marker.scale.x = 0.1
    node_marker.scale.y = 0.1
    node_marker.scale.z = 0.1
    node_marker.color.a = 1.0
    node_marker.color.r = 1.0

    # 2. iterate node 1 by 1 & append to node marker
    for item in arr:
        point = Point()
        print('inside', self.obs_xy[item])
        point.x = self.obs_xy[item][0]
        point.y = self.obs_xy[item][1]
        node_marker.points.append(point)
    
    return node_marker

  def generate_obs_marker(self) -> Marker:
    """Return obstacle marker"""
    # 1. initialize node marker
    node_marker = Marker()
    node_marker.type = 8
    node_marker.ns = 'nodes'
    node_marker.id = 0
    node_marker.header.frame_id = 'map'
    node_marker.header.stamp = rospy.Time.now()
    node_marker.lifetime = rospy.Duration(0)
    node_marker.scale.x = 0.1
    node_marker.scale.y = 0.1
    node_marker.scale.z = 0.1
    node_marker.color.a = 1.0
    node_marker.color.r = 1.0

    # 2. iterate node 1 by 1 & append to node marker
    for item in self.obs_xy:
        point = Point()
        point.x = item[0]
        point.y = item[1]
        node_marker.points.append(point)
    
    return node_marker

  def generate_edge_marker(self) -> Marker:
    """Return edge marker"""
    #check if graph is initialized
    if self.graph is None:
        print("Graph object uninitilaized")
        return

    edge_marker = Marker()
    edge_marker.type = 5
    edge_marker.ns = 'edge_list'
    edge_marker.id = 10
    edge_marker.header.frame_id = 'map'
    edge_marker.header.stamp = rospy.Time.now()
    edge_marker.lifetime = rospy.Duration(0)
    edge_marker.scale.x = 0.01
    edge_marker.scale.y = 0.01
    edge_marker.scale.z = 0.01
    edge_marker.color.a = 1.0
    edge_marker.color.b = 1.0
    edge_marker.pose.orientation.w = 1.0

    # 2. iterate edge 1 by 1 & append to edge marker
    for item in self.graph.edges:
        pos = self.graph.nodes[item[0]]['pos']
        start = Point()
        start.x = pos[0]
        start.y = pos[1]

        pos = self.graph.nodes[item[1]]['pos']
        end = Point()
        end.x = pos[0]
        end.y = pos[1]

        edge_marker.points.append(start)
        edge_marker.points.append(end)
    
    return edge_marker

  def obstacle_to_data(self,map):
    data = []
    for (x, y), value in np.ndenumerate(map):
        if value == 1:
            data.append([x, y])
    return np.array(data)

if __name__ == '__main__':
  rospy.init_node('experiment_2')
  node = EXPERIMENT1()
  rate = rospy.Rate(0.5)

  p = [
    [-5.4969, -3.3565],
    [6.6549, -4.5701],
    [6.521, 3.9988],
    [0.8364, 1.7727],
    [-2.649, 4.4471],
    [-6.3008, 3.9737]
  ]
  

  pprint(p)

  while not node.done_map_callback:
    print("waiting")
    rate.sleep()
  
 
  try:
    rospy.wait_for_service('/gng_biased/path_planner',timeout=5.0)
    print("service available")
    for outer_item in p:
      for inner_item in p:
        service = rospy.ServiceProxy('/gng_biased/path_planner', RequestPath)
        if outer_item == inner_item:
          pass
        else:
          print("start: ", outer_item, " , ", "goal: ", inner_item)
        
          req = RequestPathRequest()
          req.start.pose.position.x = outer_item[0]
          req.start.pose.position.y = outer_item[1]

          req.goal.pose.position.x = inner_item[0]
          req.goal.pose.position.y = inner_item[1]
          req.custom_weight_function = False
          res:RequestPathResponse = service(req)


          success = res.state

          dist_to_obs = []
          min_val = 0
          max_val = 0
          avg_value = 0
          if success:
            for item in res.path.poses:
              p_x = item.pose.position.x
              p_y = item.pose.position.y
              dists,indexes = node.obstacle_kd_tree.query([p_x,p_y],k=1,distance_upper_bound=100)
              # print(dists)
              dist_to_obs.append(dists)

            dist_to_obs.sort() #sort list ascending
            min_val = dist_to_obs[0]
            max_val = dist_to_obs[-1]
            avg_value = sum(dist_to_obs)/len(dist_to_obs)
          prev = []
          with open("/home/ridhwan/experiment2_gng.csv") as f:
            prev = f.readlines()
          with open("/home/ridhwan/experiment2_gng.csv","w") as f:
            prev.append(str(success)+","+str(min_val)+","+str(max_val)+","+str(avg_value)+"\n")
            f.writelines(prev)
            
          print(str(success)+","+str(min_val)+","+str(max_val)+","+str(avg_value))

          # rospy.sleep(5)
  except rospy.ROSException as err:
    print(err)


  try:
    rospy.wait_for_service('/gng_biased/path_planner',timeout=5.0)
    print("service available")
    for outer_item in p:
      for inner_item in p:
        service = rospy.ServiceProxy('/gng_biased/path_planner', RequestPath)
        if outer_item == inner_item:
          pass
        else:
          print("start: ", outer_item, " , ", "goal: ", inner_item)
        
          req = RequestPathRequest()
          req.start.pose.position.x = outer_item[0]
          req.start.pose.position.y = outer_item[1]

          req.goal.pose.position.x = inner_item[0]
          req.goal.pose.position.y = inner_item[1]
          req.custom_weight_function = True
          res:RequestPathResponse = service(req)


          success = res.state

          dist_to_obs = []
          min_val = 0
          max_val = 0
          avg_value = 0
          if success:
            for item in res.path.poses:
              p_x = item.pose.position.x
              p_y = item.pose.position.y
              dists,indexes = node.obstacle_kd_tree.query([p_x,p_y],k=1,distance_upper_bound=100)
              # print(dists)
              dist_to_obs.append(dists)

            dist_to_obs.sort() #sort list ascending
            min_val = dist_to_obs[0]
            max_val = dist_to_obs[-1]
            avg_value = sum(dist_to_obs)/len(dist_to_obs)
          prev = []
          with open("/home/ridhwan/experiment2_gng.csv") as f:
            prev = f.readlines()
          with open("/home/ridhwan/experiment2_gng.csv","w") as f:
            prev.append(str(success)+","+str(min_val)+","+str(max_val)+","+str(avg_value)+"\n")
            f.writelines(prev)
            
          print(str(success)+","+str(min_val)+","+str(max_val)+","+str(avg_value))

          # rospy.sleep(5)
  except rospy.ROSException as err:
    print(err)
 

    
      
