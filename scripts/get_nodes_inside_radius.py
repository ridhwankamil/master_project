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
    self.max_nodes = rospy.get_param('~max_nodes',1500) # number of nodes to be sampled
    self.max_connection_dist = rospy.get_param('~max_connection_distance',1.0)
    self.robot_radius = rospy.get_param('~robot_radius', 0.2) # the size of robot

    self.obstacle_kd_tree:KDTree # kd-tree for nodes array
    self.skel_kd_tree:KDTree # kd-tree for nodes array

    self.inflated_map = OccupancyMap() #original map
    self.skel_map = OccupancyMap()
    self.skel_inflated_map = OccupancyMap()

    self.done_map_callback = False
    self.done_node_marker_callback = False

    self.node_marker_msg = Marker()

    # subscribe to map topic
    self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    self.map_sub = rospy.Subscriber('node_marker',Marker,self.node_marker_callback)

    # publisher
    self.skel_inflated_map_pub = rospy.Publisher('skel_inflated_map', OccupancyGrid, latch=True, queue_size=1)
    

    print("done init experiment 1")

  def map_callback(self,map_msg):
    #every time receive map it will resampled
    self.done_map_callback = False
    # 1. Initialize original map every time new map receive
    self.inflated_map.init_map(map_msg)
    self.inflated_map.inflate(self.robot_radius)

    # 2. Generate inflated Skeletonized map
    skeletonized_map = self.generate_skeletonized_map(self.inflated_map.map)
    self.skel_inflated_map.init_map(skeletonized_map)
    self.skel_inflated_map.inflate(self.robot_radius)
    # self.skel_map_pub.publish(self.skel_map.map)
    self.skel_inflated_map_pub.publish(self.skel_inflated_map.map)
    

    self.done_map_callback = True
    print('map_callback done')

  def node_marker_callback(self,msg:Marker):
    # self.done_node_marker_callback = False
    # pprint(msg)
    self.node_marker_msg = msg
    self.done_node_marker_callback = True
    print('node_marker_callback done')

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

  def generate_skel_dist_map(self,map:OccupancyGrid):
    # 1.0 copy originalmap object
    dist_map = copy.deepcopy(map)

    # 2.0 create numpy array of map data
    np_map = np.array(dist_map.data)
    # 2.1 convert to binary array
    np_map[np_map == -1] = 1
    np_map[np_map == 100] = 1
    # 2.2 convert numpy array to boolean type
    np_map = np_map.astype(bool)
    # 2.3 change the shape of numpy array into 2d
    np_map.shape = (dist_map.info.height,dist_map.info.width)

    # print(np_map)

    # 3.0 generate distance transform
    np_map:np.ndarray = ndimage.distance_transform_edt(invert(np_map))

    final_np_map = copy.deepcopy(np_map)

    # 4.1 generate map data array from numpy map array
    np_map[np_map > 100] = 100

    np_map = np_map.astype(np.int8)
    # print(np_map.tolist())
    # np_map = np_map.astype(np.uint8)
    # np_map[np_map == 1] = 100

    temp = np_map.flatten().tolist()
    dist_map.data = temp

    # print("DATA:\n",np_map)

    return dist_map,final_np_map

  def node_sampling(self):
    self.nodes_arr = []
    min_x = self.inflated_map.map.info.origin.position.x
    max_x = self.inflated_map.max['x']

    min_y = self.inflated_map.map.info.origin.position.y
    max_y = self.inflated_map.max['y']

    while len(self.nodes_arr)<self.max_nodes:
      # generate random point based on axis origin & axis max
      x_rand = random.uniform(min_x, max_x)
      y_rand = random.uniform(min_y, max_y)

      if self.inflated_map.check_occupancy([x_rand,y_rand]):
          self.nodes_arr.append([x_rand,y_rand])
          # print(len(self.nodes_arr))
          
      else:
          # print('occupied')
          pass
    
  def generate_nxgraph(self):
    nxgraph = nx.Graph()
    k = len(self.nodes_arr)
    for idx, node in enumerate(self.nodes_arr):
      # print(idx, ": ",node)
      nxgraph.add_node(idx, pos=(node[0], node[1]))

    for node in list(nxgraph.nodes):
      start_point = list(nxgraph.nodes[node]['pos'])
      dists, indexes = self.nodes_kd_tree.query(start_point,k=k,distance_upper_bound=self.max_connection_dist)


      for item in indexes:
        if item >= len(self.nodes_arr):
          break
        end_point = list(nxgraph.nodes[item]['pos'])
        eucl_dist = math.dist(start_point,end_point)
        if eucl_dist == 0.0:
          continue
        # nxgraph.add_edge(nodeid[node_1], nodeid[node_2])
        if(self.inflated_map.check_collision(start_point,end_point)):
          nxgraph.add_edge(node,item)
        # print("euclidean_distance: ",eucl_dist)
    
    return nxgraph
  
  def generate_node_marker(self) -> Marker:
    """Return edge marker"""
    #check if graph is initialized
    if self.graph is None:
        print("Graph object uninitilaized")
        return

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
    for item in self.graph.nodes(data='pos'):
        point = Point()
        point.x = item[1][0]
        point.y = item[1][1]
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


if __name__ == '__main__':
  rospy.init_node('experiment_1')
  node = EXPERIMENT1()
  rate = rospy.Rate(0.5)
  inside_node_pub = rospy.Publisher('inside_nodes',Marker,latch=True,queue_size=10)
  file_ = rospy.get_param('~file_location', '/home/ridhwan/default.csv') 

  def generate_inside_node_marker(arr) -> Marker:
    """Return inside node marker"""


    # 1. initialize node marker
    node_marker = Marker()
    node_marker.type = 8
    node_marker.ns = 'nodes'
    node_marker.id = 100
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
        point.x = item[0]
        point.y = item[1]
        node_marker.points.append(point)
    
    return node_marker


  while not rospy.is_shutdown():
    # print("running")
    # print(node.done_map_callback, "  ", node.done_node_marker_callback)
    if node.done_map_callback and node.done_node_marker_callback:
      print("start experiment")
      is_inside = []
      is_outside = []
      for item in node.node_marker_msg.points:
        value = node.skel_inflated_map.check_pixel_value([item.x,item.y])
        if(value):
          is_inside.append([item.x,item.y])
        else:
          is_outside.append([item.x,item.y])

      marker = generate_inside_node_marker(is_inside)
      inside_node_pub.publish(marker)

      d= []
      with open(file_) as f:
        d= f.readlines()

      with open(file_, 'w') as f:
        temp = str(len(node.node_marker_msg.points)) + "," + str(len(is_inside)) +"," +str(len(is_outside))+"\n"
        d.append(temp)
        f.writelines(d)

      print("done experiment")
      node.done_node_marker_callback = False
    rate.sleep()
    
      
  rospy.spin()