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

from scipy.stats import qmc


class PRMHALTONPLANNER():
  def __init__(self):
    self.max_nodes = rospy.get_param('~max_nodes',500) # number of nodes to be sampled
    self.max_connection_dist = rospy.get_param('~max_connection_distance', 1.0) # the size of robot
    self.robot_radius = rospy.get_param('~robot_radius', 0.2) # the size of robot
    # self.mode = rospy.get_param('~mode',0) # 


    self.nodes_kd_tree:KDTree # kd-tree for nodes array
    self.nodes_arr:np.array # nodes array

    self.inflated_map = OccupancyMap() #original map
    self.skel_map = OccupancyMap()
    self.skel_dist_map = OccupancyMap()

    self.graph:nx.Graph = None
    self.done = False

    # subscribe to map topic
    self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    # publisher
    self.inflated_map_pub = rospy.Publisher('inflated_map',OccupancyGrid, latch=True, queue_size=1)
    self.skel_map_pub = rospy.Publisher('skel_map',OccupancyGrid, latch=True, queue_size=1)
    self.skel_dist_map_pub = rospy.Publisher('dist_map', OccupancyGrid, latch=True, queue_size=1)
    self.node_marker_pub = rospy.Publisher('node_marker',Marker,queue_size=10,latch=True)
    self.edge_marker_pub = rospy.Publisher('edge_marker', Marker, queue_size=10,latch=True)
    self.path_pub = rospy.Publisher('shortest_path',Path,queue_size=1,latch=True)

    self.req_path_service = rospy.Service("path_planner", RequestPath, self.request_path_callback)
    print("done init")

  def map_callback(self,map_msg):
    #every time receive map it will resampled
    self.done = False
    # 1. Initialize original map every time new map receive
    self.inflated_map.init_map(map_msg)
    self.inflated_map.inflate(self.robot_radius)
    self.inflated_map_pub.publish(self.inflated_map.map)

    # 2. Generate Skeletonized map
    skeletonized_map = self.generate_skeletonized_map(self.inflated_map.map)
    self.skel_map.init_map(skeletonized_map)
    self.skel_map_pub.publish(self.skel_map.map)

    # 4. Generate Distance-map from skel map
    skel_dist_map,np_skel_dist_map = self.generate_skel_dist_map(self.skel_map.map)
    self.skel_dist_map.init_map(skel_dist_map)
    self.skel_dist_map.np_map = np_skel_dist_map
    self.skel_dist_map_pub.publish(self.skel_dist_map.map)

    print("\nStart Sampling")
    rospy.sleep(3.0)
    self.nodes_arr = []

    # start sampling free point in inflated map
    self.node_sampling()

    # generate kd-tree from array of node
    self.nodes_kd_tree = KDTree(self.nodes_arr)
    # pprint(self.nodes_arr)

    self.graph = self.generate_nxgraph()



    # convert gng graph object to networkx graph object
    # self.graph = self.convert_gng_to_nx(self.gng)

    # extract array of nodes from nx graph
    

    #generate kd-tree from array of node
    # self.nodes_kd_tree = KDTree(self.nodes_arr)
    
    # publish all marker for rviz visualization
    node_marker = self.generate_node_marker()
    edge_marker = self.generate_edge_marker()
    node.node_marker_pub.publish(node_marker)
    node.edge_marker_pub.publish(edge_marker)

    # query dummy path data
    # path = self.query_shortest_path([],[])

    # self.path_pub.publish(path)

    self.done = True
    print('PRM Finished sampling')

  def request_path_callback(self,req:RequestPathRequest):
    # res = RequestPathResponse()
    # point = PoseStamped()
    # res.path.poses.append(point)
    res= RequestPathResponse()
    if not self.done:
        res.state = False
        res.message = "planner not ready"
        return res

    start = np.array([req.start.pose.position.x,req.start.pose.position.y])
    goal = np.array([req.goal.pose.position.x,req.goal.pose.position.y])

    path = self.query_shortest_path(start,goal,req.custom_weight_function)
    path.poses.append(req.goal)
    self.path_pub.publish(path) # for rviz

    res.path = path

    return res

  def query_shortest_path(self,start,goal,custom_weight_function):
    #dummy start & goal
    # start = np.array([0.0,0.0])
    # goal = np.array([1.5,3.0])
    # 1. temperory adding start and goal node, willbe deleted later
    self.graph.add_node("start",pos=(start[0],start[1]))
    self.graph.add_node("goal",pos=(goal[0],goal[1]))

    width = self.inflated_map.map.info.width
    height = self.inflated_map.map.info.height
    if width > height:radius = width
    else:radius = height

    dists, indexes = self.nodes_kd_tree.query(start,k=10,distance_upper_bound=radius)
    for item in indexes:
        # check collision
        # print(item)
        p1 = self.graph.nodes["start"]['pos']
        p2 = self.graph.nodes[item]['pos']
        # print(p2)
        if self.inflated_map.check_collision(p1,p2):
            #add to graph
            self.graph.add_edge("start",item)

    dists, indexes = self.nodes_kd_tree.query(goal,k=10,distance_upper_bound=radius)
    for item in indexes:
        # check collision
        p1 = self.graph.nodes["goal"]['pos']
        p2 = self.graph.nodes[item]['pos']
        if self.inflated_map.check_collision(p1,p2):
        #add to graph
            self.graph.add_edge("goal",item)

    def custom_weight_func(u, v, d):
        p1 = self.graph.nodes[u]['pos']
        p2 = self.graph.nodes[v]['pos']
        # 1st find euclidean distance
        eucl_dist = math.dist(p1,p2)
        # check the target point occupancy value
        value = self.skel_dist_map.check_pixel_value(p2)
        return eucl_dist + value

    def weight_func(u, v, d):
        p1 = self.graph.nodes[u]['pos']
        p2 = self.graph.nodes[v]['pos']
        # find euclidean distance
        eucl_dist = math.dist(p1,p2)
        return eucl_dist

    if custom_weight_function:
        waypoint = nx.dijkstra_path(self.graph,"start","goal",custom_weight_func)
    else:
        waypoint = nx.dijkstra_path(self.graph,"start","goal",weight_func)

    path= Path()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()

    # arr = np.array(waypoint)
    # print(arr)

    # print(self.graph.nodes['goal'])
    for item in waypoint:
        pose_stamped = PoseStamped()
        data = self.graph.nodes[item]
        # print(data)
        pose_stamped.pose.position.x = data['pos'][0]
        pose_stamped.pose.position.y = data['pos'][1]
        pose_stamped.pose.orientation.w = 1.0

        path.poses.append(pose_stamped)

    # self.graph.remove_node('start')
    # self.graph.remove_node('goal')


    return path

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

    print("lower bound: ",[min_x, min_y])

    sampler = qmc.Halton(d=2, scramble=False)
    sample = []
    while len(self.nodes_arr)<self.max_nodes:
      # generate halton point based on axis origin & axis max
      sampling_seed = self.max_nodes - len(self.nodes_arr)
      print("going to sample: ", sampling_seed)
      sample = sampler.random(n=sampling_seed)
      l_bounds = [min_x, min_y]
      u_bounds = [max_x,max_y]

      print("lower bound: ",[min_x, min_y])
      print("upper bound: ",[max_x,max_y])
      sample_bound = qmc.scale(sample, l_bounds, u_bounds)

      # pprint(sample_bound)

      for item in sample_bound:
        if self.inflated_map.check_occupancy([item[0],item[1]]):
          self.nodes_arr.append([item[0],item[1]])
          # print(len(self.nodes_arr))
          # print([item[0],item[1]])
            
        else:
          # print([item[0],item[1]])
          # print('occupied')
          pass
      print(len(self.nodes_arr))
    
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
  rospy.init_node('prm_halton_planner')
  node = PRMHALTONPLANNER()
  # rate = rospy.Rate(1)
      
  rospy.spin()