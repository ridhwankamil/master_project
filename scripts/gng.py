#!/usr/bin/env python3

import enum
from operator import index
import rospy
import numpy as np
from occ_map import OccupancyMap

import networkx as nx
import math
from pprint import pprint
from neupy import algorithms, utils



from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import Point, PoseStamped
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray
from matplotlib import pyplot as plt

from scipy.spatial import KDTree
from scipy import ndimage
import time

# from sklearn.neighbors import KDTree

#parrent class
class PRM():
    def __init__(self):
        self.nodes_kd_tree:KDTree
       
        # self.map_pub2 = rospy.Publisher('/map2',OccupancyGrid, latch=True, queue_size=10)
        # create instacne of occ-map object
        self.occ_map = OccupancyMap()

        self.graph:nx.Graph = None

        self.done = False


        # subscribe to map topic
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.map_pub = rospy.Publisher('/inflated_map',OccupancyGrid, latch=True, queue_size=10)
        self.node_marker_pub = rospy.Publisher('/node_marker',Marker,queue_size=10,latch=True)
        self.edge_marker_pub = rospy.Publisher('/edge_marker', Marker, queue_size=10,latch=True)
        self.path_pub = rospy.Publisher('/shortest_path',Path,queue_size=1,latch=True)
        self.gng_pub = rospy.Publisher('/gng',Marker,queue_size=10,latch=True)
        print('done')

    def map_callback(self,map_msg):
        # Initialize map every time new map receive
        self.occ_map.init_map(map_msg)
        self.done = True

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
        node_marker.scale.x = 0.02
        node_marker.scale.y = 0.02
        node_marker.scale.z = 0.01
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
        edge_marker.scale.x = 0.02
        edge_marker.scale.y = 0.02
        edge_marker.scale.z = 0.02
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

    def query_shortest_path(self,start,goal):
        #dummy start & goal
        start = np.array([0.0,0.0])
        goal = np.array([-4.9,-4.0])
        # 1. temperory adding start and goal node, willbe deleted later
        self.graph.add_node("start",x=start[0],y=start[1])
        self.graph.add_node("goal",x=goal[0],y=goal[1])

        width = self.occ_map.map.info.width
        height = self.occ_map.map.info.height
        if width > height:radius = width
        else:radius = height

        dists, indexes = self.nodes_kd_tree.query(start,k=1000,distance_upper_bound=radius)
        for item in indexes:
            # check collision

            #add to graph
            self.graph.add_edge("start",item)
            break

        dists, indexes = self.nodes_kd_tree.query(goal,k=1000,distance_upper_bound=radius)
        for item in indexes:
            # check collision

            #add to graph
            self.graph.add_edge("goal",item)
            break

        waypoint = nx.dijkstra_path(self.graph,"start","goal")

        path= Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()

        arr = np.array(waypoint)
        # print(arr)

        # print(self.graph.nodes['goal'])
        for item in waypoint:
            pose_stamped = PoseStamped()
            data = self.graph.nodes[item]
            # print(data)
            pose_stamped.pose.position.x = data['x']
            pose_stamped.pose.position.y = data['y']
            pose_stamped.pose.orientation.w = 1.0

            path.poses.append(pose_stamped)

        self.path_pub.publish(path)

        self.done = True

    def train_gng(self,):
        # 1. filter only free space in the map and get the each pixel position value into [row,col]
        data = self.image_to_data(self.occ_map.np_map.astype(np.uint8))

        # 2. apply conversion from [row,col] -> [x,y]
        data_xy = np.apply_along_axis(self.occ_map.grid_to_world,1,data)

        gng = self.create_gng(max_nodes=200)

        # for epoch in range(5):
        gng.train(data_xy, epochs=1)

        return gng

    def image_to_data(self,map):
        data = []
        for (x, y), value in np.ndenumerate(map):
            if value == 0:
                data.append([x, y])
        return np.array(data)

    def draw_image(self,graph, show=False):
        for node_1, node_2 in graph.edges:
            weights = np.concatenate([node_1.weight, node_2.weight])
            line, = plt.plot(*weights.T, color='black')
            plt.setp(line, linewidth=0.2, color='black')

        plt.xticks([], [])
        plt.yticks([], [])
        
        if show:
            plt.show()

    def create_gng(self,max_nodes, step=0.2, n_start_nodes=2, max_edge_age=50):
        return algorithms.GrowingNeuralGas(
            n_inputs=2,
            n_start_nodes=n_start_nodes,

            shuffle_data=True,
            verbose=True,

            step=step,
            neighbour_step=0.005,

            max_edge_age=max_edge_age,
            max_nodes=max_nodes,

            n_iter_before_neuron_added=100,
            after_split_error_decay_rate=0.5,
            error_decay_rate=0.995,
            min_distance_for_update=0.01,
        )

    def convert_gng_to_nx(self,g):
        nxgraph = nx.Graph()
        nodeid = {}
        for indx, node in enumerate(g.graph.nodes):
            nodeid[node] = indx
            nxgraph.add_node(nodeid[node], pos=(node.weight[0][0], node.weight[0][1]))
        # positions = nx.get_node_attributes(nxgraph, "pos")
        for node_1, node_2 in g.graph.edges:
                nxgraph.add_edge(nodeid[node_1], nodeid[node_2])
        return nxgraph


if __name__ == '__main__':
    rospy.init_node('prm')
    node = PRM()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if node.done:
            print('GNG Start Learning')
            gng = node.train_gng()

            node.graph = node.convert_gng_to_nx(gng)

            node_marker = node.generate_node_marker()
            edge_marker = node.generate_edge_marker()
            node.node_marker_pub.publish(node_marker)
            node.edge_marker_pub.publish(edge_marker)

            node.graph.add_node('start', pos=(0.0, 0.0))
            result = nx.k_nearest_neighbors(node.graph,nodes='start')
            print(result)

            node.draw_image(gng.graph,show=True)

            node.done = False


        rate.sleep()
        
    rospy.spin()



# origin_x = node.occ_map.map.info.origin.position.x
# origin_y = node.occ_map.map.info.origin.position.y
# max_x = node.occ_map.max['x']
# max_y = node.occ_map.max['y']


# create 2 subplot
# figure, axis = plt.subplots(1, 2)
# subplot 1
# axis[0].imshow(node.occ_map.np_map, cmap='gray_r',origin='lower', vmin = 0, vmax = 1, extent=[origin_x,max_x,origin_y,max_y])
# subplot 2
# axis[1].scatter(scatter_data[1],scatter_data[0], s=5, alpha=1)
#show plot window
# plt.show()