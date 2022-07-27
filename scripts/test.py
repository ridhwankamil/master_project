#!/usr/bin/env python3

import enum
from operator import index
import rospy
import numpy as np
from occ_map import OccupancyMap

import networkx as nx
import math
from pprint import pprint as print


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


         # subscribe to map topic
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.map_pub = rospy.Publisher('/inflated_map',OccupancyGrid, latch=True, queue_size=10)
        self.node_marker_pub = rospy.Publisher('/node_marker',Marker,queue_size=10,latch=True)
        self.edge_marker_pub = rospy.Publisher('/edge_marker', Marker, queue_size=10,latch=True)
        self.path_pub = rospy.Publisher('/shortest_path',Path,queue_size=1,latch=True)

    def map_callback(self,map_msg):
        # Initialize map every time new map receive
        self.occ_map.init_map(map_msg)
        self.occ_map.init_skel_map()

        # Find each pixel of skeletonized
        skel_grid = np.where(self.occ_map.np_skel_map == True)

        # stack two array into 1 array of[x,y]
        skel_grid = np.stack(skel_grid,axis=1)

        # apply conversion
        skel_pixel = np.apply_along_axis(node.occ_map.grid_to_world,1,skel_grid)

        # generate edges from nearest neighbour
        # 1. create kd tree from nodes array
        self.nodes_kd_tree = KDTree(skel_pixel,copy_data=True)

        # 2.find max radius for knn
        resolution= self.occ_map.map.info.resolution
        radius = math.hypot(resolution, resolution)

        # 3.query knn using ball point
        edges_array= self.nodes_kd_tree.query_ball_point(skel_pixel,radius)

        # 4. initialize point marker
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

        
        # 5. adding node 1 by 1
        self.graph = nx.Graph() # create emtpy undirected graph
        for index,item in enumerate(skel_pixel):
            self.graph.add_node(index,x=item[0],y=item[1])
            point = Point()
            point.x = item[0]
            point.y = item[1]
            point.z = 0.1
            node_marker.points.append(point)


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

        # 6. adding edge 1 by 1
        for index,outer_item in enumerate(edges_array):

            start_point = Point()
            start_point.x = skel_pixel[index][0]
            start_point.y = skel_pixel[index][1]
            start_point.z = 0.1

            for inner_item in outer_item:
                self.graph.add_edge(index,inner_item)

                end_point = Point()
                end_point.x = skel_pixel[inner_item][0]
                end_point.y = skel_pixel[inner_item][1]
                end_point.z = 0.1

                edge_marker.points.append(start_point)
                edge_marker.points.append(end_point)
        self.edge_marker_pub.publish(edge_marker)

        self.map_pub.publish(self.occ_map.skel_map)

        self.query_shortest_path([],[])

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
        print(arr)

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
            

if __name__ == '__main__':
    rospy.init_node('prm')
    node = PRM()

    # if node.occ_map is None:
    #     rospy.loginfo("no map receive")
    #     rospy.sleep(1)

    # skel = node.occ_map.skel
    # distance = ndimage.distance_transform_edt(invert(skel))
    # # print(distance)
    # # plt.imshow(distance, cmap='gray_r', origin='lower', vmin = 0, vmax = 50, extent=[node.occ_map.origin_x,node.occ_map.max_x,node.occ_map.origin_y,node.occ_map.max_y])

    # # plt.show()
    # node.sampling()
    # # rate = rospy.Rate(5)
    rospy.spin()

