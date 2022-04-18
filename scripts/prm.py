#!/usr/bin/env python3

from operator import index
import rospy
import numpy as np
from occ_map import OccMap
import random
import time
import math
import csv
import networkx as nx
from skimage.util import invert




from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import Point, PoseStamped
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray
from matplotlib import pyplot as plt

from scipy.spatial import KDTree
from scipy import ndimage
# from sklearn.neighbors import KDTree

#parrent class
class PRM():
    def __init__(self):
        self.animation = True


        self.rr = 0.5 # robot radius
        self.n =2000 # number of sampled
        self.max_connection = 5 # max connection per sampled 
        self.max_dist = 3.0 # max distance for edges between nodes
        self.occ_map = None # varibale store the occupancy grid map
        self.nodes = [] # variable to store the list of nodes
        self.edges = [] # varibale tp store the list of edges
        self.g = nx.Graph() #init networkx graph object

        # subscribe to map topic
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # publish node & edge marker topic
        self.node_marker_pub = rospy.Publisher('/node_marker',Marker,queue_size=10,latch=True)
        self.edge_marker_pub = rospy.Publisher('/edge_marker',Marker,queue_size=10, latch=True)

        # publish path using conventional dijkstra method
        self.path_pub = rospy.Publisher('/path',Path,queue_size=10, latch=True)
        # publish path using proposed method
        self.path_weight_pub = rospy.Publisher('/path_weight',Path,queue_size=10, latch=True)

        #subscribe to goal topic
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.goal_cb)


    def map_callback(self,map_msg):
        # convert ros messages to python dictionary maintaining same format
        map_dict = message_converter.convert_ros_message_to_dictionary(map_msg)
        # create occupancy map object
        self.occ_map = OccMap(map_dict,self.rr)

        # self.skel = self.occ_map.skel
        # plt.imshow(self.skel, cmap='gray_r', origin='lower', vmin = 0, vmax = 1, extent=[self.occ_map.origin_x,self.occ_map.max_x,self.occ_map.origin_y,self.occ_map.max_y])
    
        # plt.show()


        # self.sampling()
        # self.query([0.0,0.0],[-5.0,3.0])

    def goal_cb(self,msg):
        goal = [msg.pose.position.x, msg.pose.position.y]

        self.query([0.0,0.0],goal) # hardcoded the initial pose for queries

    def query(self,start,goal):
        print(goal)
        self.g.add_node(self.n+1,x=start[0],y=start[1])
        self.g.add_node(self.n+2,x=goal[0],y=goal[1])

        start_edge = []
        goal_edge = []
        dists, indexes = self.nodes_kd_tree.query(start, k=self.n, distance_upper_bound=self.max_dist)
        for i in range(len(indexes)):
            # print(self.nodes[indexes[i]])
            if(indexes[i] == self.n):
                break 
            if(self.check_edge_collision(start,self.nodes[indexes[i]])):
                start_edge.append(indexes[i])
            if len(start_edge) >= self.max_connection:
                break
            
        for ii in range(len(start_edge)):
            if(start_edge[ii] == self.n):
                break
            self.g.add_edge(self.n+1,start_edge[ii])

        dists, indexes = self.nodes_kd_tree.query(goal, k=self.n, distance_upper_bound=self.max_dist)
        for i in range(len(indexes)):
            if(indexes[i] == self.n):
                break 
            if(self.check_edge_collision(goal,self.nodes[indexes[i]])):
                goal_edge.append(indexes[i])
            if len(goal_edge) >= self.max_connection:
                break
            
        for ii in range(len(goal_edge)):
            if(goal_edge[ii] == self.n):
                break
            self.g.add_edge(self.n+2,goal_edge[ii])


        waypoint = nx.shortest_path(self.g, source=self.n+1, target=self.n+2)
        waypoint_weight = nx.dijkstra_path(self.g, source=self.n+1, target=self.n+2, weight='weight')
        # print(waypoint)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()

        path_weight = Path()
        path_weight.header.frame_id = 'map'
        path_weight.header.stamp = rospy.Time.now()


        for item in waypoint:

            pose = PoseStamped()
            pose.pose.position.x = self.g.nodes[item]['x']
            pose.pose.position.y = self.g.nodes[item]['y']
            pose.pose.position.z = 0.1


            path.poses.append(pose)

        self.path_pub.publish(path)

        for item in waypoint_weight:

            pose = PoseStamped()
            pose.pose.position.x = self.g.nodes[item]['x']
            pose.pose.position.y = self.g.nodes[item]['y']
            pose.pose.position.z = 0.1


            path_weight.poses.append(pose)

        self.path_pub.publish(path)
        self.path_weight_pub.publish(path_weight)

        self.g.remove_node(self.n+1)
        self.g.remove_node(self.n+2)

    def sampling(self):
        if self.occ_map == None:
            print('map has not been initialized')
            return

        # empty up the nodes list
        self.nodes = []
        
        # initialize point marker
        node_marker = Marker()
        node_marker.type = 8
        node_marker.ns = 'nodes'
        node_marker.id = 0
        node_marker.header.frame_id = 'map'
        node_marker.header.stamp = rospy.Time.now()
        node_marker.lifetime = rospy.Duration(0)
        node_marker.scale.x = 0.05
        node_marker.scale.y = 0.05
        node_marker.scale.z = 0.05
        node_marker.color.a = 1.0
        node_marker.color.r = 1.0
        
        # node_marker.pose.orientation.w = 1.0

        # plt.imshow(self.occ_map.map_array, cmap='gray_r', origin='lower', vmin = 0, vmax = 1, extent=[self.occ_map.origin_x,self.occ_map.max_x,self.occ_map.origin_y,self.occ_map.max_y])
        
        #sampling node
        while len(self.nodes)<self.n:
            # generate random point based on axis origin & axis max
            x_rand = random.uniform(self.occ_map.origin_x, self.occ_map.max_x)
            y_rand = random.uniform(self.occ_map.origin_y, self.occ_map.max_y)

            if self.occ_map.check_occupancy([x_rand,y_rand]):
                self.nodes.append([x_rand,y_rand])
                print(len(self.nodes))

                if self.animation:
                    # create point marker, append to marker object & publish
                    point = Point()
                    point.x = x_rand
                    point.y = y_rand
                    point.z = 0.1
                    node_marker.points.append(point)
                    self.node_marker_pub.publish(node_marker)
                
            else:
                # print('occupied')
                pass


        self.edges = []

        edge_marker = Marker()
        edge_marker.type = 5
        edge_marker.ns = 'edges'
        edge_marker.id = 5
        edge_marker.header.frame_id = 'map'
        edge_marker.header.stamp = rospy.Time.now()
        edge_marker.lifetime = rospy.Duration(0)
        edge_marker.scale.x = 0.005

        edge_marker.color.a = 0.5
        edge_marker.color.b = 1.0
        edge_marker.pose.orientation.w = 1.0

        # generate edges from nearest neighbour
        # 1. create kd tree from nodes array
        self.nodes_kd_tree = KDTree(np.array(self.nodes))

        # 2. interate every nodes inside nodes list and use knn to find nearest neighbour
        for item in self.nodes:
            dists, indexes = self.nodes_kd_tree.query(item, k=self.n, distance_upper_bound=self.max_dist)
            edge_id = []
 
            for ii in range(1,len(indexes)):
                # check edge collision
                
                # this is just some workaround becoz query will return self.n for the balance
                if(indexes[ii] == self.n):
                    break
                
                if(self.check_edge_collision(self.nodes[indexes[0]],self.nodes[indexes[ii]])):

                    edge_id.append(indexes[ii])
                    print(indexes[ii])

                    if self.animation:
                        # create point marker, append to marker object & publish
                        source_point = Point()
                        source_point.x = self.nodes[indexes[0]][0]
                        
                        source_point.y = self.nodes[indexes[0]][1]
                        source_point.z = 0.01

                        target_point = Point()
                        target_point.x = self.nodes[indexes[ii]][0]
                        target_point.y = self.nodes[indexes[ii]][1]
                        target_point.z = 0.01

                        edge_marker.points.append(source_point)
                        edge_marker.points.append(target_point)
                        self.edge_marker_pub.publish(edge_marker)

                if len(edge_id) >= self.max_connection:
                    break
            
            self.edges.append(edge_id)

        # print(self.edges)

        # 3. create graph object
        for i in range(len(self.nodes)):
            self.g.add_node(i,x=self.nodes[i][0],y=self.nodes[i][1],node_weight=1)

        for i in range(len(self.edges)):
            for j in range(len(self.edges[i])):
                weight = self.calc_eucl_dist(self.nodes[i],self.nodes[self.edges[i][j]])
                self.g.add_edge(i,self.edges[i][j],weight=weight)
        print('done')
        with open('nodes.csv', 'w') as f:
            write = csv.writer(f)
            write.writerows(self.nodes)

        with open('edges.csv', 'w') as f:
            write = csv.writer(f)
            write.writerows(self.edges)
        # plt.show()

    def check_edge_collision(self, source, target) -> bool:
        # print('receive')
        edge_marker = Marker()
        edge_marker.type = 8
        edge_marker.ns = 'ed'
        edge_marker.id = 10
        edge_marker.header.frame_id = 'map'
        edge_marker.header.stamp = rospy.Time.now()
        edge_marker.lifetime = rospy.Duration(0)
        edge_marker.scale.x = 0.1
        edge_marker.scale.y = 0.1
        edge_marker.scale.z = 0.1

        edge_marker.color.a = 1.0
        edge_marker.color.b = 1.0
        edge_marker.pose.orientation.w = 1.0

        # 1. find interval between point
        dist = self.calc_eucl_dist(source,target)
        interval_count = round(dist/self.occ_map.resolution)
        # print(interval_count)
        
        if interval_count < 1:
            return True

        else:
            # 2. incrementally check along edge line for occupancy checking

            for t in np.arange(0.0,1.0,1/interval_count):
                point = self.vector_lerp(source,target,t)
                # print(t)
                # time.sleep(2)
                target_point = Point()
                target_point.x = point[0]
                target_point.y = point[1]
                target_point.z = 0.1

                edge_marker.points.append(target_point)


                if(self.occ_map.check_occupancy(point)):
                    # print('free') 
                    continue
                else: 
                    # print('occupied') 
                    return False
            return True
                
        

    def vector_lerp(self,A,B,t):
        x = A[0] + (B[0] - A[0]) * t
        y = A[1] + (B[1] - A[1]) * t

        return [x,y]

    def calc_eucl_dist(self,A,B):
        return math.sqrt(abs((B[0]-A[0]) + (B[1]-A[1])))

if __name__ == '__main__':
    rospy.init_node('prm')
    node = PRM()
    if node.occ_map is None:
        rospy.loginfo("no map receive")
        rospy.sleep(1)

    skel = node.occ_map.skel
    distance = ndimage.distance_transform_edt(invert(skel))
    # print(distance)
    # plt.imshow(distance, cmap='gray_r', origin='lower', vmin = 0, vmax = 50, extent=[node.occ_map.origin_x,node.occ_map.max_x,node.occ_map.origin_y,node.occ_map.max_y])

    # plt.show()
    node.sampling()
    # rate = rospy.Rate(5)
    rospy.spin()

