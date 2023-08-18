#!/usr/bin/env python3

import enum
from operator import index
import copy
import rospy
import numpy as np
from occ_map import OccupancyMap
from skimage.transform import rescale
from scipy import ndimage

import networkx as nx
import math

from pprint import pprint
from neupy import algorithms, utils
from skimage.morphology import skeletonize
from skimage.util import invert


from master_project.srv import RequestPath,RequestPathRequest,RequestPathResponse

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
class GNGPLANNER():
    def __init__(self):
        self.max_nodes = rospy.get_param('~max_nodes',500) # number of nodes to be sampled
        self.robot_radius = rospy.get_param('~robot_radius', 0.2) # the size of robot
        # self.mode = rospy.get_param('~mode',0) # 


        self.nodes_kd_tree:KDTree # kd-tree for nodes array
        self.nodes_arr:np.array # nodes array

        self.skel_data_kd_tree:KDTree

        self.inflated_map = OccupancyMap() #original map
        self.skel_map = OccupancyMap()
        self.scaled_map = OccupancyMap() #scaled version of original map
        self.skel_dist_map = OccupancyMap()

        self.graph:nx.Graph = None
        self.gng:algorithms.GrowingNeuralGas = None
        self.done = False


        # subscribe to map topic
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # publisher
        self.inflated_map_pub = rospy.Publisher('inflated_map',OccupancyGrid, latch=True, queue_size=1)
        self.scaled_map_pub = rospy.Publisher('scaled_map',OccupancyGrid, latch=True,queue_size=2)
        self.skel_map_pub = rospy.Publisher('skel_map',OccupancyGrid, latch=True, queue_size=1)
        self.skel_dist_map_pub = rospy.Publisher('dist_map', OccupancyGrid, latch=True, queue_size=1)
        self.node_marker_pub = rospy.Publisher('node_marker',Marker,queue_size=10,latch=True)
        self.edge_marker_pub = rospy.Publisher('edge_marker', Marker, queue_size=10,latch=True)
        self.path_pub = rospy.Publisher('shortest_path',Path,queue_size=1,latch=True)

        self.req_path_service = rospy.Service("path_planner", RequestPath, self.request_path_callback)

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
        res.state = True
        if len(path.poses)==1:
            print('no path found')
            res.message = 'no path found'
            res.state = False

        return res

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

        # 3. Generate downscaled map 
        downscaled_map = self.generate_scaled_map(self.inflated_map.map)
        # 3. Initialize downscaled map
        self.scaled_map.init_map(downscaled_map)
        self.scaled_map_pub.publish(self.scaled_map.map)

        # 4. Generate Distance-map from skel map
        skel_dist_map,np_skel_dist_map = self.generate_skel_dist_map(self.skel_map.map)
        self.skel_dist_map.init_map(skel_dist_map)
        self.skel_dist_map.np_map = np_skel_dist_map
        self.skel_dist_map_pub.publish(self.skel_dist_map.map)

        print("\nStart training")
        rospy.sleep(3.0)

        # 5.0 Generate GNG training data
        # train gng
        self.gng = self.train_gng(self.scaled_map,self.max_nodes)
        print("no. of nodes: ", self.gng.graph.n_nodes)

        # convert gng graph object to networkx graph object
        self.graph = self.convert_gng_to_nx(self.gng)

        # extract array of nodes from nx graph
        self.nodes_arr = self.extract_node_from_nx(self.graph)
        pprint(self.nodes_arr)
        print("done print array")

        #generate kd-tree from array of node
        self.nodes_kd_tree = KDTree(self.nodes_arr)
        
        # publish all marker for rviz visualization
        node_marker = self.generate_node_marker()
        edge_marker = self.generate_edge_marker()
        node.node_marker_pub.publish(node_marker)
        node.edge_marker_pub.publish(edge_marker)

        # query dummy path data
        # path = self.query_shortest_path([],[])

        # self.path_pub.publish(path)

        self.done = True
        print('GNG Finished learning')

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
        
    def generate_scaled_map(self,map:OccupancyGrid,scale_factor=0.1):
        # 1.0 copy originalmap object
        scaled_map = copy.deepcopy(map)
        
        # 2.0 create numpy array of map data
        np_map = np.array(scaled_map.data)
        # 2.1 convert into binary image 
        np_map[np_map == -1] = 1
        np_map[np_map == 100] = 1
        # 2.2 convert numpy array to boolean type
        np_map = np_map.astype(bool)
        # 2.3 change the shape of numpy array into 2d
        np_map.shape = (scaled_map.info.height,scaled_map.info.width)
 

        # 3.0 rescale the numpy array
        np_map_scaled:np.ndarray = rescale(np_map,scale_factor,anti_aliasing=False)


        # 4.0 substitute scaled map info with new scaled version
        scaled_map.info.height = np_map_scaled.shape[0]
        scaled_map.info.width = np_map_scaled.shape[1]
        scaled_map.info.resolution = map.info.resolution * (1/scale_factor)
        
        
        # 4.1 generate map data array from numpy map array
        temp_np_map = np_map_scaled.astype(np.uint8)
        temp_np_map[temp_np_map == 1] = 100

        scaled_map.data = temp_np_map.flatten().tolist()

        return scaled_map

    def extract_node_from_nx(self,nxgraph:nx.Graph):
        node_arr = np.array(nxgraph.nodes(data=True))
        return np.array([(e[1]['pos']) for e in node_arr])

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

        dists, indexes = self.nodes_kd_tree.query(start,k=3,distance_upper_bound=radius)
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

        path= Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()

        try:
            if custom_weight_function:
                waypoint = nx.dijkstra_path(self.graph,"start","goal",custom_weight_func)
            else:
                waypoint = nx.dijkstra_path(self.graph,"start","goal",weight_func)

            for item in waypoint:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                data = self.graph.nodes[item]
                # print(data)
                pose_stamped.pose.position.x = data['pos'][0]
                pose_stamped.pose.position.y = data['pos'][1]
                pose_stamped.pose.orientation.w = 1.0

                path.poses.append(pose_stamped)

        except nx.NetworkXNoPath as err:
            print(err)

        self.graph.remove_node('start')
        self.graph.remove_node('goal')


        return path

    def generate_gng_train_data(self):
        # 1. take numpy scaled array and generate data
        data = self.map_to_data(self.scaled_map.np_map)
        data_xy = np.apply_along_axis(self.scaled_map.grid_to_world,1,data)
        print(data_xy.shape)

        skel_data = self.map_to_data(invert(self.skel_map.np_map))
        skel_data_xy = np.apply_along_axis(self.skel_map.grid_to_world,1,skel_data)
        print(skel_data_xy.shape)

        comb_data = np.concatenate((data_xy,skel_data_xy))
        print(comb_data.shape)

        return data_xy,skel_data_xy,comb_data

    def train_gng(self,scaled_map:OccupancyMap,max_nodes=500):
        
        data_xy, skel_data_xy, comb_data = self.generate_gng_train_data()

        gng = self.create_gng(max_nodes)

        print("start training")

        # learning with biased
        while gng.graph.n_nodes < max_nodes:
            gng.train(data_xy, epochs=1)
            gng.train(skel_data_xy,epochs=1)
            print(gng.graph.n_nodes)

        # gng.train(skel_data_xy,epochs=1)

        return gng

    def map_to_data(self,map):
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

    def convert_gng_to_nx(self,g:algorithms.GrowingNeuralGas):
        nxgraph = nx.Graph()
        nodeid = {}
        for indx, node in enumerate(g.graph.nodes):
            nodeid[node] = indx
            # according to gng library node.weight is actually x and y pos
            nxgraph.add_node(nodeid[node], pos=(node.weight[0][0], node.weight[0][1]))
        # positions = nx.get_node_attributes(nxgraph, "pos")
        for node_1, node_2 in g.graph.edges:
            # print(nodeid[node_1])
            p1 = nxgraph.nodes[nodeid[node_1]]['pos']
            p2 = nxgraph.nodes[nodeid[node_2]]['pos']
            if self.inflated_map.check_collision(p1,p2):
                nxgraph.add_edge(nodeid[node_1], nodeid[node_2])
        return nxgraph

    def testing_function(self):
        # 1. initialize node marker
        node_marker = Marker()
        node_marker.type = 8
        node_marker.ns = 'test'
        node_marker.id = 5
        node_marker.header.frame_id = 'map'
        node_marker.header.stamp = rospy.Time.now()
        node_marker.lifetime = rospy.Duration(0)
        node_marker.scale.x = 0.05
        node_marker.scale.y = 0.05
        node_marker.scale.z = 0.01
        node_marker.color.a = 1.0
        node_marker.color.r = 1.0

        #find pixel pos from xy coordinate
        point = Point()
        point.x = 1.567
        point.y = 0.02
        node_marker.points.append(point)

        status,p = self.occ_map.check_occupancy([1.567,0.02])

        for item in p:
            xy = self.occ_map.grid_to_world(item)
            poi = Point()
            poi.x = xy[0]
            poi.y = xy[1]
            node_marker.points.append(poi)

        print(status)


        self.node_marker_pub.publish(node_marker)
        
if __name__ == '__main__':
    rospy.init_node('gng_roadmap_planner')
    node = GNGPLANNER()
    rate = rospy.Rate(1)

    # while not rospy.is_shutdown():
    #     if node.done:
    #         # node.testing_function()
    #         # print('GNG Finished learning')
    #         # print("gng graph: ",node.gng)
    #         # node.draw_image(node.gng.graph,show=True)
    #         # node.done = False


    #     rate.sleep()
        
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