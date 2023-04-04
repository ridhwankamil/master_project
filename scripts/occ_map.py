#!/usr/bin/env python3

import numpy as np
import scipy.ndimage.morphology as morph
from skimage.morphology import skeletonize
from skimage.util import invert
import networkx as nx
from math import ceil,floor,dist
from rospy_message_converter import message_converter

# ros import
from nav_msgs.msg import OccupancyGrid

from matplotlib import pyplot as plt

class OccupancyMap():
    def __init__(self,map:OccupancyGrid = None) -> None:
        """Create OccupancyMap object"""
        self.map:OccupancyGrid = None 

        self.max = {} #maximum coordinate for both axis

        self.np_map = None #mapdata in numpy array, dtype = bool

    def init_map(self, map_msg:OccupancyGrid):
        """Initialize the original map"""
        # 1.0 save the OccupancyGrid map to internal object
        self.map= map_msg

        # 1.1 generate max value for easier usage later on
        self.max['x'] = self.map.info.origin.position.x + (self.map.info.width*self.map.info.resolution)
        self.max['y'] =  self.map.info.origin.position.y + (self.map.info.height*self.map.info.resolution)

        # 1.2 convert map into binary numpy array and substitute unknown value (-1) as obstacle
        self.np_map = np.array(self.map.data, dtype=np.uint8)
        self.np_map[self.np_map == -1] = 1
        self.np_map[self.np_map == 100] = 1

        # 1.3 Origialdata is in uint8 type convert to boolean
        self.np_map = self.np_map.astype(bool)

        #shape the map array into 2D
        self.np_map.shape = (self.map.info.height,self.map.info.width)

    # def init_skel_map(self) -> None:
    #     """ Initialize skeletonized map"""

    #     # check if map data is initialize
    #     if self.map is None or self.np_map is None:
    #         print("Please initialize map first")
    #         return

    #     # 1st create a copy of Original OccupancyGrid map object
    #     self.skel_map = deepcopy(self.map)

    #     #2nd create skeletonized numpy array
    #     self.np_skel_map = skeletonize(invert(self.np_map))

    #     #3rd convert 1 to 100
    #     temp_map = np.array(self.np_skel_map, dtype=np.uint8)
    #     temp_map[temp_map == 1] = 100

    #     #4th save into skel_map object
    #     self.skel_map.data = temp_map.flatten().tolist()

    def inflate(self,rr:float):
        """inflate the occupancy map according to robot radius
        rr = robot radius in m
        """
        #find out need to to how many binary dilation depends on robot radius
        step = rr/self.map.info.resolution
        # print("step: ", step)
        step = ceil(step)

        for i in range(step):
            self.np_map = morph.binary_dilation(self.np_map)
        
        temp_map = self.np_map.astype(np.uint8)
        temp_map[temp_map == 1] = 100

        self.map.data = temp_map.flatten().tolist()

    def check_pixel_value(self,xy):
        """ Check if the space is occupied with obstacle
        Return True if free
        return False if occupied"""
        # 1.0 convert xy to rowcol
        row_col = self.world_to_grid(xy)
        # print("row_col: ",row_col)

        row = round(row_col[0])
        col = round(row_col[1])
        # print(self.np_map)

        return self.np_map[row,col]

    def check_occupancy(self,xy):
        """ Check if the space is occupied with obstacle
        Return True if free
        return False if occupied"""
        # 1.0 convert xy to rowcol
        row_col = self.world_to_grid(xy)
        # print("row_col: ",row_col)

        row_low_bound = floor(row_col[0])
        row_upp_bound = ceil(row_col[0])
        col_low_bound = floor(row_col[1])
        col_upp_bound = ceil(row_col[1])

        check_points = [[row_low_bound,col_low_bound],[row_low_bound,col_upp_bound],[row_upp_bound,col_low_bound],[row_upp_bound,col_upp_bound]]

        for item in check_points:
        # check for occupancy
            if self.np_map[item[0],item[1]] == 1:
                return False
            else:
                pass
        return True

    def check_collision(self,p1,p2)-> bool:
        # find euclidean distant
        eucl_dist = dist(p1,p2)
        interval_count = ceil(eucl_dist/self.map.info.resolution)
        # print(eucl_dist)
        # print(interval_count)
        if interval_count < 1:
            return self.check_occupancy(p2)
        for t in np.arange(0.0,1.0,1/interval_count):
            p = self.lerp(p1,p2,t)
            if not self.check_occupancy(p):
                # print('found obstacle')
                return False
        # lstly check end point
        return self.check_occupancy(p2)

    def lerp(self,A,B,t):
        x = A[0] + (B[0] - A[0]) * t
        y = A[1] + (B[1] - A[1]) * t
        return [x,y]

    def world_to_grid(self,xy):
        """convert x y  to grid row col
        converted value might be floating point"""
        sub = self.map.info.resolution/2

        col = ((xy[0]-sub) - self.map.info.origin.position.x)/self.map.info.resolution
        row= ((xy[1]-sub) - self.map.info.origin.position.y)/self.map.info.resolution

        return np.array([row,col])

    def grid_to_world(self,row_col):
        """convert row col to xy coordinate"""
        add = self.map.info.resolution/2   
        ros_y = row_col[0] * self.map.info.resolution + self.map.info.origin.position.y
        ros_x = row_col[1] * self.map.info.resolution + self.map.info.origin.position.x
        return np.array([ros_x+add,ros_y+add])
