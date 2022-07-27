#!/usr/bin/env python3

import numpy as np
import scipy.ndimage.morphology as morph
from skimage.morphology import skeletonize
from skimage.util import invert
import networkx as nx
from math import ceil
from copy import deepcopy
from rospy_message_converter import message_converter

# ros import
from nav_msgs.msg import OccupancyGrid

from matplotlib import pyplot as plt

class OccupancyMap():
    def __init__(self,map:OccupancyGrid = None) -> None:
        """Create OccupancyMap object"""
        self.map = None 
        self.skel_map = None
        self.skel_dist_map = None

        self.max = {} #maximum coordinate for both axis

        self.np_map = None #mapdata in numpy array dtype = bool
        self.np_skel_map = None
        self.np_skel_dist_map = None

        #initialize & generate map
        # self.init_occ_map(occ_map)
        # #inflate map 
        # self.inflate(rr)
        # self.skel = skeletonize(invert(self.map_array))
        # print(self.skel)

    def init_map(self, map_msg:OccupancyGrid):
        """Initialize the original map"""
        # save the OccupancyGrid map to internal object
        self.map= map_msg

        # generate max value for easier usage later on
        self.max['x'] = self.map.info.origin.position.x + (self.map.info.width*self.map.info.resolution)
        self.max['y'] =  self.map.info.origin.position.x + (self.map.info.height*self.map.info.resolution)

        #convert map into binary numpy array and substitute unknown value (-1) as obstacle
        self.np_map = np.array(self.map.data, dtype=np.uint8)
        self.np_map[self.np_map == -1] = 1
        self.np_map[self.np_map == 100] = 1

        #Origialdata is in uint8 type convert to boolean
        self.np_map = self.np_map.astype(bool)

        #shape the map array into 2D
        self.np_map.shape = (self.map.info.height,self.map.info.width)


    def init_skel_map(self) -> None:
        """ Initialize skeletonized map"""

        # check if map data is initialize
        if self.map is None and self.np_map is None:
            print("Please initialize map first")
            return
        
        # 1st create a copy of Original OccupancyGrid map object
        self.skel_map = deepcopy(self.map)

        #2nd create skeletonized numpy array
        self.np_skel_map = skeletonize(invert(self.np_map))

        #3rd convert 1 to 100
        temp_map = np.array(self.np_skel_map, dtype=np.uint8)
        temp_map[temp_map == 1] = 100

        #4th save into skel_map object
        self.skel_map.data = temp_map.flatten().tolist()

    # def get_occ_map(self):
    #     map_dict = 




    #     return map_msg

    def get_skel_map(self):
        """return map dictionary according to ros occupany grid map message format"""
        if self.np_skel_map is None:
            raise Exception('Skel map not available, please generate it beforehand')
        # convert numpy to list back to save inside of dict for later to be converted to ros msg using message onverter
        map_array = self.np_skel_map.copy().astype(int)
        map_array[map_array == 1] = 100
        # convert numpy to list back to save inside of dict for later to be converted to ros msg using message onverter
        self.skel_map['data'] = map_array.flatten().tolist()   
        return self.occ_map

    def get_occ_map(self):
        """return map dictionary according to ros occupany grid map message format"""
        # convert numpy to list back to save inside of dict for later to be converted to ros msg using message onverter
        map_array = self.np_occ_map.copy().astype(int)
        map_array[map_array == 1] = 100
        # convert numpy to list back to save inside of dict for later to be converted to ros msg using message onverter
        self.occ_map['data'] = map_array.flatten().tolist()   
        return self.occ_map

    def inflate(self,rr:float):
        """inflate the occupancy map according to robot radius
        rr = robot radius in m
        """

        #find out need to to how many binary dilation depends on robot radius
        step = rr/self.resolution
        step = ceil(step)

        for i in range(step):
            self.np_occ_map = morph.binary_dilation(self.np_occ_map)

    # def check_occupancy(self,xy,opt='global'):
    #     if opt == 'global':
    #         rowcol = self.world_to_grid(xy)
    #     else:
    #         rowcol = xy
        
    #     # check for occupancy
    #     if self.map_array[rowcol[0],rowcol[1]] == 0:
    #         return True
    #     else:
    #         return False

    # def show(self):
    #     """ PLot Occupancy map using matplotlib"""
    #     # plt.imshow(self.map_array, cmap='gray_r', origin='lower', vmin = 0, vmax = 1, extent=[self.origin_x,self.max_x,self.origin_y,self.max_y])
    #     # plt.show()

    # def world_to_grid(self,xy):
    #     """convert x y  to grid row col"""
    #     x_grid = ((xy[0] + self.resolution/2) - self.origin_x)/self.resolution
    #     y_grid = ((xy[1] + self.resolution/2) - self.origin_y)/self.resolution

    #     col = int(x_grid-1)
    #     row = int(y_grid-1)

    #     return [row,col]

    def grid_to_world(self,row_col):
        add = self.map.info.resolution/2   
        ros_y = row_col[0] * self.map.info.resolution + self.map.info.origin.position.y
        ros_x = row_col[1] * self.map.info.resolution + self.map.info.origin.position.x
        return np.array([ros_x+add,ros_y+add])
