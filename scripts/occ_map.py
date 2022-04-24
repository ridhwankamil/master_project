#!/usr/bin/env python3

import numpy as np
import scipy.ndimage.morphology as morph
from skimage.morphology import skeletonize
from skimage.util import invert
from math import ceil
from rospy_message_converter import message_converter

from matplotlib import pyplot as plt

class OccMap():
    def __init__(self) -> None:
        """generate Occupancy map object from map dict from ros messages
        occ_map : Converted version of ros OccupancyGrid msgs in dictionary type
        rr : robot radius in metre
        """
        self.occ_map = None 
        self.skel_map = None
        self.skel_dist_map = None

        self.width = None # the width of map
        self.height = None # the heiight of map
        self.resolution = None #resolution of map in metre/grid
        self.origin = None #origin at lower left side
        self.max = None #maximum coordinate for both axis

        self.np_occ_map = None #mapdata in numpy array
        self.np_skel_map = None
        self.np_skel_dist_map = None

        #initialize & generate map
        # self.init_occ_map(occ_map)
        # #inflate map 
        # self.inflate(rr)
        # self.skel = skeletonize(invert(self.map_array))
        # print(self.skel)

    def init_occ_map(self, occ_map:dict):
        self.occ_map = occ_map.copy() #make a copy instead

        #obtain all info about map
        self.width = occ_map['info']['width']
        self.height = occ_map['info']['height']
        self.resolution = occ_map['info']['resolution']
        self.origin= {'x':occ_map['info']['origin']['position']['x'], 'y':occ_map['info']['origin']['position']['y']}
        self.max = {'x':self.origin['x'] + (self.width*self.resolution), 'y': self.origin['y'] + (self.height*self.resolution)}

        #convert map into binary numpy array and substitute unknown value (-1) as obstacle
        self.np_occ_map = np.array(occ_map['data'])
        self.np_occ_map[self.np_occ_map == -1] = 1
        self.np_occ_map[self.np_occ_map == 100] = 1

        #shape the map array into 2D
        self.np_occ_map.shape = (self.height,self.width)

        # convert numpy to list back to save inside of dict for later to be converted to ros msg using message onverter
        self.occ_map['data'] = self.np_occ_map.flatten().tolist()   

        # print(self.map_array)
        # self.plot = plt.imshow(self.skel, cmap='gray_r', origin='lower', vmin = 0, vmax = 1, extent=[self.origin_x,self.max_x,self.origin_y,self.max_y])
        # plt.show()

    # def init_skel():
    #     pass

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

    # def grid_to_world(self,row,col):
    #     pass