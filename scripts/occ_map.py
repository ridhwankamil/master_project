#!/usr/bin/env python3

import numpy as np
import scipy.ndimage.morphology as morph
from skimage.morphology import skeletonize
from skimage.util import invert
from math import ceil

from matplotlib import pyplot as plt

class OccMap():
    def __init__(self, occ_map:dict,rr:float) -> None:
        """generate Occupancy map object from map dict from ros messages
        occ_map : Converted version of ros OccupancyGrid msgs in dictionary type
        rr : robot radius in metre
        """

        #initialize & generate map
        self.init_map(occ_map,rr)
        self.skel = skeletonize(invert(self.map_array))
        # print(self.skel)


    def init_map(self, occ_map:dict,rr:float):
        #obtain all info about map
        self.width = occ_map['info']['width']
        self.height = occ_map['info']['height']
        self.resolution = occ_map['info']['resolution']
        self.origin_x = occ_map['info']['origin']['position']['x']
        self.origin_y = occ_map['info']['origin']['position']['y']

        #convert map into binary numpy array and substitute unknown value (-1) as obstacle
        self.map_array = np.array(occ_map['data'])
        self.map_array[self.map_array == -1] = 1
        self.map_array[self.map_array == 100] = 1

        #shape the map array into 2D
        self.map_array.shape = (self.height,self.width)
        self.max_x = self.origin_x+(self.width*self.resolution)
        self.max_y = self.origin_y+(self.height*self.resolution)

        # print(self.map_array)
        self.inflate(rr)

        # self.plot = plt.imshow(self.skel, cmap='gray_r', origin='lower', vmin = 0, vmax = 1, extent=[self.origin_x,self.max_x,self.origin_y,self.max_y])
    
        # plt.show()

    def init_skel():
        pass


    def inflate(self,rr:float):
        """inflate the occupancy map according to robot radius
        rr = robot radius in m
        """

        #find out need to to how many binary dilation depends on robot radius
        step = rr/self.resolution
        step = ceil(step)

        for i in range(step):
            self.map_array = morph.binary_dilation(self.map_array)

    def check_occupancy(self,xy,opt='global'):
        if opt == 'global':
            rowcol = self.world_to_grid(xy)
        else:
            rowcol = xy
        
        # check for occupancy
        if self.map_array[rowcol[0],rowcol[1]] == 0:
            return True
        else:
            return False

    def show(self):
        """ PLot Occupancy map using matplotlib"""
        # plt.imshow(self.map_array, cmap='gray_r', origin='lower', vmin = 0, vmax = 1, extent=[self.origin_x,self.max_x,self.origin_y,self.max_y])
        # plt.show()

    def world_to_grid(self,xy):
        """convert x y  to grid row col"""
        x_grid = ((xy[0] + self.resolution/2) - self.origin_x)/self.resolution
        y_grid = ((xy[1] + self.resolution/2) - self.origin_y)/self.resolution

        col = int(x_grid-1)
        row = int(y_grid-1)

        return [row,col]

    def grid_to_world(self,row,col):
        pass