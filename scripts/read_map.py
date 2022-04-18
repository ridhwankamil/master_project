#!/usr/bin/env python3

import rospy
import numpy as np
import time
from nav_msgs.msg import OccupancyGrid
from rospy_message_converter import message_converter
from std_msgs.msg import String
import random

from matplotlib import pyplot as plt

#parrent class
class PRM():
    def __init__(self,n = 500,neighbour_dist=1.0, interval=10):
        """n = number of samples
        neighbour_dist = max connection distance between nodes
        interval = collision interval checking"""
        self.n = n
        self.neighbour_dist = neighbour_dist
        self.interval = interval

    def update_map(self,map:dict):
        #obtain all info about map
        self.width = map['info']['width']
        self.height = map['info']['height']
        self.resolution = map['info']['resolution']
        self.origin_x = map['info']['origin']['position']['x']
        self.origin_y = map['info']['origin']['position']['y']

        #convert map into binary numpy array and substitute unknown value (-1) as obstacle
        self.map_array = np.array(map['data'])
        self.map_array[self.map_array == -1] = 1
        self.map_array[self.map_array == 100] = 1

        #shape the map array into 2D
        self.map_array.shape = (self.height,self.width)
        self.max_x = self.origin_x+(self.width*self.resolution)
        self.max_y = self.origin_y+(self.height*self.resolution)


        # self.plot = plt.imshow(self.map_array, cmap='gray_r', origin='lower', vmin = 0, vmax = 1, extent=[self.origin_x,self.max_x,self.origin_y,self.max_y])
    
        #plt.show()


    def sampling(self):

        self.nodes = []
        start_time = time.time()
        while len(self.nodes) < self.n:
            x_rand = random.uniform(self.origin_x, self.max_x)
            y_rand = random.uniform(self.origin_y, self.max_y)
            print([x_rand,y_rand])
            #plt.scatter([x_rand], [y_rand])

            if self.check_occupancy(x_rand,y_rand):
                self.nodes.append([x_rand,y_rand])
                plt.scatter([x_rand], [y_rand])
                #print('free')
            else:
                #print('occupied')
                pass
        print(time.time()-start_time)
        print(len(self.nodes))
        plt.show()


    def check_occupancy(self,x,y):
        """return True if free space"""
        x_grid = (x - self.origin_x)/self.resolution
        y_grid = (y - self.origin_y)/self.resolution

        x_grid = int(x_grid)
        y_grid = int(y_grid)

        if self.map_array[y_grid,x_grid] == 0:
            return True
        else:
            return False

        
#subclass
class RoadmapPlanner(PRM):
    def __init__(self, n, neighbour_dist, interval):
        super().__init__(n=n,neighbour_dist=neighbour_dist,interval=interval)

        rospy.Subscriber('/map',OccupancyGrid,self.map_callback)

        #just to known if there any new map data and perform GNG accordingly

    def spin(self):
        pass

    def map_callback(self,map_msg):
        print("map received")
        #convert map data to dictionary
        map = message_converter.convert_ros_message_to_dictionary(map_msg)
        self.update_map(map)
        self.new_map = True
        self.sampling()

if __name__ == '__main__':
    rospy.init_node('roadmap_planner_server')
    node = RoadmapPlanner(n=200,neighbour_dist=1,interval=10)
    rate = rospy.Rate(5)
    rospy.spin()

