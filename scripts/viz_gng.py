#!/usr/bin/env python3

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

from PIL import Image
# im = Image.fromarray(A)
# im.save("your_file.jpeg")

# from sklearn.neighbors import KDTree
#parrent class
class VIZGNG():
  def __init__(self):
    print("initializing")
    self.max_nodes = rospy.get_param('~max_nodes',500) # number of nodes to be sampled
    self.robot_radius = rospy.get_param('~robot_radius', 0.22) # the size of robot

    self.inflated_map = OccupancyMap() #original map
    self.skel_map = OccupancyMap()
    self.scaled_map = OccupancyMap() #scaled version of inflated original map
    self.skel_dist_map = OccupancyMap()


    self.inflated_map_pub = rospy.Publisher('inflated_map',OccupancyGrid, latch=True, queue_size=1)
    self.scaled_map_pub = rospy.Publisher('scaled_map',OccupancyGrid, latch=True,queue_size=2)
    self.skel_map_pub = rospy.Publisher('skel_map',OccupancyGrid, latch=True, queue_size=1)
    self.skel_dist_map_pub = rospy.Publisher('dist_map', OccupancyGrid, latch=True, queue_size=1)
    
    # subscribe to map topic
    self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
    self.done = False


  def map_callback(self,map_msg):
    #every time receive map it will resampled
    print("receiving map")

    # 1. Initialize original map every time new map receive
    self.inflated_map.init_map(map_msg)
    # self.inflated_map.inflate(self.robot_radius)
    # self.inflated_map_pub.publish(self.inflated_map.map)

    # # 2. Generate Skeletonized map
    # print("generating skeletonized map")
    # skeletonized_map = self.generate_skeletonized_map(self.inflated_map.map)
    # self.skel_map.init_map(skeletonized_map)
    # self.skel_map_pub.publish(self.skel_map.map)

    # # 3. Generate downscaled map 
    # downscaled_map = self.generate_scaled_map(self.inflated_map.map)
    # # 3. Initialize downscaled map
    # self.scaled_map.init_map(downscaled_map)
    # self.scaled_map_pub.publish(self.scaled_map.map)

    # # 4. Generate Distance-map from skel map
    # skel_dist_map,np_skel_dist_map = self.generate_skel_dist_map(self.skel_map.map)
    # self.skel_dist_map.init_map(skel_dist_map)
    # self.skel_dist_map.np_map = np_skel_dist_map
    # self.skel_dist_map_pub.publish(self.skel_dist_map.map)

    # print("\nStart training")
    # rospy.sleep(3.0)

    # 5.0 Generate GNG training data
    # train gng
    # self.gng = self.train_gng(self.scaled_map,self.max_nodes)
    # print("no. of nodes: ", self.gng.graph.n_nodes)

    # convert gng graph object to networkx graph object
    # self.graph = self.convert_gng_to_nx(self.gng)
    self.done = True

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
  
if __name__ == '__main__':
  rospy.init_node('viz_gng')
  node = VIZGNG()
  rate = rospy.Rate(1)

  while not rospy.is_shutdown():
      if node.done:
          
          origin_x = node.inflated_map.map.info.origin.position.x
          origin_y = node.inflated_map.map.info.origin.position.y
          max_x = node.inflated_map.max['x']
          max_y = node.inflated_map.max['y']


          # create 2 subplot
          # figure, axis = plt.subplots(1, 2)
          # subplot 1
          plt.imshow(node.inflated_map.np_map, cmap='gray_r',origin='lower', vmin = 0, vmax = 1, extent=[origin_x,max_x,origin_y,max_y])
          # subplot 2
          # axis[1].scatter(scatter_data[1],scatter_data[0], s=5, alpha=1)
          # show plot window
          # plt.show()
          plt.savefig('original.png')
          node.done = False


      rate.sleep()
      
  rospy.spin()



