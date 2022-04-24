#!/usr/bin/env python3

from operator import index
import rospy
import numpy as np
from occ_map import OccMap

import networkx as nx


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
        # subscribe to map topic
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.map_pub = rospy.Publisher('/inflated_map',OccupancyGrid, latch=True, queue_size=10)
        # self.map_pub2 = rospy.Publisher('/map2',OccupancyGrid, latch=True, queue_size=10)
        # create instacne of occ-map object
        self.occ_map = OccMap()

    def map_callback(self,map_msg):
        # convert ros messages to python dictionary maintaining same format
        map_dict = message_converter.convert_ros_message_to_dictionary(map_msg)
        # create occupancy map object
        self.occ_map.init_occ_map(map_dict)

        self.occ_map.inflate(0.3)

        map_msg = message_converter.convert_dictionary_to_ros_message('nav_msgs/OccupancyGrid',self.occ_map.get_skel_map())

        self.map_pub.publish(map_msg)



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

