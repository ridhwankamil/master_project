#!/usr/bin/env python3

import rospy
import math
import numpy as np
import traceback
import copy
from skimage.util import invert
from skimage.transform import rescale
from master_project.srv import RequestPath,RequestPathRequest,RequestPathResponse
# from tf2_ros import Buffer,TransformListener
import tf2_ros
# import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from move_base_msgs.msg import MoveBaseAction,MoveBaseFeedback,MoveBaseGoal,MoveBaseResult,MoveBaseActionGoal
from mbf_msgs.msg import ExePathAction,ExePathGoal,ExePathFeedback,ExePathResult

from geometry_msgs.msg import Pose,Point,PoseStamped,TransformStamped

from actionlib import SimpleActionServer,SimpleActionClient

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from occ_map import OccupancyMap

from scipy.spatial import KDTree

class NavigationServer:
    def __init__(self,name):
        # self.path_planner_ser_name = '/gng/path_planner'
        self.path_planner_ser_name = '/gng_biased/path_planner'

        # 1. subscribe to lidar
        self.lidar_sub = rospy.Subscriber('/scan_multi',LaserScan,self.scan_cb)
        # store incoming laser scan msg internally
        self.laser_scan:LaserScan = None

        # temp
        self.map_pub = rospy.Publisher('pub_map',OccupancyGrid,queue_size=10,latch=True)
        self.obs_marker_pub = rospy.Publisher('obs_marker',Marker,queue_size=100,latch=True)

        self.obstacle_kd_tree:KDTree # kd-tree for nodes array

        # 2. subscribe to map 
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid,self.map_cb)
        self.map = OccupancyMap() #original map
        self.map_done = False # this is true after have receive map

        # 3. Local Planner client
        self.controller_client = SimpleActionClient('/move_base_flex/exe_path',ExePathAction)

        # 4. setup tf2 buffer
        # init tf buffer & listener
        self.tfBuffer:tf2_ros.Buffer = tf2_ros.Buffer()
        self.tfListener:tf2_ros.TransformListener = tf2_ros.TransformListener(self.tfBuffer)

        # action server
        # self.action_name = name
        # self.action_feedback = MoveBaseFeedback()
        # self.action_result = MoveBaseResult()
        # self.action_server = SimpleActionServer(self.action_name,MoveBaseAction,self.navigate_callback,auto_start=False)
        # self.action_server.start()


        self.action_client = SimpleActionClient("/navigation_server",MoveBaseAction)
        self.action_client_goal = None
        self.new_goal = False
        
        rospy.loginfo("Navigation server is active")


    def send_goal(self):
        self.new_goal = False
        if self.action_client.wait_for_server(rospy.Duration(3.0)):
            print("action available")
            self.action_client.send_goal(goal=self.action_client_goal)

            # self.action_client.wait_for_result()
            print("done")
            
        else:
            print("action not available")
        return

    def navigate_callback(self,goal:MoveBaseGoal=MoveBaseGoal()):
        # 1 create Path planner service client
        # check if planner service is available
        start_pose:PoseStamped = PoseStamped()
        try:
            rospy.wait_for_service(self.path_planner_ser_name,3)
            rospy.loginfo("service available")
            req = RequestPathRequest()
            # uncomment if use real robot
            # transform:TransformStamped = self.tfBuffer.lookup_transform('map','base_footprint',rospy.Time.now(),rospy.Duration(2.0))
            # start_pose.header = transform.header
            # start_pose.pose.orientation.w = 1.0
            # start_pose = do_transform_pose(start_pose,transform)
            
            # req.start = start_pose
            path_requester = rospy.ServiceProxy(self.path_planner_ser_name,RequestPath)
            # req.start.header.frame_id = 'map'
            # req.start.header.stamp = rospy.Time.now()
            req.start.pose.position.x = 0.0
            req.start.pose.position.y = 0.0
            req.start.pose.position.y = 0.0
            req.start.pose.orientation.z = 0.0
            req.start.pose.orientation.w = 1.0

            req.goal.pose.position.x = -15.106101989746094
            req.goal.pose.position.y = 5.07640266418457
            req.goal.pose.orientation.z = 0.9999733361485715
            req.goal.pose.orientation.w = 0.007302533251955163
            # req.goal = goal.target_pose
            req.custom_weight_function = False
            print("requesting path planner service for shortest path")
            # requesting path planner service for shortest path
            res:RequestPathResponse = path_requester.call(req)
            print(res)
            if not res.state:
                rospy.logerr(res.message)
                # self.action_server.set_aborted()
                return
            
            # 2. after receiving shortest path, interpolated the path to get smooth path
            print("interpolating waypoint into path")
            interpolated_path = Path()

            for index in range(len(res.path.poses)-2):
                temp_path = self.interpolate_straight(res.path.poses[index],res.path.poses[index+1])
                for item in temp_path:
                    interpolated_path.poses.append(item)
            
            interpolated_path.poses.append(res.path.poses[-1])
    
            # 3. execute local planner to follow trajectory
            if self.controller_client.wait_for_server(rospy.Duration(3.0)):
                rospy.loginfo("exe_path action server available")
                done = False
                rate = rospy.Rate(100)

                def done_cb(first,second):
                    nonlocal done
                    print(type(first))
                    print(type(second))
                    print(done)
                controller_goal = ExePathGoal()
                
                controller_goal.path = interpolated_path
                print(controller_goal.path.poses[2].header.frame_id)
                self.controller_client.send_goal(controller_goal,done_cb=done_cb)

                while not done:
                    # this is loop when local planner is running

                    rate.sleep()

                print("out of while loop")
            else:
                rospy.logerr("exe_path action server is not running")
                return
            
        except rospy.ROSException:
            rospy.logerr("service not available")
            return
        except rospy.ServiceException:
            rospy.logerr("problem comunicate with service")
            return
        except Exception as err:
            rospy.logerr("other error occur when calling path planner service")
            print(traceback.format_exc())
            return
        
    def interpolate_straight(self,source:PoseStamped,target:PoseStamped):
        injection_pose_list = []
        #measure distance to from start to target
        dist = self.calc_eucl_dist(source,target)
        print("distance: ", dist)

        #from distance we estimate interval count between source to target with 0.01 metre interval
        injection_count = round(dist/0.005)

        for t in np.arange(0.0,1.0,1/injection_count):
            pose:PoseStamped = self.pose_interpolate(source,target,t)
            injection_pose_list.append(pose)

        return injection_pose_list 
    
    def pose_interpolate(self,source:PoseStamped,target:PoseStamped,t):

        source_x = source.pose.position.x
        source_y = source.pose.position.y

        target_x = target.pose.position.x
        target_y = target.pose.position.y


        x = source_x + (target_x - source_x) * t
        y = source_y + (target_y - source_y) * t

        pose = PoseStamped()
        pose.header = source.header
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = x
        pose.pose.position.y = y

        return pose  

    def calc_eucl_dist(self,source:PoseStamped,target:PoseStamped):
        x0 = source.pose.position.x
        x1 = target.pose.position.x

        y0 = source.pose.position.y
        y1 = target.pose.position.y

        sum = math.sqrt(math.pow(x1-x0,2) + math.pow(y1-y0,2))
        # sum= math.sqrt(abs((B[0]-A[0]) + (B[1]-A[1])))
        return sum    

    def scan_cb(self,msg:LaserScan):
        self.laser_scan = msg

    def map_cb(self,msg:OccupancyGrid):
        print("map_callback")
        binary_map = self.convert_to_binary_map(msg)
        self.map.init_map(binary_map)
        obstacle_data = self.map_to_data(invert(self.map.np_map))
        obstacle_data_xy = np.apply_along_axis(self.map.grid_to_world,1,obstacle_data)

        # print("length of obstacle data: ", len(obstacle_data_xy))
        self.publish_obs_marker(obstacle_data_xy)

        # generate obstacle kd tree
        self.obstacle_kd_tree = KDTree(obstacle_data_xy,copy_data=True)

        # dists, indexes = self.obstacle_kd_tree.query([0.0,0.0],10)
        self.map_pub.publish(self.map.map)
        print("done map callback")
        self.map_done = True

    def publish_obs_marker(self,obstacle_data_xy):
        # 1. initialize node marker
        node_marker = Marker()
        node_marker.type = 8
        node_marker.ns = 'obstacle'
        node_marker.id = 15
        node_marker.header.frame_id = 'map'
        node_marker.header.stamp = rospy.Time.now()
        node_marker.lifetime = rospy.Duration(0)
        node_marker.scale.x = 0.1
        node_marker.scale.y = 0.1
        node_marker.scale.z = 0.1
        node_marker.color.a = 1.0
        node_marker.color.r = 1.0

        # 2. iterate obstacle 1 by 1 & append to obstacle marker
        print("iterating")
        for item in obstacle_data_xy:
            # print(item)
            point = Point()
            point.x = item[0]
            point.y = item[1]
            node_marker.points.append(point)

        print("publishing obstacle marker")
        self.obs_marker_pub.publish(node_marker)


    def convert_to_binary_map(self,map:OccupancyGrid,scale_factor=0.1):
        # 1.0 copy originalmap object
        binary_map = copy.deepcopy(map)
        # print(type(binary_map))
        
        # 2.0 create numpy array of map data
        np_map = np.array(binary_map.data)
        # 2.1 convert into binary image 
        np_map[np_map == -1] = 1
        np_map[np_map == 100] = 1
        # 2.2 convert numpy array to boolean type
        np_map = np_map.astype(bool)
        # 2.3 change the shape of numpy array into 2d
        np_map.shape = (binary_map.info.height,binary_map.info.width)
 
        
        
        # 4.1 generate map data array from numpy map array
        temp_np_map = np_map.astype(np.uint8)
        temp_np_map[temp_np_map == 1] = 100

        binary_map.data = temp_np_map.flatten().tolist()

        return binary_map

    def map_to_data(self,map):
        data = []
        for (x, y), value in np.ndenumerate(map):
            if value == 0:
                data.append([x, y])
        return np.array(data)

if __name__ == '__main__':
    rospy.init_node('experiment3')
    server = NavigationServer(rospy.get_name())
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if server.map_done:
            
            server.navigate_callback()
            exit()
        rate.sleep()
    rospy.spin()