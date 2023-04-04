#!/usr/bin/env python3

import rospy
import math
import numpy as np
import traceback
from master_project.srv import RequestPath,RequestPathRequest,RequestPathResponse
# from tf2_ros import Buffer,TransformListener
import tf2_ros
# import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Path

from move_base_msgs.msg import MoveBaseAction,MoveBaseFeedback,MoveBaseGoal,MoveBaseResult,MoveBaseActionGoal
from mbf_msgs.msg import ExePathAction,ExePathGoal,ExePathFeedback,ExePathResult

from geometry_msgs.msg import Pose,PoseStamped,TransformStamped

from actionlib import SimpleActionServer,SimpleActionClient

class NavigationServer:
    def __init__(self,name):
        self.path_planner_ser_name = '/gng/path_planner'
        # self.path_planner_ser_name = '/gng_biased/path_planner'

        self.controller_client = SimpleActionClient('/move_base_flex/exe_path',ExePathAction)

        # init tf buffer & listener
        self.tfBuffer:tf2_ros.Buffer = tf2_ros.Buffer()
        self.tfListener:tf2_ros.TransformListener = tf2_ros.TransformListener(self.tfBuffer)

        # action server
        self.action_name = name
        self.action_feedback = MoveBaseFeedback()
        self.action_result = MoveBaseResult()
        self.action_server = SimpleActionServer(self.action_name,MoveBaseAction,self.navigate_callback,auto_start=False)
        self.action_server.start()


        self.action_client = SimpleActionClient("/navigation_server",MoveBaseAction)
        self.action_client_goal = None
        self.new_goal = False
        
        rospy.loginfo("Navigation server is active")

        

        

        
        


        # self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.simple_goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.simple_goal_callback)
        # self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)


    def simple_goal_callback(self,msg:PoseStamped):
        goal = MoveBaseGoal()
        goal.target_pose = msg
        self.action_client_goal = goal
        self.new_goal = True

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

    def navigate_callback(self,goal:MoveBaseGoal):
        # 1 create Path planner service client
        # check if planner service is available
        start_pose:PoseStamped = PoseStamped()
        try:
            rospy.wait_for_service(self.path_planner_ser_name,3)
            rospy.loginfo("service available")
            transform:TransformStamped = self.tfBuffer.lookup_transform('map','base_footprint',rospy.Time.now(),rospy.Duration(2.0))
            start_pose.header = transform.header
            start_pose.pose.orientation.w = 1.0
            start_pose = do_transform_pose(start_pose,transform)
            path_requester = rospy.ServiceProxy(self.path_planner_ser_name,RequestPath)
            req = RequestPathRequest()
            req.start = start_pose
            # req.goal.pose.position.x = 0.7078118324279785
            # req.goal.pose.position.y = -5.542358875274658
            # req.goal.pose.orientation.z = 0.7261555851590639
            # req.goal.pose.orientation.w = 0.687530411067247
            req.goal = goal.target_pose
            req.custom_weight_function = False
            print("call service")
            # print(req)
            res:RequestPathResponse = path_requester.call(req)
            print(res.message)
            if not res.state:
                rospy.logerr(res.message)
                self.action_server.set_aborted()
                return
            
            print(len(res.path.poses))

            print("interpolating pose")
            interpolated_path = Path()

            for index in range(len(res.path.poses)-2):
                temp_path = self.interpolate_straight(res.path.poses[index],res.path.poses[index+1])
                for item in temp_path:
                    interpolated_path.poses.append(item)
            
            interpolated_path.poses.append(res.path.poses[-1])
    
            print(len(interpolated_path.poses))
            
            # print(res)
            # print(type(res))
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
                    if self.action_server.is_preempt_requested():
                        self.controller_client.cancel_goal()
                        self.action_server.set_preempted()
                        return

                    rate.sleep()

                print("out of while loop")
            else:
                rospy.logerr("exe_path action server is not running")
                self.action_server.set_aborted()
                return
            
        except rospy.ROSException:
            rospy.logerr("service not available")
            self.action_server.set_aborted()
            return
        except rospy.ServiceException:
            rospy.logerr("problem comunicate with service")
            self.action_server.set_aborted()
            return
        except rospy.ROSSerializationException:
            rospy.logerr("error")
            self.action_server.set_aborted()
            return
        except Exception as err:
            rospy.logerr("other error occur when calling path planner service")
            print(traceback.format_exc())
            self.action_server.set_aborted()
            return
        self.action_server.set_succeeded()
        
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



if __name__ == '__main__':
    rospy.init_node('experiment3')
    server = NavigationServer(rospy.get_name())
    rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    #     if server.new_goal:
    #         server.send_goal()
    #         print("spin")
    #     rate.sleep()
    rospy.spin()