#!/usr/bin/env python

import rospy
# message_filter for distance check
import message_filters
from nav_msgs.msg import Odometry
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import std_srvs.srv
import ast

import json
import yaml
import requests
    
class navigation_turtlebot(object):
    def __init__(self):
        self.nav_goal = None

        # message_filter for distance check
        self.status_sub = message_filters.Subscriber("/odom", Odometry) #목표지를 [x, y, z] 리스트로 보내줌 ->추후 비전단에서 보내줄 것을 대체
        self.goal_sub = message_filters.Subscriber("/absolute_coord", String) #목표지를 받음
        
        self.ts=message_filters.ApproximateTimeSynchronizer([self.status_sub, self.goal_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.subscribe_goal_from_system_publish_goal_to_bot)

        # self.goal_pub = rospy.Publisher("/absolute_coord", String, queue_size=10) #목표지를 [x, y, z] 리스트로 보내줌 ->추후 비전단에서 보내줄 것을 대체
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10) #리스트로 받은 목표지를 로봇에게 송신
    
    def run(self):        
        while not rospy.is_shutdown():
            rospy.sleep(10)
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
            rospy.sleep(10)
            # self.publish_goal_to_system()

    # def msg2json(self, msg):
    #     ''' Convert a ROS message to JSON format'''
    #     y = yaml.load(str(msg))
    #     return json.dumps(y,indent=4)
    
    # message_filters
    def subscribe_goal_from_system_publish_goal_to_bot(self, status_msg, goal_msg):
        rospy.loginfo('distance_check')
        pose=ast.literal_eval(goal_msg.data)
       
        self.nav_goal = PoseStamped()
        self.nav_goal.header.frame_id = 'map'
        self.nav_goal.pose.position.x = pose[0]
        self.nav_goal.pose.position.y = pose[1]
        self.nav_goal.pose.orientation.z = pose[2]

        goal_x=pose[0]
        goal_y=pose[1]

        current_x=status_msg.pose.pose.position.x
        current_y=status_msg.pose.pose.position.y
        
        # msg_json=self.msg2json(status_msg) #msg->json
        # print(msg_json)
        
        #post msg_json to server
        # response = requests.post('http://1.215.235.253:27020/ros', json = msg_json)
        
        distance=math.sqrt(pow(goal_x-current_x,2)+pow(goal_y-current_y,2))

        if distance>0.1:
            self.nav_pub.publish(self.nav_goal)
            rospy.loginfo("publish_goal_to_bot || "+ str(pose))

    # def publish_goal_to_system(self):
    #     rospy.loginfo('publish_goal_to_system')
    #     goal="[1.5, 0.0, 1.0]"
    #     self.goal_pub.publish(goal)

def main():
    rospy.loginfo('start')
    rospy.init_node("nav_goal_stresstest", anonymous=False)
    nav_stress_test = navigation_turtlebot()
    nav_stress_test.run()

if __name__ == '__main__':
    main()

