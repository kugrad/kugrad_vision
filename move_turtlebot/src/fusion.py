#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
###
import message_filters
from nav_msgs.msg import Odometry
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import std_srvs.srv
import ast

pub = None

class navigation_turtlebot(object):
    def __init__(self):
        self.nav_goal = None

        # message_filter for distance check
        self.status_sub = message_filters.Subscriber("/odom", Odometry) #목표지를 [x, y, z] 리스트로 보내줌 ->추후 비전단에서 보내줄 것을 대체
        self.goal_sub = message_filters.Subscriber("/custom_goal", String) #목표지를 받음
        
        self.ts=message_filters.ApproximateTimeSynchronizer([self.status_sub, self.goal_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.subscribe_goal_from_system_publish_goal_to_bot)

        self.goal_pub = rospy.Publisher("/custom_goal", String, queue_size=10) #목표지를 [x, y, z] 리스트로 보내줌 ->추후 비전단에서 보내줄 것을 대체
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10) #리스트로 받은 목표지를 로봇에게 송신
    
    def run(self):        
        while not rospy.is_shutdown():
            rospy.sleep(10)
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
            rospy.sleep(10)
            self.publish_goal_to_system()

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

        distance=math.sqrt(pow(goal_x-current_x,2)+pow(goal_y-current_y,2))

        if distance>0.1:
            self.nav_pub.publish(self.nav_goal)
            rospy.loginfo("publish_goal_to_bot || "+ str(pose))

    def publish_goal_to_system(self):
        rospy.loginfo('publish_goal_to_system')
        goal="[1.5, 0.0, 1.0]"
        self.goal_pub.publish(goal)


def callback_laser(msg):
  # 120 degrees into 3 regions
  regions = {
    'right':  min(min(msg.ranges[0:2]), 10),
    'front':  min(min(msg.ranges[3:5]), 10),
    'left':   min(min(msg.ranges[6:9]), 10),
  }
  
  take_action(regions)
  
def take_action(regions):
  threshold_dist = 1.5
  linear_speed = 0.6
  angular_speed = 1

  msg = Twist()
  nav_stress_test = navigation_turtlebot()
  linear_x = 0
  angular_z = 0
  
  state_description = ''
  
  if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 1 - no obstacle'
    nav_stress_test.run()

    linear_x = linear_speed
    angular_z = 0
  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 7 - front and left and right'
    linear_x = -linear_speed
    angular_z = angular_speed # Increase this angular speed for avoiding obstacle faster
  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 2 - front'
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 3 - right'
    linear_x = 0
    angular_z = -angular_speed
  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 4 - left'
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 5 - front and right'
    linear_x = 0
    angular_z = -angular_speed
  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 6 - front and left'
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 8 - left and right'
    linear_x = linear_speed
    angular_z = 0
  else:
    state_description = 'unknown case'
    rospy.loginfo(regions)

  rospy.loginfo(state_description)
  msg.linear.x = linear_x
  msg.angular.z = angular_z
  pub.publish(msg)

def main():
  global pub
  
  rospy.init_node('reading_laser')
  
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  
  sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
  
  rospy.spin()

if __name__ == '__main__':
  main()