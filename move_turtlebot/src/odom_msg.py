#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import json
import yaml
import requests

def msg2json(msg):
    ''' Convert a ROS message to JSON format'''
    y = yaml.load(str(msg))
    return json.dumps(y,indent=4)

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "Callback")
    msg_json=msg2json(msg) #msg->json
    print(msg_json)

    #post msg_json to server
    response = requests.post('http://1.215.235.253:27020/ros', json = msg_json)
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/odom", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()