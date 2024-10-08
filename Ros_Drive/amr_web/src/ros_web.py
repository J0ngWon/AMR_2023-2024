#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo('Received message from web: %s', msg.data)

if __name__ == '__main__':
    rospy.init_node('amr_web')
    rospy.Subscriber('/web_log', String, callback)
    rospy.spin()

