#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

"""
def joint_states_callback(msg):
    rospy.loginfo("Received /joint_states message")
    rospy.loginfo(f"Position: {msg.position}")
    rospy.loginfo(f"Velocity: {msg.velocity}")
    rospy.loginfo(f"Effort: {msg.effort}")
"""

def joint_states_callback(msg):
     motor_degrees = ['2']
     for i in range(6):
        format_degree = f"{msg.position[i]:.6f}"
        motor_degrees.append(format_degree)

     rospy.loginfo(f"Position: {motor_degrees}")

def main():
    rospy.init_node('joint_states_listener_node', anonymous=True)
    
    # /joint_states 토픽 구독
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
    # 노드가 종료될 때까지 대기
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

