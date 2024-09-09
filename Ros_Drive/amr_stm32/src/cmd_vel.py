#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    # linear.x와 angular.z 값을 추출하여 변수에 저장
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    # 값을 출력
    rospy.loginfo(f"Linear X: {linear_x}, Angular Z: {angular_z}")

def main():
    # ROS 노드 초기화
    rospy.init_node('cmd_vel_listener', anonymous=True)

    # cmd_vel 토픽을 구독하고 콜백 함수 설정
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    # ROS 스핀 (노드 종료 시까지 대기)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

