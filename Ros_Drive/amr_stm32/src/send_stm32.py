#!/usr/bin/env python3

import rospy
import serial
import time
import threading
import math
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster

ser = None

tx_stm32_cmd_vel = ''
tx_stm32_motor_degrees = ''

odom_data = None
robot_data = '0'

#setup Serial Port
def setup_serial():
    global ser
    # Launch 파일에서 파라미터 값을 가져옵니다.
    port = rospy.get_param('serial_port', '/dev/AMR_STM32')
    baudrate = rospy.get_param('serial_baudrate', 115200)

    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=None
    )
    rospy.loginfo(f"Connected to {port} at {baudrate} baud rate.")

def cmd_vel_callback(msg):
    global tx_stm32_cmd_vel

    cmd_vel = ['1']
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    if linear_x < 0:
        linear_x = abs(linear_x)
    else:
        linear_x += 100

    if angular_z < 0:
        angular_z = abs(angular_z)
    else:
        angular_z += 100

    format_linear = f"{linear_x:03.6f}"
    format_angular = f"{angular_z:03.6f}"

    if linear_x < 100:
        format_linear = f"00{format_linear}"

    if angular_z < 100:
        format_angular = f"00{format_angular}"

    cmd_vel.append(format_linear)
    cmd_vel.append(format_angular)

    tx_stm32_cmd_vel = ''.join(map(str, cmd_vel))

    #rospy.loginfo(f"cmd_vel: {cmd_vel}")
    
def joint_states_callback(msg):
    global tx_stm32_motor_degrees

    motor_degrees = ['2']
    for i in range(6):
        format_degree = f"{msg.position[i]:.6f}"
        motor_degrees.append(format_degree)

    tx_stm32_motor_degrees = ''.join(map(str, motor_degrees))

    #rospy.loginfo(f"Position: {motor_degrees}")

def pub_odometry(odom_pub, tf_broadcaster):
    global odom_data

    if odom_data != None:
        odom_data_float = [float(val) if val != 'nan' else 0.0 for val in odom_data]

        # Odometry 메시지 생성 및 발행
        x, y, th, dt, vx, vy = odom_data_float[:6]
        quaternion = quaternion_from_euler(0, 0, th)
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*quaternion))
        odom_msg.twist.twist = Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, th * 1000 / dt))
        # Odometry 메시지 발행
        odom_pub.publish(odom_msg)
        # TF 발행
        tf_broadcaster.sendTransform(
            (x, y, 0),
            quaternion,
            rospy.Time.now(),
            "base_footprint",
            "odom"
        )
        
def pub_braccio_state(braccio_state_pub):
    global robot_data

    braccio_msg = Float64MultiArray()
    braccio_msg.data = [float(i) for i in robot_data]
    braccio_state_pub.publish(braccio_msg)

def send_command(command):
    command += '\n'
    ser.write(command.encode())
    time.sleep(0.025) #40HZ

def send_data_thread():
    global tx_stm32_cmd_vel, stx_tm32_motor_degrees
    while not rospy.is_shutdown():
        try:
            send_command(tx_stm32_cmd_vel)
            #tx_stm32_motor_degrees = '21.4835301.5708001.5708001.5708000.0000000.174533'
            #tx_stm32_motor_degrees = '21.4835300.2617990.5235992.200000.0000000.174533'
            send_command(tx_stm32_motor_degrees)

        except TimeoutError:
            rospy.logwarn("Timeout Error!!! When Send Data")

def receive_data_thread():
    global odom_data, robot_data

    buffer = b''
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            
            buffer = ser.read_until(b'\r\n')

            received_data = buffer.decode('utf-8', errors='ignore').strip()

            if '|' in received_data and '#' in received_data and received_data.count(' ') == 13:
                try:
                    
                    #rospy.loginfo(f"수신 데이터: {received_data}")

                    odom_part, robot_arm_part = received_data[1:].split('|')

                    odom_data = odom_part.strip().split()
                    robot_data = robot_arm_part.strip().split()

                except UnicodeDecodeError as e:
                    rospy.logwarn(f"Decoding error: {e}")

            buffer = b''  # 버퍼 초기화

def main():
    rospy.init_node('stm32_serial', anonymous=True)
    
    #setup Serial Port
    setup_serial()

    # 토픽 구독
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # 토픽 발행
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    braccio_state_pub = rospy.Publisher('/braccio_state', Float64MultiArray, queue_size=10)


    #TF 발행
    tf_broadcaster = TransformBroadcaster()
    
    # 송신 스레드 생성
    send_thread = threading.Thread(target=send_data_thread, daemon=True)

    # 수신 스레드 생성
    receive_thread = threading.Thread(target=receive_data_thread, daemon=True)

    # 스레드 시작
    send_thread.start()
    receive_thread.start()

    # ROS 노드가 종료될 때까지 대기
    rate = rospy.Rate(20)  # 20Hz 주기로 odom 발행
    while not rospy.is_shutdown():
        pub_odometry(odom_pub, tf_broadcaster)
        pub_braccio_state(braccio_state_pub)
        rate.sleep()

    # 노드 종료 시 시리얼 포트 닫기
    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

