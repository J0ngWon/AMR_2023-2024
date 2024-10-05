#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

# Configuration constants
THETA_EXT = 0.27
THETA_RET = np.pi / 4
L_FUDGE = 0.08
Z_MAX_SIDE = -3.15
Z_MAX_DOWN = 0
Z_MIN = 0.1
CLOSE_ENOUGH = 0.02
S_SIDE_MAX = 0.6
S_SIDE_MIN = 0.1
S_TOP_MAX = 0.5

# Flag to control logging output
coordinate_logged = False

class BraccioInverseKinematics:
    def __init__(self):
        pass

    def calculate_thetas(self, x, y, z, beta, gamma):
        """
        목표 위치 (x, y, z)와 각도 beta, gamma를 사용하여 theta1부터 theta5까지 각도를 계산하는 함수.
        """
        try:
            # 로봇 팔의 길이(mm)
            d1 = -70    # 링크 1의 길이
            a2 = 125    # 링크 2의 길이
            a3 = 125    # 링크 3의 길이
            dE = -150   # 엔드 이펙터의 길이

            # Step 1: theta1 계산
            theta1 = np.degrees(np.arctan2(x, y))

            # Step 2: r, r', h 계산
            r = np.sqrt(x**2 + y**2)
            r_prime = r - dE * np.cos(np.radians(beta))
            h = z - d1 - dE * np.sin(np.radians(beta))

            # Step 3: phi, rho, psi 계산 (코사인 법칙 사용)
            cos_phi = (a2**2 + a3**2 - r_prime**2 - h**2) / (2 * a2 * a3)
            phi = np.arccos(np.clip(cos_phi, -1.0, 1.0))

            cos_rho = (r_prime**2 + h**2 + a2**2 - a3**2) / (2 * a2 * np.sqrt(r_prime**2 + h**2))
            rho = np.arccos(np.clip(cos_rho, -1.0, 1.0))

            psi = np.arctan2(h, r_prime)

            # Step 4: theta2, theta3 계산
            if x < 0.35:
                theta2 = rho + psi - np.radians(80)
            else:
                theta2 = rho + psi - np.radians(96)

            theta3 = np.pi - phi

            # Step 5: 조건에 따라 theta4 계산
            if x < 0.35:
                theta4 = (theta2 + theta3) - np.radians(70)  
            else:
                theta4 = (theta2 + theta3)

            # Step 6: theta5 계산
            theta5 = 180

            # 각도 범위 체크 및 조정 (라디안으로 변환)
            theta1 = np.clip(np.radians(theta1), 0, np.pi)
            theta2 = np.clip(theta2, np.radians(15), np.radians(165))
            theta3 = np.clip(theta3, np.radians(0), np.radians(180))
            theta4 = np.clip(theta4, np.radians(0), np.radians(180))
            theta5 = np.clip(theta5, np.radians(10), np.radians(180))

            # 최종 각도 반환
            return [theta1, theta2, theta3, theta4, theta5]

        except ValueError as e:
            rospy.logerr(f"각도 계산 오류: {e}")
            raise ValueError("각도를 계산하는 중 오류가 발생했습니다.")

class BraccioObjectTargetInterface:
    """로봇 동작을 제어하는 인터페이스"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('braccio_xy_bb_target', anonymous=True)

        self.move_group = self.connect_to_move_group("arm", timeout=15)
        self.hand_group = self.connect_to_move_group("hand", timeout=15)
        self.kinematics = BraccioInverseKinematics()
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=20)
        self.scene = PlanningSceneInterface()
        self.bk_data_sub = rospy.Subscriber("/detectnet/pick_goal", Float64MultiArray, self.bk_data_callback)
        self.target_position = PoseStamped()
        self.initial_target_position = None
        rospy.loginfo("BraccioObjectTargetInterface 초기화 완료.")

    def connect_to_move_group(self, group_name, timeout=10):
        start_time = time.time()
        while True:
            try:
                move_group = moveit_commander.MoveGroupCommander(group_name)
                rospy.loginfo(f"'{group_name}' 계획 그룹에 연결되었습니다.")
                return move_group
            except RuntimeError as e:
                if time.time() - start_time > timeout:
                    rospy.logerr(f"{timeout} 초 이내에 '{group_name}' 계획 그룹에 연결할 수 없습니다.")
                    sys.exit(1)
                rospy.logwarn(f"오류로 인해 '{group_name}' 계획 그룹에 다시 연결 시도 중: {e}")
                time.sleep(1)

    def set_gripper_angle(self, angle):
        try:
            angle_rad = np.clip(np.radians(angle), np.radians(10), np.radians(70))
            joint_goal = self.hand_group.get_current_joint_values()
            joint_goal[0] = angle_rad
            self.hand_group.set_joint_value_target(joint_goal)
            self.hand_group.go(wait=True)
            self.hand_group.stop()
            rospy.loginfo(f"Gripper 각도 설정: {angle}도")
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"Gripper 각도 설정 중 오류 발생: {e}")

    def bk_data_callback(self, data):
        global coordinate_logged
        x, y, z = None, None, None

        if len(data.data) == 3 and not coordinate_logged:
            x, y, z = data.data

            # 목표 위치를 PoseStamped로 설정합니다. 이후에는 이 값이 절대 변하지 않도록 합니다.
            self.target_position = PoseStamped()
            self.target_position.header.frame_id = "base_link"
            self.target_position.pose.position.x = x
            self.target_position.pose.position.y = y
            self.target_position.pose.position.z = z

            # 초기 목표 위치를 별도로 저장하여 나중에 참조할 때 항상 원본 값을 사용하도록 합니다.
            self.initial_target_position = PoseStamped()
            self.initial_target_position.pose.position.x = x
            self.initial_target_position.pose.position.y = y
            self.initial_target_position.pose.position.z = z

            rospy.loginfo(f"bk_data에서 설정된 새 목표 위치 - x: {x}, y: {y}, z: {z}")

            # 지정된 각도로 로봇을 먼저 이동
            self.move_robot_to_initial_pose()

            # Gripper 각도 설정
            self.set_gripper_angle(10)

            # 마커만 표시하고 충돌 객체는 추가하지 않음
            self.add_marker_only(x, y, z)

            # 로봇을 목표 위치로 이동
            self.go_to_xy(x, y, z)

            coordinate_logged = True

    def move_robot_to_initial_pose(self):
        """
        지정된 각도로 로봇을 이동시키는 함수.
        """
        try:
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = 1.0  # 임의로 지정된 각도, 필요시 수정 가능
            joint_goal[1] = 1.5  # 임의로 지정된 각도, 필요시 수정 가능
            joint_goal[2] = 1.5  # 임의로 지정된 각도, 필요시 수정 가능
            joint_goal[3] = 0.0  # 임의로 지정된 각도, 필요시 수정 가능

            self.move_group.set_joint_value_target(joint_goal)
            self.move_group.go(wait=True)
            self.move_group.stop()

            rospy.loginfo("로봇이 지정된 초기 자세로 이동하였습니다.")
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"초기 자세로 로봇을 이동시키는 중 오류 발생: {e}")

    def go_to_xy(self, x, y, z):
        """
        목표 위치로 로봇 팔을 이동시키는 함수.
        """
        try:
            beta = 90  
            gamma = 130  
            thetas = self.kinematics.calculate_thetas(x, y, z, beta, gamma)
            self.move_robot_to_thetas(thetas)

        except Exception as e:
            rospy.logerr(f"목표 위치로 이동하는 중 오류 발생: {e}")

    def add_marker_only(self, x, y, z):
        """
        충돌 객체를 추가하지 않고 마커만 표시하는 함수
        """
        # 마커 표시
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "braccio_target"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
        rospy.loginfo(f"마커만 표시 - x: {x}, y: {y}, z: {z}")

    def move_robot_to_thetas(self, thetas):
        """
        계산된 각도를 로봇에 적용하여 움직이도록 하는 함수
        """
        try:
            # 목표 각도를 설정
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = thetas[0]
            joint_goal[1] = thetas[1]
            joint_goal[2] = thetas[2]
            joint_goal[3] = thetas[3]
            joint_goal[4] = thetas[4]
            self.move_group.set_joint_value_target(joint_goal)
            self.move_group.go(wait=True)
            self.move_group.stop()
    
            rospy.sleep(0.5)  # 이동이 완전히 완료되도록 잠시 대기
    
            # 이동 후 최종 각도 출력
            final_joint_values = self.move_group.get_current_joint_values()
            rospy.loginfo("이동 완료 후 각 관절의 최종 각도:")
            rospy.loginfo(f"theta1: {np.degrees(final_joint_values[0]):.2f}도")
            rospy.loginfo(f"theta2: {np.degrees(final_joint_values[1]):.2f}도")
            rospy.loginfo(f"theta3: {np.degrees(final_joint_values[2]):.2f}도")
            rospy.loginfo(f"theta4: {np.degrees(final_joint_values[3]):.2f}도")
            rospy.loginfo(f"theta5: {np.degrees(final_joint_values[4]):.2f}도")
    
            updated_current_pose = self.move_group.get_current_pose().pose
    
            # 목표 위치와 현재 위치 간의 오차 계산
            x_diff = abs(updated_current_pose.position.x - self.initial_target_position.pose.position.x)
            y_diff = abs(updated_current_pose.position.y - self.initial_target_position.pose.position.y)
            z_diff = abs(updated_current_pose.position.z - self.initial_target_position.pose.position.z)
    
            rospy.loginfo(f"목표 위치와의 오차 - x: {x_diff:.4f} m, y: {y_diff:.4f} m, z: {z_diff:.4f} m")
            if x_diff < 0.05 and y_diff < 0.05 and z_diff < 0.05:
                rospy.loginfo("목표 위치에 도달했습니다.")
            else:
                rospy.logwarn("목표 위치에 정확히 도달하지 못했습니다.")
                rospy.logwarn(f"현재 위치 - x: {updated_current_pose.position.x:.4f}, y: {updated_current_pose.position.y:.4f}, z: {updated_current_pose.position.z:.4f}")
                rospy.logwarn(f"목표 위치 - x: {self.initial_target_position.pose.position.x:.4f}, y: {self.initial_target_position.pose.position.y:.4f}, z: {self.initial_target_position.pose.position.z:.4f}")
            
            self.set_gripper_angle(70)
            
            # 바구니로 이동
            self.move_robot_to_new_pose()

        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"로봇 각도 적용 중 오류 발생: {e}")

    def move_robot_to_new_pose(self):

        try:
            # 새로 지정할 기본 각도
            new_joint_goal = self.move_group.get_current_joint_values()
            new_joint_goal[0] = 1.57  # 새로운 각도 예시
            new_joint_goal[1] = 1.57  # 새로운 각도 예시
            new_joint_goal[2] = 1.57  # 새로운 각도 예시
            new_joint_goal[3] = 2.15  # 새로운 각도 예시
    
            # 속도 제한 설정 (0.1배속으로 이동)
            self.move_group.set_max_velocity_scaling_factor(0.1)
    
            # 로봇을 초기 지정 각도로 이동
            self.move_group.set_joint_value_target(new_joint_goal)
            self.move_group.go(wait=True)
            self.move_group.stop()

            rospy.sleep(1)

            rospy.loginfo("바구니로 이동")
    
            # 세 번째 관절 각도만 3.0 라디안으로 변경
            new_joint_goal[2] = 3.0
    
            # 나머지 관절 각도는 유지한 채, 세 번째 관절만 변경하여 이동
            self.move_group.set_joint_value_target(new_joint_goal)
            self.move_group.go(wait=True)
            self.move_group.stop()
    
            # 그리퍼 각도를 70도로 변경
            self.set_gripper_angle(10)
            rospy.loginfo("바구니 완료")

            # 모든 동작이 완료된 후 최종 위치로 이동
            self.move_robot_to_final_pose()
        
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"새로운 지정 각도로 이동 중 오류 발생: {e}")

    def move_robot_to_final_pose(self):
        """
        모든 동작이 완료된 후 원하는 최종 각도로 로봇을 움직이는 함수
        """
        try:
            # 원하는 최종 각도 설정
            final_joint_goal = self.move_group.get_current_joint_values()
            final_joint_goal[0] = 0.2  # 원하는 각도 설정 예시
            final_joint_goal[1] = 1.9  # 원하는 각도 설정 예시
            final_joint_goal[2] = 0.0  # 원하는 각도 설정 예시
            final_joint_goal[3] = 1.5  # 원하는 각도 설정 예시
            final_joint_goal[4] = 1.0  # 원하는 각도 설정 예시
    
            # 속도 제한 설정 (0.1배속으로 이동)
            self.move_group.set_max_velocity_scaling_factor(0.1)
    
            # 로봇을 최종 지정 각도로 이동
            self.move_group.set_joint_value_target(final_joint_goal)
            self.move_group.go(wait=True)
            self.move_group.stop()
    
            rospy.loginfo("로봇이 최종 지정 각도로 이동하였습니다.")
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(f"최종 지정 각도로 이동 중 오류 발생: {e}")


def main():
    print("Loading ....")
    bb_targetter = BraccioObjectTargetInterface()
    rospy.spin()

if __name__ == "__main__":
    rospy.sleep(5)
    main()

