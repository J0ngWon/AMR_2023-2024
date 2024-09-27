import serial
import time
import threading
import msvcrt
import tkinter as tk
from tkinter import ttk

ser = serial.Serial(port="COM8", baudrate=115200, timeout=None)

goal_set = []

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i: move forward
, (comma): move backward
j: turn left
l: turn right
u: forward-left
o: forward-right
m: backward-left
. (dot): backward-right
k: stop
"""
change_set = 0.1

# 키보드 입력에 대한 이동 속도 설정
move_bindings = {
    "i": (1, 0),
    ",": (-1, 0),
    "j": (0, 1),
    "l": (0, -1),
    "u": (1, 1),
    "o": (1, -1),
    "m": (-1, 1),
    ".": (-1, -1),
    "k": (0, 0),
}

change_speed = {
    "q": change_set,
    "z": -change_set,
    "e": change_set,
    "c": -change_set,
}

# 초기 선형 및 각속도 값
speed = 0.5
turn = 1.0
linear = 0.0
angular = 0.0
tk_linear = 0
tk_angular = 0


def send_command(command):
    command = "".join(map(str, command))
    command += "\n"
    ser.write(command.encode())
    time.sleep(0.025)


def send_data_thread():
    global goal_set
    while True:
        send_command(goal_set)


def receive_data_thread():
    buffer = b""  # 데이터를 저장할 버퍼
    while True:
        if ser.in_waiting > 0:
            buffer += ser.read()  # 데이터를 읽어서 버퍼에 추가
            if b"\r\n" in buffer:  # 줄바꿈 문자가 있으면 출력
                update_labels(buffer)  # GUI 업데이트
                buffer = b""  # 버퍼 초기화


def get_key():
    global speed, turn, tk_linear, tk_angular, goal_set
    while True:
        key = msvcrt.getch().decode("utf-8")
        if key in change_speed:
            if key == "q" or key == "z":
                speed += change_speed[key]
            else:
                turn += change_speed[key]

        if key in move_bindings:
            # 선형 및 각속도 업데이트
            goal_set = ["1"]

            linear = move_bindings[key][0] * speed
            angular = move_bindings[key][1] * turn

            tk_linear = linear
            tk_angular = angular

            if linear < 0:
                linear = abs(linear)
            else:
                linear += 100
            if angular < 0:
                angular = abs(angular)
            else:
                angular += 100

            format_linear = f"{linear:03.6f}"
            format_angular = f"{angular:03.6f}"
            if linear < 100:
                format_linear = f"00{format_linear}"
            if angular < 100:
                format_angular = f"00{format_angular}"

            goal_set.append(format_linear)
            goal_set.append(format_angular)

            # 상태 출력
            update_labels()  # GUI 업데이트


def update_labels(buffer=None):
    set_linear_label_var.set(f"set_linear: {speed:.6f}")
    set_angular_label_var.set(f"set_angular: {turn:.6f}")
    linear_label_var.set(f"linear: {tk_linear:.6f}")
    angular_label_var.set(f"angular: {tk_angular:.6f}")
    stm_label_var.set(f"STM: {buffer if buffer else b''}")


if __name__ == "__main__":
    # Tkinter 윈도우 생성
    root = tk.Tk()
    root.title("Robot Teleoperation Monitor")

    style = ttk.Style()
    style.configure("Bold.TLabel", font=("Helvetica", 14, "bold"))

    # ttk 라벨 변수 생성
    set_linear_label_var = tk.StringVar()
    set_angular_label_var = tk.StringVar()
    linear_label_var = tk.StringVar()
    angular_label_var = tk.StringVar()
    stm_label_var = tk.StringVar()

    # ttk 라벨 생성 및 배치
    set_linear_label = ttk.Label(root, textvariable=set_linear_label_var, style="Bold.TLabel")
    set_linear_label.grid(row=0, column=0, padx=10, pady=10, sticky="w")

    set_angular_label = ttk.Label(root, textvariable=set_angular_label_var, style="Bold.TLabel")
    set_angular_label.grid(row=0, column=1, padx=10, pady=10, sticky="w")

    linear_label = ttk.Label(root, textvariable=linear_label_var, style="Bold.TLabel")
    linear_label.grid(row=1, column=0, padx=10, pady=10, sticky="w")

    angular_label = ttk.Label(root, textvariable=angular_label_var, style="Bold.TLabel")
    angular_label.grid(row=1, column=1, padx=10, pady=10, sticky="w")

    stm_label = ttk.Label(root, textvariable=stm_label_var, style="Bold.TLabel")
    stm_label.grid(row=2, column=0, padx=10, pady=10, columnspan=2, sticky="w")

    # 초기 라벨 설정
    update_labels()

    try:
        print(msg)
        # 송신 스레드 생성
        send_thread = threading.Thread(target=send_data_thread)
        send_thread.daemon = True

        # 수신 스레드 생성
        receive_thread = threading.Thread(target=receive_data_thread)
        receive_thread.daemon = True

        # 키 입력 스레드 생성
        cmd_vel_thread = threading.Thread(target=get_key)
        cmd_vel_thread.daemon = True

        # 스레드 시작
        send_thread.start()
        receive_thread.start()
        cmd_vel_thread.start()

        # Tkinter 이벤트 루프 실행
        root.mainloop()

    finally:
        ser.close()
