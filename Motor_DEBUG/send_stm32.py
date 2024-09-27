import serial
import time
import threading

# 시리얼 포트 설정
ser = serial.Serial(
    port='COM8',
    baudrate=115200,
    timeout=None
)

goal_set = []
motor_set = []

def send_command(command):
    #print(command)
    command = ''.join(map(str, command))
    command += '\n'
    ser.write(command.encode())

    time.sleep(0.025)

def cmd_vel():
    global goal_set

    try:
        goal_set = []

        linear = 12.345678
        angular = 23.456789
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

    except ValueError:
        print("잘못된 입력입니다. 숫자를 입력하세요.")

def motor_dgrees():
    global motor_set

    #motor_set = ['2', '1.483530', '0.261799','0.523599','1.570800','0.000000','0.174533']
    motor_set = ['2', '1.483530', '1.570800','1.570800','1.570800','0.000000','1.274090']

def send_data_thread():
    global goal_set, motor_set

    while True:
        cmd_vel()
        motor_dgrees()

        #send_command(goal_set)
        send_command(motor_set)

def receive_data_thread():
    buffer = b''  # 데이터를 저장할 버퍼
    while True:
        if ser.in_waiting > 0:
            buffer += ser.read()  # 데이터를 읽어서 버퍼에 추가
            if b'\r\n' in buffer:  # 줄바꿈 문자가 있으면 출력
                #received_data = buffer.decode('utf-8').strip()  # 디코딩 후 앞뒤 공백 제거
                #print(f"수신 데이터: {received_data}")
                print(f"수신 데이터: {buffer}")
                buffer = b''  # 버퍼 초기화

try:
    # 송신 스레드 생성
    send_thread = threading.Thread(target=send_data_thread)

    # 수신 스레드 생성
    receive_thread = threading.Thread(target=receive_data_thread)

    # 스레드 시작
    send_thread.start()
    receive_thread.start()

    # 메인 스레드가 종료되지 않도록 대기
    send_thread.join()
    receive_thread.join()

finally:
    ser.close()
