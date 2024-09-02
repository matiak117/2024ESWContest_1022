import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np
import serial
import threading
import time
import re
import math
import matplotlib.pyplot as plt
import queue
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# 전역 변수 설정
distance_to_marker = 0
speedleft1 = speedright1 = 0
speedleft2 = speedright2 = 0
speedleft3 = speedright3 = 0
last_detection_time = 0
emergency_stop_message = ""
distance1 = distance2 = distance3 = distance4 = 0

# 시리얼 포트 설정
arduino_port = 'COM3'  # 아두이노 포트 설정
arduino_baudrate = 9600
lidar_port = 'COM5'  # LiDAR 포트 설정
lidar_baudrate = 128000

# 데이터 큐
data_queue = queue.Queue()

def setup_serial():
    global arduino, lidar
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            arduino = serial.Serial(arduino_port, arduino_baudrate)
            time.sleep(2)
            print("Arduino serial port opened successfully.")
            break
        except serial.SerialException as e:
            print(f"Arduino 연결 시도 {attempt + 1}/{max_attempts} 실패: {e}")
            if attempt == max_attempts - 1:
                print("Arduino 연결 실패")
                arduino = None

    for attempt in range(max_attempts):
        try:
            lidar = serial.Serial(lidar_port, lidar_baudrate)
            if lidar.is_open:
                print("LiDAR serial port opened successfully.")
                break
            else:
                raise serial.SerialException("Failed to open LiDAR port.")
        except serial.SerialException as e:
            print(f"LiDAR 연결 시도 {attempt + 1}/{max_attempts} 실패: {e}")
            if attempt == max_attempts - 1:
                print("LiDAR 연결 실패")
                lidar = None
setup_serial()

# 모터 제어 함수
def stop_motors():
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    if arduino:
        send_to_arduino(0, 0, 0, 0, 0, 0)
    speedleft1 = speedright1 = 0
    speedleft2 = speedright2 = 0
    speedleft3 = speedright3 = 0
    update_labels()

def smooth_acceleration(target_speeds, step=0.05):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    while True:
        speedleft1 = accelerate_towards(speedleft1, target_speeds[0], step)
        speedright1 = accelerate_towards(speedright1, target_speeds[1], step)
        speedleft2 = accelerate_towards(speedleft2, target_speeds[2], step)
        speedright2 = accelerate_towards(speedright2, target_speeds[3], step)
        speedleft3 = accelerate_towards(speedleft3, target_speeds[4], step)
        speedright3 = accelerate_towards(speedright3, target_speeds[5], step)
        
        set_motor_speeds()
        update_labels()
        
        if all_close([speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3], target_speeds):
            break
        time.sleep(0.1)

def accelerate_towards(current, target, step):
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

def all_close(values, targets, tolerance=0.01):
    return all(abs(v - t) < tolerance for v, t in zip(values, targets))

def set_motor_speeds():
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    if arduino:
        send_to_arduino(speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3)

def send_to_arduino(left_speed1, right_speed1, left_speed2, right_speed2, left_speed3, right_speed3):
    if arduino:
        try:
            arduino.write(f"L1{left_speed1}R1{right_speed1}L2{left_speed2}R2{right_speed2}L3{left_speed3}R3{right_speed3}\n".encode())
        except serial.SerialException as e:
            print(f"데이터 전송 오류: {e}")

def update_labels():
    left_label1.config(text=f"왼쪽 모터 1: {round(speedleft1, 2)}")
    right_label1.config(text=f"오른쪽 모터 1: {round(speedright1, 2)}")
    distance_label.config(text=f"마커까지 거리: {round(distance_to_marker, 2)} cm")
    emergency_label.config(text=emergency_stop_message)  # 긴급 정지 메시지 업데이트

def increase_speed(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        min(speedleft1 + 0.05, 1),
        min(speedright1 + 0.05, 1),
        min(speedleft2 + 0.05, 1),
        min(speedright2 + 0.05, 1),
        min(speedleft3 + 0.05, 1),
        min(speedright3 + 0.05, 1),
    ]
    smooth_acceleration(target_speeds)
    
def decrease_speed(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        max(speedleft1 - 0.05, -1),
        max(speedright1 - 0.05, -1),
        max(speedleft2 - 0.05, -1),
        max(speedright2 - 0.05, -1),
        max(speedleft3 - 0.05, -1),
        max(speedright3 - 0.05, -1),
    ]
    smooth_acceleration(target_speeds)

def steer_right(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        max(speedleft1 - 0.05, -1),
        min(speedright1 + 0.05, 1),
        max(speedleft2 - 0.05, -1),
        min(speedright2 + 0.05, 1),
        max(speedleft3 - 0.05, -1),
        min(speedright3 + 0.05, 1),
    ]
    smooth_acceleration(target_speeds)

def steer_left(event=None):
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    target_speeds = [
        min(speedleft1 + 0.05, 1),
        max(speedright1 - 0.05, -1),
        min(speedleft2 + 0.05, 1),
        max(speedright2 - 0.05, -1),
        min(speedleft3 + 0.05, 1),
        max(speedright3 - 0.05, -1),
    ]
    smooth_acceleration(target_speeds)

def quit_program(event=None):
    stop_motors()
    if arduino:
        arduino.close()
    if lidar:
        lidar.close()
    cap.release()
    root.destroy()

def detect_aruco_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    markers = []
    if ids is not None:
        for i in range(len(ids)):
            marker_corners = corners[i][0]
            center = np.mean(marker_corners, axis=0).astype(int)
            markers.append({
                'id': ids[i][0],
                'center': center,
                'corners': marker_corners
            })
    
    return markers

def draw_marker_overlay(frame, markers):
    for marker in markers:
        marker_corners = marker['corners']
        cv2.polylines(frame, [marker_corners.astype(np.int32)], True, (0, 255, 0), 2)
        center = marker['center']
        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
        marker_id = marker['id']
        cv2.putText(frame, str(marker_id), tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

def aruco_following():
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3, distance_to_marker, last_detection_time
    stop_event = threading.Event()

    while not stop_event.is_set():
        if mode_var.get() == 2:  # ArUco 모드일 때만 작동
            ret, frame = cap.read()

            if ret:
                markers = detect_aruco_markers(frame)
                print(f"Detected markers: {markers}")  # 디버깅용

                if markers:
                    marker_center_x = markers[0]['center'][0]
                    frame_center_x = frame.shape[1] // 2

                    deviation = (marker_center_x - frame_center_x) / frame_center_x
                    speed_adjustment = deviation * 0.5

                    marker_size = np.sqrt(np.sum((markers[0]['corners'][0] - markers[0]['corners'][1])**2))
                    focal_length = 1000
                    real_marker_size = 9
                    distance_to_marker = focal_length * real_marker_size / marker_size

                    print(f"Marker size: {marker_size}, Distance to marker: {distance_to_marker}")  # 디버깅용

                    last_detection_time = time.time()

                    desired_distance = 40
                    if distance_to_marker < desired_distance:
                        speedleft1 = speedright1 = speedleft2 = speedright2 = speedleft3 = speedright3 = 0
                    else:
                        speed_base = 0.4
                        speedleft1 = speed_base - speed_adjustment
                        speedright1 = speed_base + speed_adjustment
                        speedleft2 = speed_base - speed_adjustment
                        speedright2 = speed_base + speed_adjustment
                        speedleft3 = speed_base - speed_adjustment
                        speedright3 = speed_base + speed_adjustment

                    # 속도 범위 제한
                    speedleft1 = max(min(speedleft1, 0.6), -0.6)
                    speedright1 = max(min(speedright1, 0.6), -0.6)
                    speedleft2 = max(min(speedleft2, 0.6), -0.6)
                    speedright2 = max(min(speedright2, 0.6), -0.6)
                    speedleft3 = max(min(speedleft3, 0.6), -0.6)
                    speedright3 = max(min(speedright3, 0.6), -0.6)

                    set_motor_speeds()

                    # GUI 업데이트 예약
                    root.after(100, update_distance_label)

                else:
                    if time.time() - last_detection_time > 1:
                        stop_motors()

        time.sleep(0.1)

    stop_event.set()

def update_distance_label():
    distance_label.config(text=f"마커까지 거리: {round(distance_to_marker, 2)} cm")

def mode_selection_changed(*args):
    if mode_var.get() == 2:  # ArUco 모드로 변경된 경우
        stop_motors()  # 모든 모터를 멈춤
        aruco_follow_thread = threading.Thread(target=aruco_following, daemon=True)
        aruco_follow_thread.start()

def update_serial_data():
    global arduino, distance1, distance2, distance3, distance4, emergency_stop_message
    try:
        if arduino and arduino.is_open:
            if arduino.in_waiting:
                data = arduino.readline().decode().strip()
                print(f"수신 데이터: {data}")
                match = re.match(r'Distance1: (\d+)\s*cm,\s*Distance2: (\d+)\s*cm,\s*Distance3: (\d+)\s*cm,\s*Distance4: (\d+)\s*cm', data)
                if match:
                    distance1 = float(match.group(1))
                    distance2 = float(match.group(2))
                    distance3 = float(match.group(3))
                    distance4 = float(match.group(4))
                    emergency_stop_message = ""
                    update_labels()
                elif "Emergency Stop!" in data:
                    emergency_stop_message = "긴급 정지 발생!"
                    stop_motors()
                    update_labels()
        else:
            print("Arduino 연결이 끊어졌습니다. 재연결 시도 중...")
            setup_serial()
    except serial.SerialException as e:
        print(f"시리얼 통신 오류: {e}")
        print("Arduino 재연결 시도 중...")
        setup_serial()
    except Exception as e:
        print(f"예상치 못한 오류 발생: {e}")
    
    root.after(100, update_serial_data)

# LiDAR 관련 코드 시작
def plot_lidar(distdict, obstacles, closest_obstacle_dist):
    num_points = 180  # 110도부터 240도까지 180개 포인트
    x = [0 for _ in range(num_points)]
    y = [0 for _ in range(num_points)]
    distances = [0 for _ in range(num_points)]  # 거리를 색상으로 사용할 배열
    
    for angle in range(num_points):
        angle_deg = 110 + angle  # 110도부터 시작
        if angle_deg > 360:
            angle_deg -= 360
        distance = distdict.get(angle, 0)
        x[angle] = distance * math.cos(math.radians(angle_deg))
        y[angle] = distance * math.sin(math.radians(angle_deg))  # y축 반전
        distances[angle] = distance  # 거리 값을 저장

    ax.clear()  # 이전 플롯을 지웁니다
    ax.set_ylim(-3000, 3000)
    ax.set_xlim(-3000, 3000)

    # 거리 값에 따라 색상을 다르게 설정
    scatter = ax.scatter(x, y, c=distances, cmap='viridis', s=8)  # 'viridis'는 색상 맵 중 하나입니다.
    fig.colorbar(scatter, ax=ax, label='Distance')  # 색상 바 추가

    # 장애물 위치를 빨간색 'X'로 표시
    if obstacles:
        ox, oy = zip(*obstacles)
        ax.scatter(ox, oy, c='red', marker='x', label='Obstacles')

    ax.set_aspect('equal', adjustable='box')  # 축 비율을 동일하게 설정
    ax.legend()

    # 가장 가까운 장애물 거리 표시
    if closest_obstacle_dist is not None:
        ax.text(-2500, 2700, f'Closest Obstacle Distance: {closest_obstacle_dist:.2f} mm', fontsize=12, color='red')

    canvas.draw()

def _CheckSum(data):
    try:
        ocs = _HexArrToDec((data[6], data[7]))
        LSN = data[1]
        cs = 0x55AA ^ _HexArrToDec((data[0], data[1])) ^ _HexArrToDec((data[2], data[3])) ^ _HexArrToDec((data[4], data[5]))
        for i in range(0, 2 * LSN, 2):
            cs = cs ^ _HexArrToDec((data[8+i], data[8+i+1])) 
        return cs == ocs
    except:
        return False

def _HexArrToDec(data):
    littleEndianVal = 0
    for i in range(len(data)):
        littleEndianVal += data[i] * (256 ** i)
    return littleEndianVal

def _AngleCorr(dist):
    if dist == 0:
        return 0
    else:
        return math.atan(21.8 * ((155.3 - dist) / (155.3 * dist))) * (180 / math.pi)

def _Calculate(d):
    ddict = []
    LSN = d[1]
    Angle_fsa = (_HexArrToDec((d[2], d[3])) >> 1) / 64.0
    Angle_lsa = (_HexArrToDec((d[4], d[5])) >> 1) / 64.0
    Angle_diff = Angle_lsa - Angle_fsa if Angle_fsa < Angle_lsa else 360 + Angle_lsa - Angle_fsa
    for i in range(0, 2 * LSN, 2):
        dist_i = _HexArrToDec((d[8+i], d[8+i+1])) / 4
        Angle_i_tmp = (Angle_diff / float(LSN)) * (i / 2) + Angle_fsa
        Angle_i = Angle_i_tmp if Angle_i_tmp >= 0 else Angle_i_tmp + 360
        Angle_i = Angle_i if Angle_i <= 360 else Angle_i - 360
        Angle_i += _AngleCorr(dist_i)
        # 점의 개수를 늘리기 위해 각도를 더 세밀하게 조정
        for angle_offset in range(0, 2):
            ddict.append((dist_i, Angle_i + angle_offset * 0.25))
    return ddict

def _Mean(data):
    length_of_data_without_zero = sum(i != 0 for i in data)
    return float(sum(data) / length_of_data_without_zero) if data and length_of_data_without_zero else 0

def code(lidar):
    data1 = lidar.read(6000)
    data2 = data1.split(b"\xaa\x55")[1:-1]
    num_points = 180  # 110도부터 240도까지 180개 포인트
    distdict = {i: 0 for i in range(num_points)}
    for e in data2:
        try:
            if e[0] == 0 and _CheckSum(e):
                d = _Calculate(e)
                for ele in d:
                    angle = math.floor(ele[1])  # 각도를 1도 간격으로 설정
                    if 110 <= angle < 240:  # 110도부터 240도까지 필터링
                        distdict[angle - 110] = ele[0]  # 110도를 기준으로 0부터 179까지 매핑
        except:
            pass
    return distdict

def control_robot():
    global speedleft1, speedright1, speedleft2, speedright2, speedleft3, speedright3
    while True:
        try:
            angle_data, obstacles, closest_obstacle_dist = data_queue.get(timeout=1)
            if not obstacles:
                speedleft1 = speedright1 = 100
                speedleft2 = speedright2 = 100
                speedleft3 = speedright3 = 100
                continue

            left_speed = 100
            right_speed = 100
            left_obstacle = False
            right_obstacle = False

            for x, y in obstacles:
                if x > 0:  # 오른쪽 장애물
                    right_obstacle = True
                else:  # 왼쪽 장애물
                    left_obstacle = True

            if left_obstacle and right_obstacle:
                # 양쪽에 장애물이 있는 경우, 직진
                speedleft1 = speedright1 = 50
                speedleft2 = speedright2 = 50
                speedleft3 = speedright3 = 50
            elif left_obstacle:
                # 왼쪽에 장애물이 있는 경우, 오른쪽으로 회피
                speedleft1 = 100
                speedright1 = 50
                speedleft2 = 100
                speedright2 = 50
                speedleft3 = 100
                speedright3 = 50
            elif right_obstacle:
                # 오른쪽에 장애물이 있는 경우, 왼쪽으로 회피
                speedleft1 = 50
                speedright1 = 100
                speedleft2 = 50
                speedright2 = 100
                speedleft3 = 50
                speedright3 = 100
            else:
                # 장애물이 없는 경우, 직진
                speedleft1 = speedright1 = 100
                speedleft2 = speedright2 = 100
                speedleft3 = speedright3 = 100
        except queue.Empty:
            continue

def lidar_thread(lidar):
    while lidar.is_open:
        try:
            angle_data = code(lidar)
            if angle_data:
                # 장애물 데이터 처리
                obstacles = []
                closest_obstacle_dist = None
                for angle in range(180):
                    distance = angle_data[angle]
                    if 20 < distance < 2000:  # 유효한 거리 범위
                        angle_deg = 110 + angle
                        x = distance * math.cos(math.radians(angle_deg))
                        y = distance * math.sin(math.radians(angle_deg))  # y축 반전
                        obstacles.append((x, y))
                        if closest_obstacle_dist is None or distance < closest_obstacle_dist:
                            closest_obstacle_dist = distance
                print(f"Obstacles detected: {obstacles}")  # 디버깅용
                data_queue.put((angle_data, obstacles, closest_obstacle_dist))
            else:
                print("No valid data received.")
            time.sleep(0.1)  # 데이터를 수집할 때마다 잠시 대기
        except serial.SerialException as e:
            print(f"LiDAR thread error: {e}")
            break  # 포트가 닫힌 경우 스레드 종료

def motor_control_thread(arduino):
    while arduino.is_open:
        try:
            arduino.write(f'L1{speedleft1}R1{speedright1}L2{speedleft2}R2{speedright2}L3{speedleft3}R3{speedright3}\n'.encode())
            time.sleep(0.1)  # 모터 제어 명령 주기
        except serial.SerialException as e:
            print(f"Arduino thread error: {e}")
            break  # 포트가 닫힌 경우 스레드 종료

def main():
    global arduino, lidar

    setup_serial()

    if arduino is None and lidar is None:
        print("Arduino와 LiDAR 모두 연결할 수 없습니다. 프로그램을 종료합니다.")
        return

    if lidar:
        try:
            start_command = bytearray([int('a5', 16), int('60', 16)])
            lidar.write(start_command)
            print("Sent scan start command.")

            threading.Thread(target=lidar_thread, args=(lidar,), daemon=True).start()
        except Exception as e:
            print(f"LiDAR 시작 오류: {e}")

    if arduino:
        threading.Thread(target=motor_control_thread, args=(arduino,), daemon=True).start()

    threading.Thread(target=control_robot, daemon=True).start()

    # GUI 시작
    root.after(100, update_serial_data)
    root.after(100, update_lidar_plot)
    root.mainloop()

    # 프로그램 종료 시 정리
    if lidar and lidar.is_open:
        lidar.close()
        print("LiDAR serial port closed.")
    if arduino and arduino.is_open:
        arduino.close()
        print("Arduino port closed.")

if __name__ == '__main__':
    main()

# GUI 초기화
root = tk.Tk()
root.title("로봇 제어 GUI")

# 모터 속도와 거리 표시 레이블
left_label1 = tk.Label(root, text=f"왼쪽 모터 1: {round(speedleft1, 2)}")
left_label1.pack()

right_label1 = tk.Label(root, text=f"오른쪽 모터 1: {round(speedright1, 2)}")
right_label1.pack()

distance_label = tk.Label(root, text=f"마커까지 거리: {round(distance_to_marker, 2)} cm")
distance_label.pack()

# 긴급 정지 메시지 레이블
emergency_label = tk.Label(root, text=emergency_stop_message, fg="red")
emergency_label.pack()

# 제어 프레임 설정
control_frame = tk.Frame(root)
control_frame.pack()

btn_forward = tk.Button(control_frame, text="전진 (w)", command=increase_speed)
btn_forward.grid(row=0, column=1)

btn_left = tk.Button(control_frame, text="좌회전 (a)", command=steer_left)
btn_left.grid(row=1, column=0)

btn_stop = tk.Button(control_frame, text="정지 (q)", command=stop_motors)
btn_stop.grid(row=1, column=1)

btn_right = tk.Button(control_frame, text="우회전 (d)", command=steer_right)
btn_right.grid(row=1, column=2)

btn_backward = tk.Button(control_frame, text="후진 (s)", command=decrease_speed)
btn_backward.grid(row=2, column=1)

btn_quit = tk.Button(control_frame, text="종료 (x)", command=quit_program)
btn_quit.grid(row=3, column=1)

# 모드 선택 라디오 버튼
mode_var = tk.IntVar()
mode_var.set(1)  # 기본 모드는 원격 제어 모드

mode_frame = tk.Frame(root)
mode_frame.pack()

mode_label = tk.Label(mode_frame, text="모드 선택:")
mode_label.grid(row=0, column=0)

remote_mode = tk.Radiobutton(mode_frame, text="원격 제어", variable=mode_var, value=1)
remote_mode.grid(row=0, column=1)

aruco_mode = tk.Radiobutton(mode_frame, text="ArUco 마커 추적", variable=mode_var, value=2)
aruco_mode.grid(row=0, column=2)

# 모드 변경 추적
mode_var.trace('w', mode_selection_changed)

# 영상 표시 라벨
video_label = tk.Label(root)
video_label.pack()

# 웹캠 및 ArUco 마커 검출 설정
cap = cv2.VideoCapture(0, cv2.CAP_V4L)  # 웹캠 인덱스 0 사용

# LiDAR 화면을 위한 Matplotlib Figure
fig, ax = plt.subplots(figsize=(5, 4))
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# LiDAR 화면 업데이트 함수
def update_lidar_plot():
    if not data_queue.empty():
        angle_data, obstacles, closest_obstacle_dist = data_queue.get()
        if obstacles:
            print(f"Plotting obstacles: {obstacles}")  # 디버깅용
            plot_lidar(angle_data, obstacles, closest_obstacle_dist)

    root.after(100, update_lidar_plot)  # 100 밀리초마다 LiDAR 화면 업데이트

# 키보드 바인딩
root.bind("<w>", lambda event: increase_speed())
root.bind("<s>", lambda event: decrease_speed())
root.bind("<a>", lambda event: steer_left())
root.bind("<d>", lambda event: steer_right())
root.bind("<q>", lambda event: stop_motors())
root.bind("<x>", lambda event: quit_program())

# 영상 피드 업데이트 함수
def update_video_feed():
    _, frame = cap.read()
    if frame is not None:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if mode_var.get() == 2:  # ArUco 모드인 경우에는 마커 오버레이를 그리지 않음
            markers = detect_aruco_markers(frame)
            if markers:
                draw_marker_overlay(frame, markers)
        frame = Image.fromarray(frame)
        frame = ImageTk.PhotoImage(frame)
        video_label.imgtk = frame
        video_label.config(image=frame)
    video_label.after(10, update_video_feed)  # 10 밀리초마다 영상 업데이트

update_video_feed()
update_serial_data()
update_lidar_plot()  # LiDAR 화면 업데이트 시작

root.mainloop()
