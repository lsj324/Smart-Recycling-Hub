import threading  # 쓰레드를 사용하기 위한 모듈
import time  # 시간 관련 모듈
import firebase_admin  # 파이어베이스 관리자 SDK
from firebase_admin import credentials  # 자격증명 관련 모듈
from firebase_admin import db  # 데이터베이스 관련 모듈
import keyboard  # 키보드 이벤트를 처리하기 위한 모듈
import random  # 랜덤 값 생성을 위한 모듈
import serial  # 시리얼 통신을 위한 모듈
import cv2  # OpenCV를 사용한 컴퓨터 비전 모듈
import numpy as np  # 숫자 배열을 처리하기 위한 모듈
from ultralytics import YOLO  # YOLO 객체 탐지 모델
from ultralytics.utils.plotting import Annotator, colors  # 객체 탐지 결과를 시각화하기 위한 모듈
from pymycobot.mycobot import MyCobot  # MyCobot 로봇 팔 제어 모듈
from collections import defaultdict  # 기본값을 갖는 딕셔너리를 만들기 위한 모듈

# 세대들
nodes = {}  # 파이어베이스 실시간데이터베이스에 접근하기 위한 참조점
ref = None  # 파이어베이스 참조 변수

# 파이어베이스 실시간데이터베이스에 접속하는 부분(한번은 실행해줘야 함)
def init():
    global ref
    # 파이어베이스 자격증명을 통해 데이터베이스에 연결
    cred = credentials.Certificate("C:/Users/user/Desktop/Last 프로젝트/로봇암/파이어베이스/kairos-teamproject-firebase-adminsdk-qdpoo-b9da88cf13.json")
    firebase_admin.initialize_app(cred, {'databaseURL': 'https://kairos-teamproject-default-rtdb.firebaseio.com/'})
    ref = db.reference('')  # 데이터베이스의 루트 참조를 설정
    nodes["101"] = ref.child('101')  # 101 세대 데이터베이스 참조 설정
    nodes["102"] = ref.child('102')  # 102 세대 데이터베이스 참조 설정
    nodes["103"] = ref.child('103')  # 103 세대 데이터베이스 참조 설정

init()

class RobotController:
    def __init__(self, port, baudrate, model_path, arduino_port, arduino_baudrate):
        # MyCobot 로봇 팔 초기화
        self.mc = MyCobot(port, baudrate)
        # 아두이노와 시리얼 통신 설정
        self.arduino = serial.Serial(arduino_port, arduino_baudrate, timeout=1)
        self.is_quit = False  # 쓰레드 종료 플래그
        self.direction_y = None  # y축 방향 변수
        self.direction_x = None  # x축 방향 변수
        self.obj = "No"  # 인식된 객체 초기화
        self.obj_hand = None  # 잡은 객체 초기화
        self.step = 0  # 현재 단계 초기화
        self.mode = 'angles'  # 초기 동작 모드 설정
        self.angles_target = 'middle'  # 초기 목표 각도 설정
        # 현재 로봇 팔의 좌표 가져오기
        self.x, self.y, self.z, self.rx, self.ry, self.rz = self.mc.get_coords()
        # 동작별 각도 설정
        self.angles = {
            'default': [0, 0, 0, 0, 0, 0],
            'agv': [-104.32, -2.98, 107.05, -19.86, -88, -15],
            'middle': [-48.5, -4.92, 45.08, 26.36, -89.03, -11.07],
            'trash_can': [-29.17, 69.25, -10.28, 20.47, -90.26, 11.86],
            'trash_plastic': [22.67, 54.22, 2.72, 11.25, -89.64, -15.38],
        }
        # 객체 인식을 위한 YOLO 모델 로드
        self.model = YOLO(model_path)
        self.names = self.model.model.names  # 모델 클래스 이름 설정
        self.track_history = defaultdict(lambda: [])  # 트랙 기록 초기화
        self.cap = cv2.VideoCapture(0)  # 웹캠 초기화

        # 카운트 변수 초기화
        self.can_count = 0
        self.plastic_count = 0
        self.bottle_count = 0
        self.counted_bottles = False  # 병 개수를 셌는지 여부를 기록하는 플래그
        self.divide_trash = False  # 쓰레기를 나눌 때 사용하는 플래그

        # 타이머 초기화
        self.timer_start = None  # 타이머 시작 시간
        self.timer_duration = 10  # 객체 인식 대기 시간
        self.bottle_timer_start = None  # 병만 인식되는 시간을 기록하기 위한 변수

    # 객체 인식 결과에 따라 obj 설정
    def obj_detect(self):
        if self.obj == 0:
            self.obj = 'can'
            self.can_count += 1  # 캔 카운트 증가
        elif self.obj == 1:
            self.obj = 'plastic'
            self.plastic_count += 1  # 플라스틱 카운트 증가
        elif self.obj == 2:
            self.obj = 'bottle'
            self.obj = "No"  # 병은 잡지 않음
        else:
            self.obj = "No"

    # 카메라 쓰레드 실행
    def camera_thread(self):
        # 객체 이름 매핑
        name_mapping = {
            'can': "Can",
            'plastic': "Plastic",
            'bottle': "Bottle",
            'No': "Unknown"
        }

        while self.cap.isOpened():
            success, frame = self.cap.read()  # 프레임 읽기
            if not success:
                break

            height, width, _ = frame.shape  # 프레임 크기 가져오기
            results = self.model.track(frame, persist=True, verbose=False)  # YOLO 모델로 객체 추적
            boxes = results[0].boxes.xyxy.cpu()  # 객체 경계 상자 좌표
            centers = []  # 객체 중심 좌표 리스트 초기화
            bottle_only = True  # 추가된 부분: 병만 있는지 확인하기 위한 플래그

            if results[0].boxes.id is not None:
                clss = results[0].boxes.cls.cpu().tolist()  # 객체 클래스 리스트
                track_ids = results[0].boxes.id.int().cpu().tolist()  # 트랙 ID 리스트
                confs = results[0].boxes.conf.float().cpu().tolist()  # 신뢰도 리스트

                annotator = Annotator(frame, line_width=2)  # 프레임에 라벨을 그리기 위한 도구

                for box, cls, track_id, conf in zip(boxes, clss, track_ids, confs):
                    if conf >= 0.3:
                        annotator.box_label(box, color=colors(int(cls), True), label=self.names[int(cls)])  # 객체 라벨 그리기
                        center_x = int((box[0] + box[2]) / 2)  # 객체 중심 x 좌표
                        center_y = int((box[1] + box[3]) / 2)  # 객체 중심 y 좌표
                        centers.append((center_x, center_y, int(cls)))  # 중심 좌표 리스트에 추가

                        if cls != 2:  # 병이 아닌 경우 플래그를 False로 설정
                            bottle_only = False

                        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)  # 중심에 원 그리기

                if centers:
                    # 가장 가까운 객체 선택 (병 제외)
                    closest_obj = self.select_closest_non_bottle(centers, width, height)
                    if closest_obj is None:
                        closest_obj = self.select_closest_object(centers, width, height)  # 가장 가까운 객체 선택
                    if self.obj == 'No':  # 객체가 이미 설정된 경우 변경하지 않음
                        self.obj = closest_obj[2]
                        self.obj_detect()
                        print(f"Detected object: {self.obj}")

                    # 객체 표시
                    if self.obj in name_mapping:
                        obj_name = name_mapping[self.obj]
                    else:
                        obj_name = "Unknown"
                    cv2.putText(frame, f"Target: {obj_name}", (closest_obj[0], closest_obj[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    # 타이머 갱신
                    if not bottle_only:
                        self.timer_start = time.time()
                        self.bottle_timer_start = None  # 병만 있는 상태가 아니므로 타이머 초기화
                    elif self.bottle_timer_start is None:
                        self.bottle_timer_start = time.time()

                if self.obj != "No":
                    # 객체 방향 업데이트
                    self.update_direction(closest_obj[0], closest_obj[1], width, height)

            if cv2.waitKey(1) & 0xFF == ord("x") or self.is_quit:
                self.is_quit = True
                time.sleep(0.5)
                break

            cv2.imshow("yolo", frame)  # 프레임 출력

        self.cap.release()
        cv2.destroyAllWindows()

    # 가장 가까운 객체 선택
    def select_closest_object(self, centers, width, height):
        center_camera_x = width // 2  # 카메라 중심 x 좌표
        center_camera_y = height // 2  # 카메라 중심 y 좌표

        min_distance = float('inf')  # 최소 거리 초기화
        closest_obj = None  # 가장 가까운 객체 초기화

        for center_x, center_y, cls in centers:
            distance = np.sqrt((center_x - center_camera_x) ** 2 + (center_y - center_camera_y) ** 2)  # 거리 계산
            if distance < min_distance:
                min_distance = distance
                closest_obj = (center_x, center_y, cls)  # 가장 가까운 객체 업데이트

        return closest_obj

    # 가장 가까운 객체 선택 (병 제외)
    def select_closest_non_bottle(self, centers, width, height):
        center_camera_x = width // 2  # 카메라 중심 x 좌표
        center_camera_y = height // 2  # 카메라 중심 y 좌표

        min_distance = float('inf')  # 최소 거리 초기화
        closest_obj = None  # 가장 가까운 객체 초기화

        for center_x, center_y, cls in centers:
            if cls != 2:  # 병이 아닌 경우
                distance = np.sqrt((center_x - center_camera_x) ** 2 + (center_y - center_camera_y) ** 2)  # 거리 계산
                if distance < min_distance:
                    min_distance = distance
                    closest_obj = (center_x, center_y, cls)  # 가장 가까운 객체 업데이트

        return closest_obj

    # 객체 방향 업데이트
    def update_direction(self, center_x, center_y, width, height):
        center_camera_x = width // 2  # 카메라 중심 x 좌표
        center_camera_y = height // 2  # 카메라 중심 y 좌표

        if center_y < center_camera_y + 120:
            self.direction_y = "front"  # 객체가 카메라 중심보다 위에 있는 경우
        elif center_y > center_camera_y + 140:
            self.direction_y = "backward"  # 객체가 카메라 중심보다 아래에 있는 경우
        else:
            self.direction_y = "good"  # 객체가 카메라 중심과 일치하는 경우

        if center_x < center_camera_x + 50:
            self.direction_x = "left"  # 객체가 카메라 중심보다 왼쪽에 있는 경우
            print(center_x)
        elif center_x > center_camera_x + 60:  #55
            self.direction_x = "right"  # 객체가 카메라 중심보다 오른쪽에 있는 경우
            print(center_x)
        else:
            self.direction_x = "good"  # 객체가 카메라 중심과 일치하는 경우
            print(center_x)

    # 큰 동작 수행
    def big_move(self, target):
        self.mode = 'angles'  # 모드를 각도 모드로 변경
        self.angles_target = target  # 목표 각도 설정
        time.sleep(3)
        print(f"Move complete to {target}")
        # 현재 로봇 팔의 좌표 가져오기
        self.x, self.y, self.z, self.rx, self.ry, self.rz = self.mc.get_coords()
        self.mode = 'standby'  # 모드를 대기 모드로 변경

    # 다음 단계 쓰레드 실행
    def nextstep_thread(self):
        # 각 단계에 대한 설명 설정
        steps = {
            0: '[Robot] Move to myAGV and check bottle count',
            1: '[Robot] Identify can or plastic - depending on obj value',
            2: '[myAGV] Wait for movement',
            3: '[myAGV] Adjust direction - depending on direction value',
            4: '[myAGV] Grasp can or plastic',
            5: '[Trash bin] Grasp can or plastic',
            6: '[Middle point] Move to middle point',
            7: '[Trash bin] Move to trash bin',
            8: '[Trash bin] Place can or plastic',
            9: '[Trash bin] Return to initial position',
            10: '[Middle point] Move to middle point',
            11: '[Robot] Move to trash bin',
        }

        while not self.is_quit:
            step_action = steps.get(self.step, None)  # 현재 단계에 해당하는 설명 가져오기
            if step_action:
                print(f"### Step {self.step} : {step_action}")
                self.execute_step()  # 현재 단계 실행
            else:
                print(f"Invalid step: {self.step}")

    # 각 단계 실행
    def execute_step(self):
        match self.step:
            case 0:
                self.big_move('agv')  # AGV로 이동
                self.obj = 'No'
                self.divide_trash = False
                if not self.counted_bottles:
                    self.count_bottles()  # 병 개수 세기
                    self.counted_bottles = True
                self.step += 1
            case 1:
                # 객체 인식 대기 시간 설정
                self.timer_start = time.time()  # 타이머 시작
                while not self.is_quit:
                    if time.time() - self.timer_start > self.timer_duration:
                        print("No object recognized for 10 seconds, terminating robot.")
                        self.is_quit = True
                        break
                    # 추가된 조건: 캔이나 플라스틱이 없고 병만 10초 동안 인식되면 종료
                    if self.bottle_timer_start is not None and time.time() - self.bottle_timer_start > self.timer_duration:
                        print("Only bottles recognized for 10 seconds, terminating robot.")
                        self.is_quit = True
                        break
                    if self.obj in ['can', 'plastic']:
                        self.obj_hand = self.obj
                        self.step += 1
                        break
                    time.sleep(0.1)

                print(f"Checking object: {self.obj}")
                time.sleep(0.5)
            case 2:
                time.sleep(3)
                self.step += 1
            case 3:
                self.adjust_y_direction()  # y 방향 조정
            case 4:
                self.adjust_x_direction()  # x 방향 조정
            case 5:
                self.approach_to_object()  # 객체에 접근
            case 6 | 11:
                self.big_move('middle')  # 중간 지점으로 이동
                distance = self.read_ultrasonic_sensor()  # 초음파 센서 거리 읽기
                if distance and distance <= 10:
                    self.step = 7 if self.step == 6 else 0
                else:
                    self.step = 0  # 객체를 다시 잡기 위해 0 단계로 돌아감
                    self.arduino.write(b'release\n')
            case 7 | 10:
                if not self.divide_trash:
                    if self.obj_hand == 'can':
                        self.angles_target = 'trash_can'  # 목표 각도를 쓰레기통으로 설정
                        print('Setting target to trash_can')
                    elif self.obj_hand == 'plastic':
                        self.angles_target = 'trash_plastic'  # 목표 각도를 플라스틱 쓰레기통으로 설정
                        print('Setting target to trash_plastic')
                    self.divide_trash = True
                self.big_move(self.angles_target)  # 목표 각도로 이동
                self.step += 1
            case 8:
                self.place_item()  # 객체 배치
            case 9:
                self.open_gripper()  # 그리퍼 열기

    # y 방향 조정
    def adjust_y_direction(self):
        self.mode = 'coords'
        if self.direction_y == 'front':
            self.y += 2.5
            if self.y > 320:
                self.y = 320
                self.direction_y = 'good'
        elif self.direction_y == 'backward':
            self.y -= 2.5
            if self.y < 210:
                self.y = 210
                self.direction_y = 'good'
        elif self.direction_y == 'good':
            self.step += 1
        time.sleep(0.1)

    # x 방향 조정
    def adjust_x_direction(self):
        self.mode = 'coords'
        if self.direction_x == 'left':
            self.x -= 2.5
            if self.x < -160:
                self.x = -160
                self.direction_x = 'good'
        elif self.direction_x == 'right':
            self.x += 2.5
            if self.x > 80:
                self.x = 80
                self.direction_x = 'good'
        elif self.direction_x == 'good':
            self.step += 1
        time.sleep(0.1)

    # 초음파 센서 거리 읽기
    def read_ultrasonic_sensor(self):
        self.arduino.write(b'read_distance\n')
        time.sleep(0.1)
        if self.arduino.in_waiting > 0:
            distance = self.arduino.readline().decode().strip()
            return float(distance)
        return None

    # 객체에 접근
    def approach_to_object(self):
        self.mode = 'coords'
        self.z = 90
        time.sleep(1)
        while True:
            distance = self.read_ultrasonic_sensor()  # 초음파 센서 거리 읽기
            if distance and distance > 5:
                self.z -= 5
                time.sleep(0.1)
                if self.z < -15:
                    self.z = 50
            elif distance and distance <= 5:
                self.z -= 40
                self.arduino.write(b'grip\n')
                time.sleep(3)
                self.step += 1
                break

    # 병 개수 세기
    def count_bottles(self):
        # 병 개수 초기화
        self.bottle_count = 0
        
        success, frame = self.cap.read()
        if success:
            height, width, _ = frame.shape
            results = self.model(frame)
            boxes = results[0].boxes.xyxy.cpu()
            centers = []

            if results[0].boxes.id is not None:
                clss = results[0].boxes.cls.cpu().tolist()
                confs = results[0].boxes.conf.float().cpu().tolist()

                for box, cls, conf in zip(boxes, clss, confs):
                    if conf >= 0.3:
                        center_x = int((box[0] + box[2]) / 2)
                        center_y = int((box[1] + box[3]) / 2)
                        centers.append((center_x, center_y, int(cls)))

                bottle_count = sum(1 for center_x, center_y, cls in centers if cls == 2)  # 병 개수 세기
                self.bottle_count = bottle_count
                print(f"Bottle Count: {self.bottle_count}")

    # 객체 잡기
    def grasp_item(self):
        self.mode = 'gripper_close'
        time.sleep(3)
        self.mode = 'coords'
        self.z = 70
        time.sleep(1)
        self.z = 40
        time.sleep(3)
        self.arduino.write(b'grip\n')
        self.step += 1
        time.sleep(5)

    # 객체 배치
    def place_item(self):
        self.mode = 'coords'
        self.z -= 70
        time.sleep(1)
        self.step += 1

    # 그리퍼 열기
    def open_gripper(self):
        self.mode = 'gripper_open'
        time.sleep(0.5)
        self.arduino.write(b'release\n')
        self.step += 1
        time.sleep(2)

    # 로봇 팔 쓰레드 실행
    def cobot_thread(self):
        while not self.is_quit:
            print(self.mode)
            match self.mode:
                case 'standby':
                    time.sleep(0.1)
                case 'angles':
                    self.mc.sync_send_angles(self.angles[self.angles_target], speed=50, timeout=0.5)
                    self.x, self.y, self.z, self.rx, self.ry, self.rz = self.mc.get_coords()
                    print('TARGET ANGLES: ', self.angles_target)
                    time.sleep(0.5)
                case 'coords':
                    self.mc.sync_send_coords([self.x, self.y, self.z, self.rx, self.ry, self.rz], speed=50, mode=1, timeout=0.1)
                    print(f"Moving to coords: x={self.x}, y={self.y}, z={self.z}, rx={self.rx}, ry={self.ry}, rz={self.rz}")
                    time.sleep(0.1)
                case 'gripper_open':
                    time.sleep(1)
                    self.mode = 'standby'
                case 'gripper_close':
                    time.sleep(1)
                    self.mode = 'standby'

        print("cobot thread end...")

    # 로봇 컨트롤러 시작
    def start(self):
        t1 = threading.Thread(target=self.cobot_thread, daemon=True)  # 로봇 팔 제어 쓰레드
        t2 = threading.Thread(target=self.nextstep_thread, daemon=True)  # 다음 단계 쓰레드
        t3 = threading.Thread(target=self.camera_thread, daemon=True)  # 카메라 쓰레드
        t1.start()
        t2.start()
        t3.start()
        t3.join()  # 카메라 쓰레드 종료 대기

        # 병 개수 출력
        print(f"Can Count: {self.can_count}")
        print(f"Plastic Count: {self.plastic_count}")
        print(f"Bottle Count: {self.bottle_count}")

# COBOT
def cobot_recycle(sedae):
    print("Executing trash separation action...")
    robot_controller = RobotController('COM5', 115200, '로봇암/Finalll/winter/best.pt', 'COM6', 9600)
    robot_controller.start()
    print("end...")

    bottle = robot_controller.bottle_count  # RobotController에서 병 개수 가져오기
    print(f"Recognized bottles: {bottle} count")

    next_step = 7  # 다음 단계: 편의점으로 이동 (AGV)

    # 파이어베이스 실시간 데이터베이스에 데이터 업데이트
    nodes[sedae].update({'step': str(next_step), 'bottle': str(bottle)})

# 파이어베이스 실시간 데이터베이스 변경 시 실행되는 함수 (ref.listen()에 등록된 콜백 함수)
def listener(event):
    print("---------------------------------------")
    print(event.event_type, event.path, event.data)
    sedae = event.path.replace("/", "")

    if sedae not in ['101', '102', '103']:
        return

    if 'step' in event.data:  # 변경된 데이터에 step이 포함된 경우
        data = nodes[sedae].get()  # 파이어베이스 실시간 데이터베이스에서 데이터 읽기
        step = data['step']
        mission = data['mission']

        if step == '6':  # 특정 단계에서 cobot_recycle 실행
            cobot_recycle(sedae)

ref.listen(listener)

# 테스트용: 숫자 키가 눌렸을 때 파이어베이스 step 데이터를 해당 숫자 값으로 업데이트
keyboard.add_hotkey('1', lambda: nodes['101'].update({'step': '1'}))
keyboard.add_hotkey('2', lambda: nodes['101'].update({'step': '2'}))
keyboard.add_hotkey('3', lambda: nodes['101'].update({'step': '3'}))
keyboard.add_hotkey('4', lambda: nodes['101'].update({'step': '4'}))
keyboard.add_hotkey('5', lambda: nodes['101'].update({'step': '5'}))
keyboard.add_hotkey('6', lambda: nodes['101'].update({'step': '6'}))
keyboard.add_hotkey('7', lambda: nodes['101'].update({'step': '7'}))
keyboard.add_hotkey('8', lambda: nodes['101'].update({'step': '8'}))
keyboard.add_hotkey('9', lambda: nodes['101'].update({'step': '9'}))
keyboard.wait()
print("...")

while True:
    time.sleep(1)  # 메인 쓰레드 유지
