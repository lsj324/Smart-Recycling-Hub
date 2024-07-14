import cv2
import numpy as np
import threading
import time
import rospy
from std_msgs.msg import String, Int32
# from pymycobot.myagv import MyAgv

# AGV 객체를 생성 및 포트 연결 : myagv_odometry 쪽과 ttyAMA2 충돌함
# agv = MyAgv("/dev/ttyAMA2", 115200)
prevcommand = None
front_back = False
angle_move = False
action = None
is_quit = False
no_qr_count = 0  # QR 코드가 감지되지 않은 프레임의 카운트

def send_stop():
    msg = String()
    msg.data = '_'
    pub_teleop.publish(msg)
    

# QR 코드 디코딩 함수
def qr_decode(frame):
    global prevcommand, front_back, angle_move, no_qr_count
    qr_detector = cv2.QRCodeDetector()
    retval, points = qr_detector.detect(frame)

    if not retval:
        no_qr_count += 1
        print(no_qr_count)
        if no_qr_count > 30:  # QR 코드가 30프레임 동안 감지되지 않으면 완료 처리
            send_stop()
            print("complete")
            return "complete"
    else:
        no_qr_count = 0
        points = points[0]
        
        # QR 코드 모서리 좌표 계산
        top_left = tuple(points[0])
        top_right = tuple(points[1])
        bottom_right = tuple(points[2])
        bottom_left = tuple(points[3])
        
        # 왼쪽과 오른쪽 수직선의 길이 계산
        left_length = np.linalg.norm(np.array(bottom_left) - np.array(top_left))
        right_length = np.linalg.norm(np.array(bottom_right) - np.array(top_right))

        # QR 코드의 중심 x 좌표를 계산
        center_line_x = frame.shape[1] // 2
        cx = int((top_left[0] + top_right[0]) // 2)

        if not angle_move:
            if abs(left_length - right_length) > 20:
                if left_length > right_length:
                    prevcommand = "rotate_left"
                    return "rotate_left"
                else:
                    prevcommand = "rotate_right"
                    return "rotate_right"
            else:
                angle_move = True
                send_stop()
                return "stop"
        else:
            if not front_back:
                if cx < center_line_x - 20:
                    prevcommand = "left"
                    # print("left")
                    return "left"
                elif cx > center_line_x + 20:
                    prevcommand = "right"
                    # print("right")
                    return "right"
                elif cx > center_line_x - 20 and cx < center_line_x + 20:
                    front_back = True
                    # agv.stop()
                    send_stop()
                    # print("stop")
                    return "stop"
            else:
                prevcommand = "forward"
                # print("go_ahead")
                return "forward"
            
    return None

# 카메라 함수
cap = None
def camera_thread():
    global is_quit, action, cap, no_qr_count
    cap = cv2.VideoCapture(0)
    
    # 해상도를 낮춰서 처리 속도 향상
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    # qr 카운터 초기화
    no_qr_count = 0
    
    while not is_quit:
        ret, frame = cap.read()

        if not ret:
            print("Camera error")
            break

        action = qr_decode(frame)

        if action == "complete":
            is_quit = True
            break
        
        # cv2.imshow('Frame', frame)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     is_quit = True
        #     break

    cap.release()
    cv2.destroyAllWindows()

# AGV 제어 함수
def agv_thread():
    global action, is_quit
    while not is_quit:
        if action:
            move_agv(action)
            action = None
        time.sleep(0.1)

# AGV 이동 함수
def move_agv(action):
    if action == "rotate_right":
        # agv.pan_right1(1)
        print("AGV rotating right")
        msg = String()
        msg.data = 'o'
        pub_teleop.publish(msg)
    elif action == "rotate_left":
        # agv.pan_left1(1)
        print("AGV rotating left")
        msg = String()
        msg.data = 'u'
        pub_teleop.publish(msg)
    elif action == "right":
        # agv.pan_right1(1)
        print("AGV moving right")
        msg = String()
        msg.data = 'l'
        pub_teleop.publish(msg)
    elif action == "left":
        # agv.pan_left1(1)
        print("AGV moving left")
        msg = String()
        msg.data = 'j'
        pub_teleop.publish(msg)        
    elif action == "forward":
        # agv.go_ahead1(1)
        print("AGV moving forward")
        msg = String()
        msg.data = 'i'
        pub_teleop.publish(msg)     
    elif action == "stop":
        # agv.stop()
        print("AGV stopped")
        msg = String()
        msg.data = '_'
        pub_teleop.publish(msg)        

def QR():
    global is_quit
    rospy.loginfo("Starting QR code detection and AGV control")
    is_quit = False
    agv_thread_inst = threading.Thread(target=agv_thread, daemon=True)
    agv_thread_inst.start()
    camera_thread()
    agv_thread_inst.join()

# 1. 토픽 메시지("xx")를 받으면 QR을 실행
def topic_event_handler(msg):
    if msg.data not in ['3qr', '5qr','10qr', '25qr']: # topic message
        return
    step = msg.data
    
    print(f"[Topic Event] data:{msg.data}")

    print(".....................")
    print("QR")
    print(".....................")

    QR()
    
    time.sleep(15)
    
    # 2. QR이 종료되면 토픽 메시지("xx_done")를 발행
    msg.data = step.replace("qr","") ## (수정-추가)
    msg.data = msg.data + "_done"
    pub.publish(msg)
    print(f"[Publish] data:{msg.data}")

# teleop로 메시지 발행
pub_teleop = rospy.Publisher('/my_teleop_key', String, queue_size=100) # topic name

# ros_server.py로 메시지 발행
pub = rospy.Publisher('/from_node', String, queue_size=100) # topic name

# qr_position_MJH.py로 메시지 수신
sub = rospy.Subscriber("/to_qr", String, topic_event_handler) # topic name

try:
    print('my_node_qr start')
    rospy.init_node("my_node_qr") # node name
    rospy.spin()
finally:
    print("my_node_qr done.")
