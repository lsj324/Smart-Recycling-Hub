# 토픽을 받아서 AGV를 움직이는 코드 : myagv_teleop.py 수정

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('myagv_teleop')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Twist

# 키보드 입력을 설명하는 메시지
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

# 키와 동작을 매핑하는 딕셔너리
moveBindings = {
        'i':(1,0,0,0),
        'o':(0,0,0,-1),
        'j':(0,1,0,0),
        'l':(0,-1,0,0),
        'u':(0,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,-1),
        'm':(-1,0,0,1),
        'O':(0,0,0,-1),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(0,0,0,1),
        '<':(-1,0,0,0),
        '>':(-1,0,0,-1),
        'M':(-1,0,0,1),
    }

# 속도 변경 키와 동작을 매핑하는 딕셔너리
speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

# 메시지를 게시하는 스레드 클래스 정의
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    # 구독자가 연결될 때까지 대기
    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    # 상태를 업데이트
    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.condition.notify()
        self.condition.release()

    # 스레드 중지
    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    # 스레드 실행
    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            self.condition.wait(self.timeout)

            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()
            self.publisher.publish(twist_msg)

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

# 터미널 설정 저장
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

# 터미널 설정 복원
def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# 현재 속도와 회전 속도를 문자열로 반환
def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

settings = saveTerminalSettings()

speed = rospy.get_param("~speed", 0.1)
turn = rospy.get_param("~turn", 0.5)
speed_limit = rospy.get_param("~speed_limit", 1.0)
turn_limit = rospy.get_param("~turn_limit", 1.0)
repeat = rospy.get_param("~repeat_rate", 0.0)
key_timeout = rospy.get_param("~key_timeout", 0.52)
stamped = rospy.get_param("~stamped", False)
twist_frame = rospy.get_param("~frame_id", '')
if stamped:
    TwistMsg = TwistStamped

pub_thread = PublishThread(repeat)

x = 0
y = 0
z = 0
th = 0
status = 0
  
# 1. 토픽 메시지로 키를 받으면 myagv_teleop.py에서 키를 누른 것과 같은 동작
my_teleop_key_topic_msg_data = None
def topic_event_handler(msg):
    global my_teleop_key_topic_msg_data
    global x, y, z, th, status, speed, turn, speed_limit, turn_limit, repeat, key_timeout, stamped, twist_frame, TwistMsg

    print(f"[Topic Event] data:{msg.data}")
    print(".....................")
    print("teleop")
    print(".....................")
    
    key = msg.data
    if key in moveBindings.keys():
        x = moveBindings[key][0]
        y = moveBindings[key][1]
        z = moveBindings[key][2]
        th = moveBindings[key][3]
    elif key in speedBindings.keys():
        speed = min(speed_limit, speed * speedBindings[key][0])
        turn = min(turn_limit, turn * speedBindings[key][1])
        if speed == speed_limit:
            print("Linear speed limit reached!")
        if turn == turn_limit:
            print("Angular speed limit reached!")
        print(vels(speed, turn))
        if status == 14:
            print(msg)
        status = (status + 1) % 15
    else:
        x = 0
        y = 0
        z = 0
        th = 0
    pub_thread.update(x, y, z, th, speed, turn)

from std_msgs.msg import String, Int32

sub = rospy.Subscriber("/my_teleop_key", String, topic_event_handler)

print('my_node_teleop start')
rospy.init_node("my_node_teleop")

try:
    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)

    print(msg)
    print(vels(speed, turn))

    rospy.spin()

except Exception as e:
    print(e)

finally:
    pub_thread.stop()
    restoreTerminalSettings(settings)