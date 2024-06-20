#!/usr/bin/env python

# *** 역할 ***
# 파이어베이스 step이 변경되면 해당 step을 토픽 메시지로 발행하고, 
# step 완료 토픽 메시지(step_done)를 받으면 파이어베이스에 step_done을 기록

import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import rospy
from std_msgs.msg import String, Int32


# 1. 파이어베이스 실시간데이터베이스를 준비
cred = credentials.Certificate("/home/lsj/myagv_ros/src/ros_server_client/script/kairos-teamproject-firebase-adminsdk-qdpoo-b9da88cf13.json")
firebase_admin.initialize_app(cred, {'databaseURL': 'https://kairos-teamproject-default-rtdb.firebaseio.com/'})
ref = db.reference('') # 파이어베이스 실시간데이터베이스에 접근하기 위한 참조점..?
nodes = {} # 아파트 세대들
nodes["101"] = ref.child('101') # 101호
nodes["102"] = ref.child('102') # 102호
nodes["103"] = ref.child('103') # 103호
sedae = None # 어느 세대에서 AGV를 호출했는지를 담는 변수


# 2. 파이어베이스 step이 변경되면 토픽 메시지(step)를 발행
def firebase_event_handler(event):
    print("23")
    global sedae
    sedae = event.path.replace("/","")
    print(sedae)
    # print(event.data['step'])
    if sedae not in ['101', '102', '103']:
        return

    if 'step' not in event.data:
        return

    if '_done' in event.data['step']: # (변경된 step이 _done인 경우 처리하지 않음)
        return
    
    print(f'[Firebase Event] path:{event.path}, data:{event.data}')

    msg = String()
    msg.data = str(event.data['step'])
    pub.publish(msg)
    print(f"[Publish] data:{msg.data}")


# 3. 토픽 메시지(xx_done)를 받으면 파이어베이스 step을 변경
def topic_event_handler(msg):
    if '_done' not in msg.data:
        return
    
    print(f"[Topic Event] data:{msg.data}")

    # Update next step
    step_next = msg.data
    print(f"[Firebase Update] path:{sedae}, data:{step_next}")
    nodes[sedae].update({'step':str(step_next)})


# 4. 시작
print('firebase watcher start')
print("123")
ref.listen(firebase_event_handler)

pub = rospy.Publisher('/to_node', String, queue_size=100) # topic name
sub = rospy.Subscriber("/from_node", String, topic_event_handler) # topic name

rospy.init_node('firebase_watcher', anonymous=True) # node name
rospy.spin()