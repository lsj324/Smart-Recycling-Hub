#!/usr/bin/env python3
# ROS와 관련된 라이브러리 임포트
import rospy
import time
import actionlib
import os, sys

from math import radians, sin, cos
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult, MoveBaseFeedback
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String, Int32

# 피드백 콜백 함수 정의
def feedback_callback(msg):
    rospy.loginfo("[Feedback] Going to Goal Pose...")

# 오일러 각도를 쿼터니언으로 변환하는 함수 정의
def euler_to_quaternion(yaw):
    q = Quaternion()
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q

# 목표 위치로 이동하는 함수 정의
def Send_Goal():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'

    goal.target_pose.pose.position.x = 0.20
    goal.target_pose.pose.position.y = -2.39
    goal.target_pose.pose.position.z = 0.0

    yaw = -0.23
    quaternion = euler_to_quaternion(yaw)
    # quaternion = euler_to_quaternion()
    goal.target_pose.pose.orientation = quaternion

    client.send_goal(goal, feedback_cb=feedback_callback)
    client.wait_for_result()




# 1. 토픽 메시지("xx")를 받으면 Send_Goal을 실행
def topic_event_handler(msg):
    if msg.data not in ['8', '12', '28']: # topic message
        return
    
    rospy.loginfo(f"[Topic Event] data:{msg.data}")

    rospy.loginfo(".....................")
    rospy.loginfo("Send_Goal : CU")
    rospy.loginfo(".....................")
    
    Send_Goal() ## (임시 주석)
    rospy.loginfo("Send_Goal_done")
    
    # 2. Send_Goal이 종료되면 토픽 메시지("xx_done")을 발행
    msg.data = msg.data + "qr"
    pub.publish(msg)
    rospy.loginfo(f"[Publish] data:{msg.data}")


pub = rospy.Publisher('/to_qr', String, queue_size=100) # topic name
sub = rospy.Subscriber("/to_node", String, topic_event_handler) # topic name

rospy.init_node("my_node_cu") # node name
rospy.loginfo('my_node_cu start')
rospy.spin()
