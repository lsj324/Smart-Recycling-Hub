
# node_back.py
import rospy
import time

from std_msgs.msg import String, Int32

# AGV를 뒤로 이동시키는 함수 정의
def BACK():
    for key in ',_,_o_o_':
        rospy.loginfo("AGV moving back")
        msg = String()
        msg.data = key
        pub_teleop.publish(msg)
        time.sleep(1)

# 토픽 이벤트 핸들러 함수 정의
def topic_event_handler(msg):
    if msg.data not in ['4', '7']:
        return
    
    rospy.loginfo(f"[Topic Event] data:{msg.data}")
    
    BACK()
    
    rospy.loginfo("AGV_back_done")
    
    # 2. QR이 종료되면 토픽 메시지("xx_done")을 발행
    msg.data = msg.data + "_done"
    pub.publish(msg)
    print(f"[Publish] data:{msg.data}")


pub_teleop = rospy.Publisher('/my_teleop_key', String, queue_size=100) # topic name
pub = rospy.Publisher('/from_node', String, queue_size=100) # topic name

sub = rospy.Subscriber("/to_node", String, topic_event_handler) # topic name

rospy.init_node("my_node_back") # node name
rospy.loginfo('my_node_back start')
rospy.spin()
