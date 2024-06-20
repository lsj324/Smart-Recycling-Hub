import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import keyboard
import random


# *** 역할 ***
# 1. 다음을 반복
#   a. 파이어베이스에 step_done 형식의 데이터가 입력되면... 
#   b.   → 다음 스텝을 결정하는 로직을 실행
#   c. 다음 스텝이 결정되면...
#   d.   → 파이어베이스에 step 형식의 데이터를 입력
# 2. (참고)
#   a. (파이어베이스에 step 형식의 데이터가 입력되면...)
#   b.   → (AGV, COBOT, APP에서 처리)


# FB realtimeDB 준비
cred_path = "./Godot/ChagokChagok/ChagokChagokApp/firebase-adminsdk.json"
databaseURL = 'https://kairos-teamproject-default-rtdb.firebaseio.com/'
cred = credentials.Certificate(cred_path)
firebase_admin.initialize_app(cred, {'databaseURL': databaseURL})
ref = db.reference('')


# 아파트 세대별 realtimeDB 문서
nodes = {} # 아파트 세대들
nodes["101"] = ref.child('101app') # 101호
nodes["102"] = ref.child('102') # 102호
nodes["103"] = ref.child('103') # 103호


# 다음 스텝을 결정하는 함수 : FB realtimeDB 데이터가 변경되면 실행됨
def listener(event):
    
    # 어떤 아파트 세대에서 데이터가 변경되었는지 판단
    sedae = event.path.replace("/","")
    
    # -변경된 데이터가 'step_done'인 경우만 처리
    # _done이 아닌 스텝은 AGV, COBOT, APP에서 처리
    is_return = False
    if 'step' not in event.data:
        is_return = True
    
    if 'step' in event.data:
        if '_done' not in event.data['step']:
            is_return = True
        
    if is_return:
        return
    
    # FB realtimeDB의 데이터를 읽음
    data = nodes[sedae].get() 
    mission = data['mission']
    bottle = int(data['bottle'])
    
    # AGV 등에서 보내온 step_done만 처리
    step_done = data['step']
    step_done = step_done.replace("_done", "")
    next_step = None
        
    # 3단계 완료시 고객이 앱에서 완료버튼 누를 때까지 대기
    if step_done == '3':
        if mission == 'recycle_trash_not_ready':
            while True:
                data = nodes[sedae].get()
                mission = data['mission']
                if mission != 'recycle_trash_ready':
                    time.sleep(5)
                else:
                    break
            next_step = '4'
        elif mission == 'recycle_trash_ready':
            next_step = '4'

    # 7단계 완료시 빈 병 여부에 따라 분기
    elif step_done == '7':
        if bottle > 0:
            next_step = '8'
        else:
            next_step = '10'
    
    # 10단계 완료시 12단계 시작
    elif step_done == '10':
        next_step = '12'      
    
    # 프로세스 끝에 도달시 1단계로
    elif step_done in ['12', '28']:
        next_step = '1'
    
    # 그 외의 경우 현재 스텝의 다음(+1) 스텝을 진행
    elif step_done in ['2','4','5','6','8','9','11','22','23','24','25','26','27']:
        next_step = int(step_done) + 1
    
    # 다음 스텝이 결정된 경우 FB realtimeDB에 갱신
    if next_step:
        print("update:next_step:", next_step)
        nodes[sedae].update({'step':str(next_step)})
    return


# 위 함수를 FB realtimeDB에 연결                            
ref.listen(listener)

while True:
    time.sleep(1)
