# 파이어베이스에서 읽은 데이터를 담는 변수들
# static으로 선언하여 여러 노드에서 직접 사용할 수 있도록 함

class_name MyData extends Node

# 구매 가능한 품목 데이터
const shop_item_list = [
	{'name':'새우깡', 'price':1500, 'img':'1.jpg'},
	{'name':'루비레드', 'price':3000, 'img':'2.png'},
	{'name':'유기농오렌지주스', 'price':2000, 'img':'3.png'},
	{'name':'유기농토마토주스', 'price':2000, 'img':'4.png'},
	{'name':'광동돼지감자', 'price':1000, 'img':'5.png'},
	{'name':'썬업오렌지', 'price':3000, 'img':'6.png'},
	{'name':'캔디', 'price':1000, 'img':'7.png'},
	{'name':'유기농구미베어', 'price':1000, 'img':'8.png'},
	{'name':'우유캔디', 'price':500, 'img':'9.png'},
	{'name':'열라면', 'price':2500, 'img':'10.png'},
	{'name':'참깨라면', 'price':3300, 'img':'11.png'},
	{'name':'김치라면', 'price':2100, 'img':'12.png'},
	{'name':'얼큰쌀국수', 'price':2200, 'img':'13.png'},
	{'name':'해물맛쌀국수', 'price':2200, 'img':'14.png'},
	{'name':'곤약정통짜장면', 'price':4500, 'img':'15.png'},
]

# 파이어베이스의 데이터를 담을 static 변수 : 다른 스크립트에서 접근 가능
static var sedae = ''
static var total_point_used = 0
static var point = 10000
static var bottle = 0
static var buy = ''
static var cnt = 0

func log():
	print("data.point:", point)
	print("data.total_point_used:", total_point_used)
