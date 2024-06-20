# AGV가 이동하는 씬
# AGV를 배경에 배치
# ParallaxLayer를 설정
# 카메라 이동에 따른 배경 스크롤

extends Node2D

func _ready():
	%AGV2.position = %AGVStartMarker.position
	%AGV2.position.y -= 70
	%AGV2/%Camera2D.limit_bottom = 670

func pause():
	pass

func resume(delta):
	pass

