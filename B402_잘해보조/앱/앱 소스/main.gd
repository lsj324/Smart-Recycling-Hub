# 여러 씬들을 모아놓은 메인 씬
# World 노드에서는 AGV 스프라이트와 배경 이미지 등이 동작함
# UI 노드에서는 버튼, 레이블, 말풍선 등이 동작함

extends Node

var dungeon_scene = preload("res://dungeon.tscn").instantiate()
var road_scene = preload("res://road.tscn").instantiate()
var isometric_scene = preload("res://isometric.tscn").instantiate()
var step = 0
var agv_test = false
var agv = null

@onready var world : Node = %World


# ui씬에서 보내오는 신호를 처리할 함수를 연결
func _ready():
	if agv_test:
		return
	print("main:_ready:")
	%Splash.show()
	%UI.scene_change.connect(_on_scene_change)
	%UI.screens['101'].agv_move.connect(_on_agv_move)
	init_localnotification_ANDROID_ONLY()


# 안드로이드 알림 설정 (LocalNotification 플러그인 : 프로젝트 내보내기에서 설정)
func init_localnotification_ANDROID_ONLY():
	LocalNotification.init()
	LocalNotification.requestPermission()
	await LocalNotification.on_permission_request_completed
	

# 앱 시작 화면
func _on_timer_timeout():
	if agv_test:
		return	
	print('main:_on_timer_timeout')
	await splash()


# 앱 시작 화면
func splash():
	%UI.show()
	var tween = get_tree().create_tween().set_parallel().set_trans(Tween.TRANS_QUAD)
	tween.tween_property(%Splash/Sprite2D, "position:y", 1300, 0.5)
	tween.tween_property(%Splash/Label, "modulate:a", 0.0, 0.8)
	tween.tween_property(%Splash/Label, "position:y", 800, 0.8)
	await tween.finished
	%Splash.queue_free()


# 파이어베이스 데이터 감시 시작
func init_db():
	while !Global.authenticated :
		await get_tree().create_timer(0.2).timeout
	Global.watch("101", _on_db_event)
	Global.watch("102", _on_db_event)
	Global.watch("103", _on_db_event)


# 파이어베이스 데이터 변경 감지시 step값 갱신
func _on_db_event(changed):
	print('main:_on_db_event:', changed.path, changed.data)
	if 'step' in changed.data:
		step = changed.data.step


# 세대 씬 변경시 월드 씬 변경
func _on_scene_change(scene):
	if scene in ['101', '102', '103']:
		%World.add_child(road_scene)
	else:
		%World.remove_child(road_scene)


# ui 노드에서 agv 이동 신호 보내면 월드 노드의 agv에게 전달
func _on_agv_move(type):
	if world.get_children().size() <= 0:
		return
		
	var agv = world.get_children()[0].get_node("%AGV2")
		
	match type:
		"go_home":
			agv.go_home()
		"go_recycle":
			agv.go_recycle()
		"go_cu":
			agv.go_cu()
		"stop":
			agv.stop()


# 안드로이드 알림 관련
func _on_notification_scheduler_notification_opened(notification_id):
	print("NOTIFICATION:opened:", notification_id)


# 안드로이드 알림 관련
func _on_notification_scheduler_permission_granted(permission_name):
	print("NOTIFICATION:granted:", permission_name)


# 안드로이드 알림 관련
func _on_notification_scheduler_permission_denied(permission_name):
	print("NOTIFICATION:denied:", permission_name)
