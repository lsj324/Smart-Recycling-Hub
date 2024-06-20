# 홈, 세대, 편의점 씬 전환을 담당
# 각 씬 안에서 버튼, 레이블, 말풍선, 알림창 등을 배치하고 동작을 관리

extends CanvasLayer

signal hosu_change
signal store_open
signal scene_change

@export var home_scene : PackedScene
@export var sedae_scene : PackedScene
@export var store_scene : PackedScene
@onready var screens = {
	"home": home_scene.instantiate(), 
	"101": sedae_scene.instantiate(),
	"102": sedae_scene.instantiate(),
	"103": sedae_scene.instantiate(),
	"store": store_scene.instantiate(),
}

# 각 화면에서 오는 신호를 연결
func _ready():
	for key in screens:
		var screen_name = key
		var screen = screens[key]
		screen.screen_name = screen_name
		%Screens.add_child(screen)
		if screen_name in ['home']:
			screen.hosu_change.connect(_on_hosu_change)
		screen.screen_change.connect(_on_screen_change)
		screen.hide()
	_on_screen_change('home')


# 화면 전환 신호를 받았을 때 해당 화면으로 전환하고 파이어베이스 연결 처리
func _on_screen_change(screen_name):
	print("ui:_on_screen_change:", screen_name)
	for node in screens.values():
		node.hide()
	screens[screen_name].show()
		
	if screen_name == 'store':
		store_open.emit()
		
	scene_change.emit(screen_name)
	if screen_name in ['home','store']:
		for name_ in ['101', '102', '103']:
			screens[name_].unwatch()
			
	if screen_name in ['101', '102', '103']:
		for name_ in ['101', '102', '103']:
			screens[name_].unwatch()
		screens[screen_name].watch()
		var doc = await Global.read(screen_name)
		Global.data_sync(doc)


# 세대 바뀜 신호 처리
func _on_hosu_change(hosu):
	print("ui:_on_hosu_change:", hosu)
	screens[hosu].show()
	hosu_change.emit(hosu)


func _on_agv_move(mode):
	print("ui:", mode)
