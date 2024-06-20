# 세대, 편의점 버튼 클릭시 해당 화면으로 전환 요청 신호를 발생

extends CanvasLayer

signal screen_change
signal hosu_change
var screen_name = 'home'

# 씬이 준비되면 버튼 동작 함수를 연결
func _ready():
	%Button101.pressed.connect(_on_nav_pressed.bind('101'))
	%Button102.pressed.connect(_on_nav_pressed.bind('102'))
	%Button103.pressed.connect(_on_nav_pressed.bind('103'))
	%ButtonStore.pressed.connect(_on_nav_pressed.bind('store'))

# 세대, 편의점 버튼 눌렀을 때
func _on_nav_pressed(screen_name):
	screen_change.emit(screen_name)
	if screen_name in ['101', '102', '103']:
		hosu_change.emit(screen_name)
		await Global.read(screen_name)
