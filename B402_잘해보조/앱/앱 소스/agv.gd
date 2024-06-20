# AGV 캐릭터
# 상태에 따른 스프라이트 애니메이션을 관리
# 상태에 따른 이동 방향 관리
# Camera2D 노드를 이용하여 캐릭터가 화면 중앙에 오도록 함

extends CharacterBody2D
var animations = [
	'wait','move','wait','move','wait', # 대기:원위치, 이동:세대, 대기:쓰레기싣기, 이동:쓰레기장, 대기:분리수거,
	'wait','move','wait','wait','wait', # 대기:병개수, 이동:편의점, 대기:빈병수거, 대기:포인트, 대기:상품적재, 
	'move','move','wait','wait','wait', # 이동:세대, 이동:세대, 대기:전달, 대기:완료버튼, 대기:미션완료,
	'move' # 이동:원위치
]

@export var MAX_SPEED = 70
@onready var screen_size = get_viewport_rect().size
@onready var target_pos = global_position


func _ready():
	call_deferred("actor_setup")


func actor_setup():
	await get_tree().physics_frame


func set_animation(ani):
	$AnimatedSprite2D.animation = ani
	$AnimatedSprite2D.play()


# 세대로 이동
var mode = 'stop'
func go_home():
	mode = 'go_home'
	%AnimatedSprite2D.animation = 'run'
	%AnimatedSprite2D.flip_h = false

# 분리수거장으로 이동
func go_recycle():
	mode = 'go_recycle'
	%AnimatedSprite2D.animation = 'run'
	%AnimatedSprite2D.flip_h = true
	
# 편의점으로 이동
func go_cu():
	mode = 'go_cu'
	%AnimatedSprite2D.animation = 'run'
	%AnimatedSprite2D.flip_h = true

# 정지
func stop():
	mode = 'stop'
	%AnimatedSprite2D.animation = 'wait'
	
# 캐릭터 이동
func _physics_process(delta):
	
	# 로드
	match mode:
		'go_home':
			velocity = Vector2(1, 0) * MAX_SPEED
			move_and_slide()
		'go_recycle', 'go_cu':
			velocity = Vector2(-1, 0) * MAX_SPEED
			move_and_slide()
		'stop':
			velocity = Vector2.ZERO
	return
	