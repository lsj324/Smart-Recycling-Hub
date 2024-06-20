# 편의점 화면
# 편의점 화면에 속한 버튼, 레이블들을 배치하고 동작을 관리
# 파이어베이스 변경 감지하여 step에 따른 앱 동작 수행

extends CanvasLayer

signal screen_change
signal robotstatechange
signal ui_action

@onready var buttons = {
	"bottle": %ButtonBottle,
	"load": %ButtonLoad,
}
var screen_name = 'store'
var sedae = ''
var bottle = 0
var point = 0


func _ready():
	print("store:_ready")
	await init_db()
	

# 파이어베이스 연결
func init_db():
	print("store:init_db:DB_READY")
	await Global.watch("101", _on_db_event)
	await Global.watch("102", _on_db_event)
	await Global.watch("103", _on_db_event)
	

# 스텝에 따른 동작
func _on_db_event(changed):
	print('store:_on_db_event:', changed.path, changed.data)
	if 'step' in changed.data:
		sedae = changed.path
		sedae = sedae.replace("app","")
		var step = changed.data.step
		process_step(sedae, step)

# 알림 표시 (UI에서 알림 + 안드로이드 알림)
func user_alert(msg):
	LocalNotification.show("차곡차곡", msg, 1, 10)
	%AlertLabel.text = msg
	var tw1 = create_tween().set_trans(Tween.TRANS_QUAD)
	tw1.tween_property(%Alert, "position:y", 0, 0.5)
	await tw1.finished

# 알림 숨기기
func user_alert_hide(delay):
	await get_tree().create_timer(delay).timeout
	var tw2 = create_tween().set_trans(Tween.TRANS_QUAD)
	tw2.tween_property(%Alert, "position:y", -200, 0.5)
	await tw2.finished

# 버튼 비활성화
func disable_buttons():
	set_button_disable('bottle')
	set_button_disable('load')
	
# 일정시간 대기
func wait(delay):
	if delay == 0:
		delay = 1.5
	await get_tree().create_timer(delay).timeout

# 음성 출력
func voice(num):
	print("voice:",num)
	%AudioStore.stream = load('res://mp3/편의점/' + num + '.mp3')
	%AudioStore.play()
		
# 해당 단계에 맞는 앱 동작 수행
func process_step(sedae, step):
	print("store:process_step:", sedae, ' ', step)
	disable_buttons()
	
	match step:
		"9":
			console_log('병 개수를 확인해주세요.')
			voice(step)
			user_alert("로봇 도착!\n빈 병을 확인해주세요.")
			var doc = await Global.read(sedae)
			bottle = int(doc.bottle)
			point = int(doc.point)
			print(bottle)
			set_bottle(bottle)
			set_button_enable('bottle')
		"9_done":
			console_log('빈 병을 수거했습니다.')
			user_alert_hide(0.2)
		"23":
			console_log('고객이 구매를 요청했어요.')
			voice(step)
			var doc = await Global.read(sedae)
			user_alert("고객이 구매를 요청했어요." + "\n" + doc.buy)
			set_buy(doc.buy)
			await Global.update(sedae, {"step":"23_done"})
		"24":
			console_log('상품을 적재해주세요.')
			voice(step)
			var doc = await Global.read(sedae)
			set_buy(doc.buy)
			user_alert("로봇 도착!\n상품을 적재해주세요.")
			set_button_enable('load')
		"24_done":
			console_log('상품을 적재했습니다.')
			user_alert_hide(0.2)
		
# 화면 중앙의 메시지 표시
func console_log(txt):
	%Console.text = txt

# 버튼 활성화
func set_button_enable(button_name):
	buttons[button_name].disabled = false

# 버튼 비활성화
func set_button_disable(button_name):
	buttons[button_name].disabled = true

# 빈 병 수량
func set_bottle(bottle_):
	print("store:set_bottle:", bottle_)
	%Bottle.text = str(bottle_)
	bottle = int(bottle_)

# 구매 물품 목록
func set_buy(buy):
	print("store:set_buy:", buy)
	buy = buy.replace(",","\n")
	%Buy.text = buy

# 빈병 +
func _on_plus_pressed():
	bottle += 1
	set_bottle(bottle)

# 빈병 -
func _on_minus_pressed():
	if bottle > 0:
		bottle -= 1
	set_bottle(bottle)

# 빈 병 수거 버튼 눌렀을 때
func _on_button_bottle_pressed():
	print("store:_on_button_bottle_pressed:ui_action.emit")
	user_alert_hide(0.2)
	point += bottle * 100
	await Global.update(sedae, {'step':'9_done', 'bottle': str(bottle), "point": str(point)})
	set_bottle(0)

# 상품 적재 버튼 눌렀을 때
func _on_button_load_pressed():
	print("store:_on_button_load_pressed:ui_action.emit")
	user_alert_hide(0.2)
	await Global.update(sedae, {'step':'24_done'})

# 홈 버튼 눌렀을 때
func _on_gnb_home():
	screen_change.emit('home')
