# 세대 화면
# 세대 화면에 속한 버튼, 레이블들을 배치하고 동작을 관리
# 파이어베이스 변경 감지하여 step에 따른 앱 동작 수행
# step에 따른 agv 이동 신호를 발생(상위 노드로 전달)
# 구매 버튼 눌렀을 때의 처리

extends CanvasLayer

signal screen_change
signal ui_action
signal agv_move

@onready var buttons = {
	"recycle": %ButtonRecycle,
	"trash": %ButtonTrash,
	"shopping": %ButtonShopping,
	"delivered": %ButtonDelivered,
}
@onready var data = MyData.new()
@onready var shop = %Shopping
var screen_name = '101'

# 파이어베이스 관련 초기 설정, 포인트 초기 설정
func _ready():
	print("sedae:_ready:", screen_name)
	%Name.text = screen_name
	await watch()
	var doc = await Global.read(screen_name)
	Global.data_sync(doc)
	sync_point()


# 파이어베이스 연결
var is_watching = false
func watch():
	is_watching = true
	await Global.watch(screen_name, _on_db_event)
	

# 파이어베이스 연결 해제
func unwatch():
	is_watching = false
	await Global.unwatch(screen_name, _on_db_event)


# 데이터 변경시 해당 step에 맞는 앱 동작 수행
func _on_db_event(changed):
	print('sedae:'+screen_name+':_on_db_event:', changed.path, ' ', changed.data)
	if 'step' in changed.data:
		var step = changed.data.step
		process_step(step)


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
	set_button_disable('recycle')
	set_button_disable('trash')
	set_button_disable('shopping')
	set_button_disable('delivered')


# 일정시간 대기
func wait(delay):
	if delay == 0:
		delay = 1.5
	#print("delay:", delay)
	await get_tree().create_timer(delay).timeout


# 음성 출력
func voice(num):
	%AudioSedae.stream = voices[num]
	%AudioSedae.play()


var trash_done = null
func process_step(step):
	print("sedae:process_step:", screen_name, ' ', step)
	disable_buttons()
	
	match step:
		"1":
			await wait(0)
			voice(step)
			console_log('-대기중-')
			user_alert_hide(0)
			button_switch('trash', 'recycle')
			button_switch('delivered', 'shopping')
			set_button_enable('recycle')
			set_button_enable('shopping')
			agv_stop('cu', 'wait')
		"2_done":
			voice(step)
			console_log('-호출 완료-')
			user_alert("로봇 호출 완료!")
			trash_done = false
		"3":
			await wait(0)
			voice(step)
			user_alert_hide(3)
			console_log('고객님께 이동하고 있어요.')
			agv_move_to('home')
			button_switch('recycle', 'trash')
			set_button_enable('trash')
		"3_done":
			voice(step)
			console_log('고객님께 도착했어요.')
			user_alert("로봇 도착!\n쓰레기를 확인해주세요.")
			if not trash_done:
				set_button_enable('trash')
			agv_stop('home', 'basket')
		"4":
			await wait(0)
			voice(step)
			user_alert_hide(0)
			console_log('분리수거장으로 이동하고 있어요.')
			agv_move_to('recycle')
		"5_done":
			voice(step)
			console_log('분리수거장에 도착했어요.')
			agv_stop('recycle', 'arrived')
		"6":
			await wait(0)
			voice(step)
			console_log('분리수거를 하고 있어요.')
			agv_stop('recycle', 'bottle_count')
		"6_done":
			voice(step)
			console_log('분리수거를 마쳤어요.')
			agv_stop('recycle', 'bottle_count')
		"7":
			await wait(0)
			voice(step)
			console_log('편의점으로 이동하고 있어요.')
			agv_move_to('cu')
		"8_done":
			voice(step)
			console_log('편의점에 도착했어요.')
			agv_stop('cu', 'arrived')
		"9":
			await wait(0)
			voice(step)
			console_log('빈 병을 수거하고 있어요.')
			agv_stop('cu', 'bottle')
		"9_done":
			voice(step)
			console_log('포인트를 적립했어요.')
			var doc = await Global.read(screen_name)
			Global.data_sync(doc)
			sync_point()
			user_alert("포인트 적립완료!\n" + str(doc.point) + "포인트 사용가능")
		"10":
			await wait(0)
			voice(step)
			console_log('바구니를 반납하러 고객님께 이동하고 있어요.')
			user_alert_hide(3)
			agv_move_to('home')
		"10_done":
			voice(step)
			console_log('고객님께 도착했어요.')
			agv_stop('home', 'basket')
		"11":
			await wait(0)
			voice(step)
			console_log('바구니를 반납하고 있어요.')
			agv_stop('cu', 'load')
		"12":
			await wait(0)
			voice(step)
			console_log('분리수거를 마치고 복귀 중이예요.')
			agv_move_to('cu')
		"12_done":
			voice(step)
			console_log('복귀를 마쳤어요.')
			agv_stop('cu', 'wait')
			
		"22":
			voice(step)
			console_log('주문을 완료했어요.')
			agv_stop('cu', 'load')
		"23":
			await wait(0)
			voice(step)
			console_log('편의점에서 상품을 준비 중이예요.')
			agv_stop('cu', 'load')
		"24_done":
			voice(step)
			console_log('편의점에서 상품을 싣고 출발했어요.')
			agv_stop('cu', 'load')
		"25":
			await wait(0)
			voice(step)
			console_log('고객님께 이동하고 있어요.')
			agv_move_to('home')
		"26":
			await Global.update(screen_name, {"step":"26_done"})
		"26_done":
			voice(step)
			console_log('상품이 도착했어요.')
			user_alert("상품이 도착했어요.")
			agv_stop('home', 'buy_arrive')
		"27":
			await wait(0)
			voice(step)
			console_log('상품을 인수해주세요.')
			user_alert("상품을 인수해주세요.")
			button_switch('shopping', 'delivered')
			set_button_enable('delivered')
			agv_stop('home', 'buy_insu')
		"27_done":
			voice(step)
			console_log('상품을 인수했습니다.')
			shop.reset() # 쇼핑 리셋
			agv_stop('home', 'buy_insu_ok')
		"28":
			await wait(0)
			voice(step)
			console_log('구매를 마치고 복귀 중이예요.')
			agv_move_to('cu')
		"28_done":
			voice(step)
			console_log('복귀를 마쳤어요.')
			agv_stop('cu', 'wait')




# 말풍선
func console_log(txt):
	%Console.text = txt


# 월드 노드의 agv 노드에게 이동 신호 보내기 (앱 상에서 agv 움직임)
func agv_move_to(location):
	agv_move.emit('go_'+location)
	%Image.hide()


# 멈춤 신호 보내기 (앱 상에서 agv 멈춤)
func agv_stop(location, action):
	agv_move.emit('stop')
	%Image.hide()
	

# 화면상의 포인트를 데이터상의 포인트와 일치시킴
func sync_point():
	print("sedae:sync_point:")
	%Point.text = str(data.point)


# 하단 버튼을 다른 버튼으로 교체(수거버튼->배출완료버튼, 구매->닫기버튼,상품인수버튼)
func button_switch(button_name_to_hide, button_name_to_show):
	buttons[button_name_to_show].show()
	buttons[button_name_to_hide].hide()


# 버튼 활성화
func set_button_enable(button_name):
	buttons[button_name].disabled = false


# 버튼 비활성화
func set_button_disable(button_name):
	buttons[button_name].disabled = true
	

# 수거 버튼 눌렀을 때
func _on_button_recycle_pressed():
	print("sedae:_on_recycle:", 'recycle', null)
	#await Global.update(screen_name, {"step":"2_done", "mission":"recycle_trash_not_ready", "bottle":"0"})
	await Global.update(screen_name, {"step":"2_done", "mission":"recycle_trash_not_ready"})


# 배출 완료 버튼 눌렀을 때 
func _on_button_trash_pressed():
	await Global.update(screen_name, {"mission":"recycle_trash_ready"})
	user_alert_hide(0.2)
	set_button_disable('trash')
	trash_done = true
	
# 구매 버튼 눌렀을 때 (팝업 띄움)
func _on_button_shopping_pressed():
	%Popup.show()


# 구매에서 포인트 사용에 따라 화면 포인트 갱신
func _on_shopping_total_point_used_change(total_point_used):
	%Point.text = str(data.point - total_point_used)


# 구매 확인 버튼 눌렀을 때
func _on_shopping_purchase(buy):
	print("sedae:_on_shopping_purchase:", 'buy', buy)
	data.point -= data.total_point_used # 세대 포인트 확정
	data.total_point_used = 0 # 쇼핑에 사용한 포인트 초기화
	await Global.update(screen_name, {"step":"22_done", "mission":"buy", "buy":buy, "point":data.point})
	%Popup.hide()


# 상품인수 버튼 눌렀을 때
func _on_button_delivered_pressed():
	await Global.update(screen_name, {"step":"27_done", 'buy':''})
	user_alert_hide(0.2)


# 구매 닫기 버튼 눌렀을 때
func _on_shopping_close():
	shop.reset()
	sync_point()
	%Popup.hide()


# 홈 버튼 눌렀을 때
func _on_gnb_home():
	_on_shopping_close()
	screen_change.emit('home')

