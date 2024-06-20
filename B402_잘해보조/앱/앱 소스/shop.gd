# 구매 관련 버튼들을 배치하고 동작을 정의
# shop_item 들을 배치

extends Control

signal purchase
signal close
signal total_point_used_change

@export var goods_scene : PackedScene
@onready var shop_item_container = %Items
@onready var data = MyData.new()

# shop_item들을 배치
func _ready():
	for i in range(15):
		var shop_item = goods_scene.instantiate()
		shop_item_container.add_child(shop_item)
		shop_item.sync_item(i)
		shop_item.shop_item_cnt_change.connect(_on_shop_item_cnt_change)

# 구매 초기화
func reset():
	data.total_point_used = 0
	get_tree().call_group("shop_item", "reset")
	
# 구매 확인 버튼 눌렀을 때 (상품 수량이 존재하면 구매 신호 상위로 전달)
func _on_purchase_pressed():
	var buy = ""
	for shop_item in shop_item_container.get_children():
		if shop_item.cnt > 0:
			buy += shop_item.item_name + ":" + str(shop_item.cnt) + ","
	buy += ","
	buy = buy.replace(",,","")
	purchase.emit(buy)

# 닫기 버튼 눌렀을 때 (신호 전달)
func _on_close_pressed():
	close.emit()

# 상품 수량 변경되었을 때(shop_item에서 +,-버튼 클릭했을 때)
func _on_shop_item_cnt_change(shop_item, point_):
	if  point_ > 0 and data.total_point_used + point_ > data.point:
		shop_item.cnt_cancel()
	else:
		data.total_point_used += point_
		total_point_used_change.emit(data.total_point_used)
