# 상품 항목을 표시(1개)
# +,- 버튼 동작 정의

extends MarginContainer

signal shop_item_cnt_change
@onready var data = MyData.new()

var item_name = ""
var item_price = 0
var cnt = 0

# mydata의 상품정보를 가져와서 이름, 가격, 사진을 표시
func sync_item(index):
	item_name = data.shop_item_list[index].name
	item_price = data.shop_item_list[index].price
	%TextureRect.texture = load('res://goods/' + data.shop_item_list[index].img)
	%ProductName.text = item_name + "\n(" + str(item_price) + ")"


func _ready():
	sync_item(0)


# 구매 초기화시 수량 초기화
func reset():
	cnt = 0
	%Amount.text = str(cnt)


# 수량 증가 불가시 취소(포인트 부족)
func cnt_cancel():
	print('cnt_cancel')
	cnt -= 1
	%Amount.text = str(cnt)
	
# -버튼
func _on_minus_pressed():
	if cnt > 0:
		cnt -= 1
		shop_item_cnt_change.emit(self, -item_price)
		%Amount.text = str(cnt)

# +버튼
func _on_plus_pressed():
	cnt += 1
	shop_item_cnt_change.emit(self, +item_price)
	%Amount.text = str(cnt)


