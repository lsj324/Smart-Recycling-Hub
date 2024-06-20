# 오토로드로 등록되어 앱 실행시 가장 먼저 로드되는 스크립트
# 로드되면 파이어베이스 인증을 얻고 실시간 데이터베이스 사용 준비
# 파이어베이스 데이터 변경 감지, 데이터 읽기, 쓰기 등의 편의 함수를 제공

extends Node
@onready var data = MyData.new()

var collection = null
var ref : RealtimeReference = null
var docs = {}
var authenticated = false

const firebaseConfig = {
	"apiKey": "...",
	"authDomain": "com.chagokchagok",
	"projectId": "kairos-teamproject",
	"storageBucket": "kairos-teamproject.appspot.com",
	"databaseURL": "https://kairos-teamproject-default-rtdb.firebaseio.com",
	"appId": "1:...",
}


# 파이어베이스 인증을 얻고 실시간데이터베이스 사용을 준비
func _ready():
	print("global _ready")
	Firebase.setup_modules(firebaseConfig)
	
	# realtimeDB 사용 준비1 (인증) (authenticate for realtime database)
	collection = Firebase.Firestore.collection('chagokchagok')
	await collection.get_doc("101").document_got
	authenticated = true
	
	# realtimeDB 사용 준비2 (참조) Note you need to be authenticated for this to work
	ref = Firebase.Realtime.get_realtime_reference()
	docs['101'] = Firebase.Realtime.get_realtime_reference('101app')
	docs['102'] = Firebase.Realtime.get_realtime_reference('102')
	docs['103'] = Firebase.Realtime.get_realtime_reference('103')


func watch(key, fn):
	print(key, ":WATCH")
	while !Global.authenticated :
		await get_tree().create_timer(0.5).timeout
		
	docs[key].new_data_update.connect(fn) # create
	docs[key].patch_data_update.connect(fn) # update
	

func unwatch(key, fn):
	print(key, ":UNWATCH")
	while !Global.authenticated :
		await get_tree().create_timer(0.05).timeout

	docs[key].new_data_update.disconnect(fn) # create
	docs[key].patch_data_update.disconnect(fn) # update
	
	
func update(key, data):
	while !Global.authenticated :
		await get_tree().create_timer(0.05).timeout
		
	docs[key].update(data)


func read(key):
	while !Global.authenticated :
		await get_tree().create_timer(0.05).timeout
		
	print("GLOBAL:READ !!!")
	print(key)
	return await docs[key].get_snapshot()


# 매개변수로 받은 데이터(doc)를 mydata.gd의 static 변수에 저장
func data_sync(doc):
	print("GLOBAL:DATA_SYNC !!!")
	data.bottle = int(doc.bottle)
	data.point = int(doc.point)
	data.buy = doc.buy
