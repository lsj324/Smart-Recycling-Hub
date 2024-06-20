int In1 = 4;      // 제어신호 1핀
int In2 = 5;      // 제어신호 2핀
int In3 = 7;
int In4 = 8;
int SpeedPin_A = 6;    // PWM제어를 위한 핀
int SpeedPin_B = 9; 
const int trigPin = 10; // 초음파 센서 trig 핀
const int echoPin = 11; // 초음파 센서 echo 핀

void setup() {
  pinMode(In1, OUTPUT); // 제어 1번핀 출력모드 설정
  pinMode(In2, OUTPUT); // 제어 2번핀 출력모드 설정
  pinMode(SpeedPin_A, OUTPUT); // PWM제어핀 출력모드 설정
  pinMode(In3, OUTPUT); // 제어 1번핀 출력모드 설정
  pinMode(In4, OUTPUT); // 제어 2번핀 출력모드 설정
  pinMode(SpeedPin_B, OUTPUT); // PWM제어핀 출력모드 설정
  pinMode(trigPin, OUTPUT); // 초음파 센서 trig 핀 설정
  pinMode(echoPin, INPUT); // 초음파 센서 echo 핀 설정
  Serial.begin(9600); // 시리얼 통신 시작
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // 공백 제거

    if (command.equals("grip")) {
      digitalWrite(In1, HIGH); // 모터가 시계 방향으로 회전
      digitalWrite(In2, LOW);
      analogWrite(SpeedPin_A, 200); // 모터 속도를 최대로 설정
      delay(100); // 모터를 1초 동안 동작
      digitalWrite(In3, HIGH); // 모터가 시계 방향으로 회전
      digitalWrite(In4, LOW);
      analogWrite(SpeedPin_B, 200); // 모터 속도를 최대로 설정
      delay(100);
    } else if (command.equals("release")) {
      digitalWrite(In1, HIGH); // 모터가 시계 방향으로 회전
      digitalWrite(In2, LOW);
      analogWrite(SpeedPin_A, 0); // 모터 속도를 최대로 설정
      delay(100); // 모터를 1초 동안 동작
      digitalWrite(In3, HIGH); // 모터가 시계 방향으로 회전
      digitalWrite(In4, LOW);
      analogWrite(SpeedPin_B, 0); // 모터 속도를 최대로 설정
      delay(100);
    } else if (command.equals("read_distance")) {
      long duration, distance;
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = (duration / 2) / 29.1;
      Serial.println(distance);
    }
  }
}
