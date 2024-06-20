#include <Servo.h>

const int trigPin = 9;
const int echoPin = 10;
const int servoPin = 3;

long duration;
int distance;
Servo myServo;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(servoPin);
  myServo.write(0); // 서보 모터 초기 위치
  Serial.begin(9600);
}

void loop() {
  // 초음파 센서로 거리 측정
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // 거리 계산 (cm 단위)
  
  Serial.print("Distance: ");
  Serial.println(distance);
  
  if (distance <= 5) {
    // 서보 모터 동작
    myServo.write(90); // 원하는 각도로 변경 가능
    delay(10000); // 30초 동안 대기
    myServo.write(0); // 원래 위치로 복귀
    delay(30000);
  }
  
  delay(100); // 0.1초마다 거리 측정
}
