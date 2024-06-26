
![제목 없음](https://github.com/lsj324/Smart-Recycling-Hub/assets/170494075/3c78f63c-3ad9-4068-959f-d24e8e7553af)


**목표**

- 쓰레기를 받아 주변 환경을 인식하여 운반하고, 로봇 팔과의 협동 작업을 통해 쓰레기를 분류하는 시스템을 구현

**역할**

- 총 5명이 함께 진행
    - (앱 및 파이어베이스, ROS프레임워크, 로봇팔 제어, YOLO 학습, 기구 설계및 아두이노 제어)
- SLAM을 이용한 이동과 ROS 프레임워크 개발

**문제**

- AGV내의 라즈베리파이로 Slam과 QR detection을 동시에 사용하는데 카메라 딜레이로 인해 실시간으로 사용하기에 어려움이 발생

**해결**

- AGV와 노트북을 ROS통신으로 연결하여 노트북에서 SLAM을, AGV에선 QR detection을 돌리는 방식으로 카메라 딜레이를 해결
![MyAGV + VBOX](https://github.com/lsj324/Smart-Recycling-Hub/assets/170494075/2449f6cb-cff9-4c95-975c-1444aa87d596)

**성과**

- 쓰레기 수거 진행 순서
    1. 사용자가 앱으로 호출한 myAGV가 쓰레기를 수거
    2. 쓰레기장에서 YOLO로 판별(캔, 페트병, 병) 후 로봇팔이 분리수거
    3. 병의 유무에 따라 다르게 진행
        1. 분리수거 이후 병이 있으면 편의점으로 가서 판매
        2. 병이 없으면 사용자 집으로 가서 쓰레기 수거함을 반납하는 과정을 반복함
- 택배 수거 진행 순서
    1. 사용자가 앱으로 호출한 myAGV가 편의점에서 택배를 수거
    2. 사용자의 집으로 택배를 배달
    3. 다시 편의점으로 복귀

**시기**

- 2024.05.08 ~ 2024.06.17 (6주간)

**실행영상**


https://github.com/lsj324/Smart-Recycling-Hub/assets/170494075/36751cf1-edeb-4e64-b373-4fcdd611eebe



https://github.com/lsj324/Smart-Recycling-Hub/assets/170494075/cf4b3a35-9581-4cc9-b703-e1c1dc42e061

