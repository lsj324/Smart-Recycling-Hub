
![제목 없음](https://github.com/lsj324/Smart-Recycling-Hub/assets/170494075/59c10c64-0643-4650-ae04-9df25f12dd2d)


**목표**

- 쓰레기를 받아 주변 환경을 인식하여 운반하고, 로봇암과의 협동 작업을 통해 쓰레기를 분류하는 시스템을 구현

**팀원**

- 총 5명이 함께 진행
    - (앱 및 파이어베이스, ROS프레임워크, 로봇팔 제어, YOLO 학습, 기구 설계및 아두이노 제어)

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

https://github.com/lsj324/Smart-Recycling-Hub/assets/170494075/09ca8da7-e5b0-4243-b482-c3b6ca5aaff5
https://github.com/lsj324/Smart-Recycling-Hub/assets/170494075/6bcea7c4-13f9-4a65-8457-0ce4f6f083f2

