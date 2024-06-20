cmake_minimum_required(VERSION 3.0.2)
project(ros_server_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  script/node_1.py
  script/node_2.py
  script/node_3.py
  script/node_back.py
  script/ros_server.py
  script/node_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(server src/server.cpp)
target_link_libraries(server ${catkin_LIBRARIES})

add_executable(client1 src/client1.cpp)
target_link_libraries(client1 ${catkin_LIBRARIES})

add_executable(client2 src/client2.cpp)
target_link_libraries(client2 ${catkin_LIBRARIES})

add_executable(client3 src/client3.cpp)
target_link_libraries(client3 ${catkin_LIBRARIES})

add_executable(client4 src/client4.cpp)
target_link_libraries(client4 ${catkin_LIBRARIES})

add_executable(client5 src/client5.cpp)
target_link_libraries(client5 ${catkin_LIBRARIES})

add_executable(client6 src/client6.cpp)
target_link_libraries(client6 ${catkin_LIBRARIES})
