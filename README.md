# DDS-QoS-example

DDS QoS 실습 )
로보티즈 example_ws 다운로드 / 빌드 / 실행을 확인한다.
1) 다운로드 : https://github.com/ROBOTIS-Platform/ros2_examples
2) ros2 workspace 생성한 후 소스코드 추가
   예를들어 "robotis_ws/src/" 폴더 하위에 다운받은 파일의 압축을 해제한다.
3) ros2 workspace로 이동
   $ cd ~/robotis_ws
4) 하기 실행으로 소스코드를 빌드
   $ colcon build --symlink-install
5) Source the environment
   $ . install/setup.bash
6) Publisher와 Subscribe 노드를 각각 실행하여 확인한다.
  - publisher : $ ros2 run examples_rclcpp publisher
  - subscribe : $ ros2 run examples_rclcpp subscriber

QoS 실습 예제 추가)
1) 다운로드 : https://github.com/jung-ha-jung/DDS-QoS-example.git
2) 로보티즈 example_ws 파일 대체
   "robotis_ws/src/ros2_examples/examples_rclcpp/src/" 하위의 폴더 중 "publisher"와 "subscriber"를 대체함
4) 하기 실행으로 소스코드를 빌드
   $ colcon build --symlink-install
5) Source the environment
   $ . install/setup.bash
6) 옵션번호를 변경하면서 실행하면 QoS 설정에 따른 데이터 전달을 확인할 수 있음 
     - publisher : $ ros2 run examples_rclcpp publisher -q ?  (? : 옵션번호)
     - subscribe : $ ros2 run examples_rclcpp subscriber -q ?  (? : 옵션번호)

  옵션번호)
  printf("\t6 : Reliability - reliable\n");
  printf("\t7 : Reliability - best_effort\n");
  printf("\t8 : Durability - transient_local\n");
  printf("\t9 : Durability - volatile\n");
  printf("\t10: Durability - KeepAll - transient_local\n");
  printf("\t11 : Deadline - 1000ms\n");
  printf("\t12 : Deadline - 2000ms\n");
  
