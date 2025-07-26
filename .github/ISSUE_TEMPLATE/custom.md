---
name: Custom issue template
about: Describe this issue template's purpose here.
title: ''
labels: ''
assignees: ''

---

## 목적
사용자의 키보드 입력을 통해 Hamster 로봇을 원격으로 제어할 수 있도록 teleop 노드를 구현합니다.  
WASD 키 입력을 실시간으로 받아 Twist 메시지를 발행하고, 이를 통해 로봇이 움직이도록 합니다.

## 작업 내용

- [x] 키보드 입력 기반 조작 로직 구현 (WASD)
- [x] `termios`를 이용한 실시간 입력 처리
- [x] 입력된 방향에 따라 `cmd_vel` topic으로 Twist 메시지 발행
- [x] 실행 entry point 등록 (`ros2 run hamster_teleop teleop_node`)

## 참고 사항
- 방향 제어는 linear.x 및 angular.z 조합으로 구현됨
- 입력 차단(blocking) 없이 동작해야 하므로 termios 사용
