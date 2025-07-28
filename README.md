# Hamster Autodrive ROS

햄스터 로봇을 위한 ROS 2 기반 자율주행 시스템

## 개요

이 프로젝트는 햄스터 로봇에 자율주행 기능을 구현하기 위한 ROS 2 패키지입니다. 차선 유지, 장애물 회피, 등의 기능을 제공합니다.

- **대상:** ROS를 배우고 싶은 학생, 교육자, 로봇 입문자
- **목표:** 센서 기반의 간단한 자율주행 알고리즘을 햄스터 로봇에 적용
- **구성:** ROS 2 패키지를 모듈화하여 기능별로 분리

## 주요 기능

- **카메라 비전**: 실시간 이미지 처리 및 객체 검출
- **원격 제어**: 키보드 기반 로봇 원격 조종
- **하드웨어 드라이버**: 햄스터 로봇 기본 제어 인터페이스

## 시스템 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **Python**: 3.10+
- **하드웨어**: 햄스터 로봇, 카메라, 근접 센서, 가속도 센서, 바닥 센서, 조도 센서

## 설치 방법

### 1. ROS 2 Humble 설치

```bash
# ROS 2 Humble 설치
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. 워크스페이스 설정

```bash
# ROS 2 환경 설정
source /opt/ros/humble/setup.bash

# 워크스페이스 생성
mkdir -p ~/hamster_ws/src
cd ~/hamster_ws/src

# 프로젝트 클론
git clone https://github.com/hcooch2ch3/hamster-autodrive-ros.git
```

### 3. 의존성 설치

```bash
cd ~/hamster_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 빌드

```bash
cd ~/hamster_ws
colcon build
source install/setup.bash
```

## 사용 방법

### 1. 통합 시스템 실행 

```bash
# 전체 시스템 (드라이버 + 카메라 + 원격제어)
ros2 launch hamster_bringup hamster.launch.py enable_camera:=true enable_teleop:=true

# 드라이버 + 카메라만
ros2 launch hamster_bringup hamster.launch.py enable_camera:=true

# 드라이버 + 원격제어만
ros2 launch hamster_bringup hamster.launch.py enable_teleop:=true

# 자동 객체 추적 모드
ros2 launch hamster_bringup hamster.launch.py enable_camera:=true enable_object_following:=true

# 드라이버만 (기본값)
ros2 launch hamster_bringup hamster.launch.py
```

### 2. 개별 노드 실행

```bash
# 햄스터 드라이버만
ros2 run hamster_bringup hamster_driver_node

# 카메라 노드만
ros2 run hamster_camera camera_node

# 원격 제어만
ros2 run hamster_teleop teleop_node
```

### 3. 개별 런치 파일

```bash
# 카메라만 실행
ros2 launch hamster_camera camera.launch.py

# 객체 추적 기능 포함 카메라
ros2 launch hamster_camera camera.launch.py enable_object_following:=true
```

## 패키지 구조

```
src/
├── hamster_bringup/           # 햄스터 로봇 기본 드라이버
├── hamster_camera/            # 카메라 비전 처리
└── hamster_teleop/            # 원격 제어
```

### 패키지 설명

- **hamster_bringup**: 햄스터 로봇의 기본 하드웨어 드라이버와 센서 인터페이스
- **hamster_camera**: IP 카메라 연결, 이미지 처리, 객체 검출 기능
- **hamster_teleop**: 키보드를 이용한 원격 조종 기능

## 설정

### 카메라 설정

카메라 IP 주소 및 스트림 설정은 `hamster_camera/camera_node.py`에서 수정할 수 있습니다.

```python
self.camera_url = "http://192.168.66.1:9527/videostream.cgi?loginuse=admin&loginpas=admin"
```

이미지 확인 방법
```bash
# RQT로 이미지 확인
ros2 run rqt_image_view rqt_image_view

# 또는 토픽 목록 확인
ros2 topic list
ros2 topic echo /camera/image_raw
```

### 토픽 확인

```bash
# 카메라 이미지 토픽 확인
ros2 topic list | grep camera

# 이미지 스트림 확인
ros2 run rqt_image_view rqt_image_view
```

## 개발 참여

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.
