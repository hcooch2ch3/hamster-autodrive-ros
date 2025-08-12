<img src="media/HAMSTER.PNG" alt="Hamster Robot" width="100%">

## 개요

이 프로젝트는 햄스터 S 로봇을 이용하여 자율주행 기능을 구현하기 위한 ROS 2 패키지입니다. 차선 유지, 장애물 회피, 등의 기능을 제공합니다.

- **대상:** ROS를 배우고 싶은 학생, 교육자, 로봇 입문자
- **목표:** 카메라 및 센서 기반의 간단한 자율주행 알고리즘을 햄스터 로봇에 적용
- **구성:** ROS 2 패키지를 모듈화하여 기능별로 분리

## 주요 기능

- **카메라 비전**: 실시간 이미지 처리 및 객체 검출
- **자율 주행**: 차선을 인식하여 목표 지점까지 차선을 따라 자동으로 이동
- **원격 제어**: 키보드 기반 로봇 원격 조종(teleop)
- **하드웨어 드라이버**: 햄스터 로봇 기본 제어 인터페이스 ([Roboid](https://pypi.org/project/roboid/))

## 시스템 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **Python**: 3.10+
- **하드웨어**:
    - [햄스터 S](https://robomation-shop.co.kr/product/detail.html?product_no=679&cate_no=24&display_group=1)
    - [햄스터 AI 카메라](https://robomation-shop.co.kr/product/detail.html?product_no=1003&cate_no=24&display_group=1)
    - Ubuntu(22.04)가 설치된 컴퓨터
- **센서**: 카메라, 근접 센서, 가속도 센서, 바닥 센서, 조도 센서

## 로봇 세팅 방법

1. 햄스터 동글을 Ubuntu PC에 꽂는다.
2. 햄스터 전원을 키고 동글에 가까이 이동시키면 햄스터와 동글이 연결됨
3. AI 카메라 동글도 마찬가지로 Ubuntu PC에 꽂는다.
4. Ubuntu "Select Network"에서 AI 카메라를 선택한다.
([AI 카메라 연결 방법](https://robomation.net/?p=9974))
5. 햄스터 로봇에 [마운트 키트](https://robomation-shop.co.kr/product/detail.html?product_no=1365&cate_no=24&display_group=1)를 장착하고 AI 카메라를 키트에 장착한다.

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

# ROS 2 패키지 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# Python 패키지 의존성 설치
pip install -r src/hamster-autodrive-ros/requirements.txt
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

## 라인 팔로워 사용법

### 시각화 및 디버깅

라인 팔로워의 검출 결과를 실시간으로 확인할 수 있습니다:

```bash
# 처리된 이미지 확인 (권장)
ros2 run rqt_image_view rqt_image_view /line_follower/processed_image

# 원본 카메라 이미지 확인
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### 시각화 요소 설명

- **🟢 연두색 사각형**: ROI (관심 영역) - 라인을 찾는 구역
- **🔴 빨간색 선들**: 검출된 라인 세그먼트들
- **🔵 파란색 원**: 계산된 라인 중심점 (로봇이 따라가는 목표점)
- **🟡 노란색 세로선**: 이미지 중앙 기준선 (로봇의 진행방향)
- **노란 숫자**: 각 선분의 각도 (디버깅용)

**정상 작동 상태**: 파란 원이 라인 위에 있고, 노란 선 근처에서 안정적으로 움직임

### 실시간 파라미터 조정

라인 팔로워 실행 중에 파라미터를 실시간으로 조정할 수 있습니다:

```bash
# 현재 파라미터 확인
ros2 param list /line_follower
ros2 param get /line_follower brightness_threshold

# 흰색 선 검출 임계값 조정 (180-220 권장)
ros2 param set /line_follower brightness_threshold 190

# 최소 선 길이 조정 (작을수록 짧은 점선도 검출)
ros2 param set /line_follower min_line_length 6

# 점선 간격 허용치 조정 (작을수록 촘촘한 점선 검출)
ros2 param set /line_follower max_line_gap 12

# ROI 영역 조정 (카메라 각도에 따라)
ros2 param set /line_follower roi_height_ratio 0.15  # 관심 영역 높이 (0.1-0.4)
ros2 param set /line_follower roi_y_offset 0.85      # 관심 영역 시작점 (0.6-0.9)

# Canny 엣지 검출 임계값 조정
ros2 param set /line_follower canny_low 35   # 낮을수록 더 많은 엣지 검출
ros2 param set /line_follower canny_high 100 # 높을수록 강한 엣지만 검출

# PID 제어 게인 조정
ros2 param set /line_follower kp 1.0  # 비례 게인 (높을수록 빠른 반응)
ros2 param set /line_follower kd 0.2  # 미분 게인 (높을수록 안정적)
```

### GUI를 이용한 파라미터 조정

더 편리한 GUI 환경에서 파라미터를 조정할 수 있습니다:

```bash
rqt
# Plugins → Configuration → Parameter Reconfigure 선택
```

### 일반적인 조정 가이드

**점선이 잘 안 보일 때:**
- `brightness_threshold` 낮추기 (180 → 160)
- `min_line_length` 낮추기 (8 → 5)
- `max_line_gap` 낮추기 (15 → 10)

**너무 많은 노이즈가 검출될 때:**
- `brightness_threshold` 높이기 (200 → 220)
- `canny_low` 높이기 (40 → 50)

**카메라가 수평에 가까울 때:**
- `roi_height_ratio` 작게 (0.15 이하)
- `roi_y_offset` 크게 (0.85 이상)

## 개발 참여

이 프로젝트에 참여하기 전에 [CONTRIBUTING.md](CONTRIBUTING.md)를 참고해주세요.

### 빠른 시작

1. 저장소를 포크합니다
2. 기능 브랜치를 생성합니다 (`git checkout -b feature/새로운-기능`)
3. 변경사항을 커밋합니다 (`git commit -m 'feat(camera): 새로운 기능 추가'`)
4. 브랜치에 푸시합니다 (`git push origin feature/새로운-기능`)
5. 풀 리퀘스트를 생성합니다

더 자세한 개발 환경 설정, 코딩 스타일, 커밋 규칙 등은 [기여 가이드](CONTRIBUTING.md)에서 확인하실 수 있습니다.

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.
