# Hamster Autodrive ROS

햄스터 로봇을 위한 ROS 2 기반 자율주행 시스템

## 개요

이 프로젝트는 햄스터 로봇에 자율주행 기능을 구현하기 위한 ROS 2 패키지입니다. 차선 유지, 장애물 회피, 등의 기능을 제공합니다.

- **대상:** ROS를 배우고 싶은 학생, 교육자, 로봇 입문자
- **목표:** 센서 기반의 간단한 자율주행 알고리즘을 햄스터 로봇에 적용
- **구성:** ROS 2 패키지를 모듈화하여 기능별로 분리

## 주요 기능

- **차선 유지 (Lane Keeping)**: 카메라를 이용한 차선 검출 및 추종
- **장애물 회피 (Obstacle Avoidance)**: LiDAR/초음파 센서를 활용한 장애물 감지 및 회피

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

### 1. 시뮬레이션 실행

```bash
# Gazebo 시뮬레이션 환경 실행
ros2 launch hamster_autodrive simulation.launch.py

# 자율주행 모드 실행
ros2 launch hamster_autodrive autodrive.launch.py
```

### 2. 실제 로봇 실행

```bash
# 로봇 하드웨어 드라이버 실행
ros2 launch hamster_autodrive robot.launch.py

# 자율주행 시작
ros2 run hamster_autodrive autodrive_node
```

### 3. 원격 제어

```bash
# 키보드 원격 제어
ros2 run hamster_autodrive teleop_keyboard

# 게임패드 제어
ros2 run hamster_autodrive teleop_gamepad
```

## 패키지 구조

```
src/
├── hamster_autodrive/          # 메인 자율주행 패키지
├── hamster_description/        # 로봇 URDF 모델
├── hamster_gazebo/            # Gazebo 시뮬레이션
├── hamster_navigation/        # 내비게이션 스택
└── hamster_perception/        # 센서 데이터 처리
```

## 설정

### 카메라 캘리브레이션

```bash
# 카메라 캘리브레이션 실행
ros2 run hamster_perception camera_calibration
```

### 센서 설정

센서 파라미터는 `config/` 디렉토리의 YAML 파일에서 수정할 수 있습니다.

## 개발 참여

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.
