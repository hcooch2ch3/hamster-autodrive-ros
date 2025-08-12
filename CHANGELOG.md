# Changelog

이 파일에는 프로젝트의 모든 중요한 변경 사항이 기록됩니다.

이 형식은 [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)를 기준으로 작성되었으며,
이 프로젝트는 [Semantic Versioning](https://semver.org/spec/v2.0.0.html)를 따릅니다.

## [1.0.0] - 2025-08-12

### Added
- **햄스터 S 로봇 기본 드라이버** (`hamster_bringup`)
  - Roboid 라이브러리 기반 로봇 제어
  - ROS 2 Twist 메시지 지원
- **AI 카메라 패키지** (`hamster_camera`)
  - 실시간 이미지 스트리밍
  - OpenCV 기반 이미지 처리
- **텔레오퍼레이션** (`hamster_teleop`)
  - 키보드 기반 원격 조작
- **라인 팔로워** (`hamster_line_follower`)
  - 컴퓨터 비전 기반 차선 인식
  - 다중 검출 알고리즘 지원 (Hough, 모멘트, 컨투어, 회귀)
  - 곡선 주행 대응 적응적 차선 폭 추정
  - 실시간 시각화 및 디버깅 정보 표시
- **커스텀 메시지** (`hamster_msgs`)
  - 햄스터 로봇 전용 ROS 2 메시지 타입
- **의존성 관리 체계**
  - `requirements.txt`로 Python 패키지 관리
  - `package.xml` ROS 2 표준 준수
  - 자동화된 설치 스크립트

### Features
- **고급 차선 검출**
  - HSV + BGR 색상 필터링
  - 적응적 임계값 처리
  - 형태학적 연산 기반 노이즈 제거
- **곡선 주행 최적화**
  - 좌우 차선 분리 인식
  - 히스토리 기반 차선 폭 예측
  - PID 제어를 통한 부드러운 조향
- **실시간 시각화**
  - 픽셀 격자 표시
  - 거리 정보 오버레이
  - 검출 상태 실시간 모니터링

### Technical
- ROS 2 Humble 지원
- Ubuntu 22.04 LTS 호환
- Python 3.10+ 지원
- OpenCV 4.5+ 지원

### Documentation
- 상세한 설치 가이드
- 하드웨어 연결 방법
- 사용법 예제
- 로봇 세팅 방법

[1.0.0]: https://github.com/hcooch2ch3/hamster-autodrive-ros/releases/tag/v1.0.0
