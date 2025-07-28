# 기여 가이드 (Contributing Guide)

햄스터 자율주행 ROS 프로젝트에 기여하기 위해서 다음 내용을 참고하여 진행해주세요.

## 목차

- [개발 환경 설정](#개발-환경-설정)
- [브랜치 전략](#브랜치-전략)
- [코딩 스타일](#코딩-스타일)
- [커밋 메시지 규칙](#커밋-메시지-규칙)
- [풀 리퀘스트 가이드](#풀-리퀘스트-가이드)
- [이슈 신고](#이슈-신고)
- [테스트](#테스트)

## 개발 환경 설정

### 필수 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **Python**: 3.10+
- **Git**: 최신 버전

### 환경 설정

1. **저장소 포크 및 클론**
   ```bash
   # GitHub에서 저장소 포크 후
   git clone https://github.com/YOUR_USERNAME/hamster-autodrive-ros.git
   cd hamster-autodrive-ros
   
   # 원본 저장소를 upstream으로 추가
   git remote add upstream https://github.com/hcooch2ch3/hamster-autodrive-ros.git
   ```

2. **ROS 2 환경 설정**
   ```bash
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

3. **의존성 설치**
   ```bash
   cd hamster-autodrive-ros
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **빌드 및 테스트**
   ```bash
   colcon build
   source install/setup.bash
   colcon test
   ```

## 브랜치 전략

### 브랜치 네이밍 규칙

- `feature/기능명`: 새로운 기능 개발
- `fix/버그명`: 버그 수정
- `docs/문서명`: 문서 업데이트
- `refactor/리팩토링명`: 코드 리팩토링
- `test/테스트명`: 테스트 추가/수정

### 작업 플로우

1. **최신 main 브랜치 동기화**
   ```bash
   git checkout main
   git pull upstream main
   ```

2. **기능 브랜치 생성**
   ```bash
   git checkout -b feature/새로운-기능
   ```

3. **개발 및 커밋**
   ```bash
   # 개발 작업 수행
   git add .
   git commit -m "feat(camera): 새로운 객체 검출 알고리즘 추가"
   ```

4. **브랜치 푸시**
   ```bash
   git push origin feature/새로운-기능
   ```

## 코딩 스타일

### Python 코드 스타일

- **PEP8** 준수
- **Black** 포매터 사용 권장
- **타입 힌트** 적극 활용

```python
# 좋은 예
def process_image(self, cv_image: np.ndarray) -> Tuple[np.ndarray, List[Dict]]:
    """이미지 처리 함수"""
    processed = cv_image.copy()
    return processed, []

# 나쁜 예
def process_image(self,cv_image):
    processed=cv_image.copy()
    return processed,[]
```

### ROS 2 패키지 구조

```
패키지명/
├── package.xml          # 패키지 메타데이터
├── setup.py            # Python 패키지 설정
├── setup.cfg           # 설정 파일
├── 패키지명/
│   ├── __init__.py
│   └── 노드명.py        # ROS 노드
├── launch/             # 런치 파일
├── config/             # 설정 파일 (YAML)
├── resource/           # 리소스 파일
└── test/               # 테스트 파일
```

### 주석 및 문서화

- **한국어 주석** 사용
- **docstring**으로 함수/클래스 설명
- **타입 힌트**와 함께 명확한 매개변수 설명

```python
def detect_objects(self, image: np.ndarray, contours: List) -> List[Dict]:
    """
    윤곽선을 기반으로 객체 검출을 수행합니다.
    
    Args:
        image: 처리할 OpenCV 이미지
        contours: 검출된 윤곽선 리스트
        
    Returns:
        검출된 객체 정보 딕셔너리 리스트
    """
```

## 커밋 메시지 규칙

### 형식

```
타입(범위): 제목

본문 (선택사항)

하단 (선택사항)
```

### 타입

- `feat`: 새로운 기능
- `fix`: 버그 수정
- `docs`: 문서 수정
- `style`: 코드 스타일 변경
- `refactor`: 코드 리팩토링
- `test`: 테스트 추가/수정
- `chore`: 빌드 프로세스, 도구 설정 등

### 범위

- `camera`: 카메라 관련
- `bringup`: 드라이버 관련
- `teleop`: 원격 제어 관련
- `msgs`: 메시지 타입 관련
- `launch`: 런치 파일 관련

### 예시

```bash
# 좋은 예
git commit -m "feat(camera): YOLO 기반 객체 검출 알고리즘 추가"
git commit -m "fix(bringup): 시리얼 통신 연결 오류 수정"
git commit -m "docs(readme): 설치 가이드 업데이트"

# 나쁜 예
git commit -m "카메라 수정"
git commit -m "버그픽스"
git commit -m "업데이트"
```

## 풀 리퀘스트 가이드

### PR 생성 전 체크리스트

- [ ] 코드가 정상적으로 빌드되는가?
- [ ] 테스트가 모두 통과하는가?
- [ ] 코딩 스타일을 준수하는가?
- [ ] 관련 문서가 업데이트되었는가?
- [ ] 커밋 메시지가 규칙을 따르는가?

### PR 템플릿

```markdown
## 변경 사항
- 추가된 기능이나 수정된 내용을 간략히 설명

## 테스트 방법
- 변경 사항을 테스트하는 방법
- 예상되는 결과

## 체크리스트
- [ ] 빌드 테스트 완료
- [ ] 기능 테스트 완료
- [ ] 문서 업데이트 완료
- [ ] 코딩 스타일 준수

## 관련 이슈
- Closes #이슈번호
```

### 리뷰 프로세스

1. **자동 검증**: CI/CD 파이프라인 통과
2. **코드 리뷰**: 최소 1명의 리뷰어 승인
3. **테스트**: 기능 및 통합 테스트 확인
4. **머지**: Squash and merge 방식 사용

## 이슈 신고

### 버그 리포트

```markdown
**버그 설명**
발생한 버그에 대한 명확하고 간결한 설명

**재현 방법**
1. 실행한 명령어
2. 설정한 파라미터
3. 발생한 상황

**예상 동작**
원래 어떻게 동작해야 하는지 설명

**실제 동작**
실제로 어떤 일이 발생했는지 설명

**환경 정보**
- OS: Ubuntu 22.04
- ROS: Humble
- Python: 3.10
- 패키지 버전: v1.0.0

**추가 정보**
스크린샷, 로그, 설정 파일 등
```

### 기능 요청

```markdown
**기능 설명**
추가하고 싶은 기능에 대한 설명

**동기 및 배경**
왜 이 기능이 필요한지 설명

**제안하는 해결책**
어떤 방식으로 구현할 수 있을지 제안

**대안**
다른 해결 방법이 있다면 설명
```

## 테스트

### 단위 테스트

```bash
# 특정 패키지 테스트
colcon test --packages-select hamster_camera

# 테스트 결과 확인
colcon test-result --verbose
```

### 통합 테스트

```bash
# 전체 시스템 테스트
ros2 launch hamster_bringup hamster.launch.py enable_camera:=true

# 개별 노드 테스트
ros2 run hamster_camera camera_node
```

### 코드 품질 검사

```bash
# Flake8 린터
flake8 src/

# 타입 체크 (mypy 설치 후)
mypy src/
```

## 질문 및 도움

- **GitHub Discussions**: 일반적인 질문
- **GitHub Issues**: 버그 리포트, 기능 요청
- **이메일**: hcooch2ch3@gmail.com

## 라이선스

이 프로젝트에 기여한 내용이 MIT 라이선스 하에 배포되는 것에 동의합니다.
