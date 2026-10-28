# 자동차인공지능: 비전 기반 주차 차단기 시뮬레이션

2026년 6월 **자동차인공지능** 수업 프로젝트입니다. ROS 2에서 차량·번호판 인식 결과를 Gazebo의 차단기와 차량 이동 제어에 연결합니다.

## 구현 기능

- `yolo_barrier`: OpenCV DNN 기반 YOLOv4 차량 탐지. 신뢰도 0.8 초과 탐지가 1초 이상 유지되면 차단봉을 열고, 차량이 탐지되지 않으면 닫습니다.
- `yolo_ocr_barrier`: 탐지한 번호판 영역에 EasyOCR 한국어·영어 인식을 적용합니다. 등록 번호판과 일치하면 접근 → 개방 → 통과 → 폐쇄 순서로 진행합니다.
- Gazebo launch: `empty.sdf`에 차단기·차량 URDF 모델을 생성하고 `/barrier_cmd`, `/car_cmd`를 ROS–Gazebo 브리지로 연결합니다.

## 구성

```text
src/barrier_sim/
├── barrier_sim/        # YOLO / YOLO + OCR 노드
├── launch/             # Gazebo와 토픽 브리지
├── models/             # YOLO 설정과 클래스명
├── urdf/               # 차단기와 차량 모델
├── test/               # ROS 기본 스타일 검사
├── package.xml
└── setup.py
```

`models/obj.names`의 클래스 순서는 `car`, `license_plate`입니다.

## 실행 전 준비

- Linux 기반 ROS 2, `colcon`, `ament_python`
- ROS 패키지: `rclpy`, `std_msgs`, `ament_index_python`, `launch`, `launch_ros`, `ros_gz_sim`, `ros_gz_bridge`
- Python 모듈: `numpy`, `cv2`(OpenCV), `easyocr`
- GUI를 사용할 수 있는 Gazebo 실행 환경

ROS 배포판과 Python 의존성의 고정 버전은 기록되어 있지 않습니다. 소스에서 Gazebo Harmonic을 언급하므로 ROS–Gazebo 호환성을 확인해야 합니다. `package.xml`의 실행 의존성 선언은 불완전하므로 위 패키지·모듈을 별도로 준비하세요. EasyOCR는 최초 실행 시 모델을 다운로드할 수 있습니다.

가중치와 입력 영상은 별도 준비가 필요합니다. 두 노드가 고정된 이름으로 읽으므로 **빌드 전에** 아래 위치에 배치하세요. 해당 파일들의 출처는 원본에 기록되어 있지 않습니다.

| 파일 | 저장소 내 배치 위치 |
| --- | --- |
| YOLOv4 가중치 | `src/barrier_sim/models/yolov4-ANPR.weights` |
| 입력 영상 | `src/barrier_sim/models/test_video.mp4` |

## 빌드와 실행

ROS 환경을 로드한 Linux 터미널에서 실행합니다. 저장소 폴더명은 `car-ai`로 가정합니다.

```bash
cd car-ai
colcon build --symlink-install --packages-select barrier_sim
source install/setup.bash
ros2 launch barrier_sim gazebo.launch.py
```

별도 터미널에서 같은 ROS 환경을 로드한 후 두 인식 노드 중 하나를 선택합니다.

```bash
cd car-ai
source install/setup.bash
ros2 run barrier_sim yolo_barrier
```

번호판 인가 시나리오는 영상에 맞는 등록 번호판을 `REGISTERED_PLATES` 환경변수로 전달합니다. 쉼표로 여러 번호판을 구분하며, 아래 값은 설정 예시입니다.

```bash
REGISTERED_PLATES='00가0000,00나0000' ros2 run barrier_sim yolo_ocr_barrier
```

환경변수가 비어 있으면 OCR 인가 시나리오가 시작되지 않습니다. 두 노드는 같은 차단기 제어 토픽에 명령을 발행하므로 하나씩 실행하세요.

## 검증 상태와 라이선스

Python 구문은 정적으로 확인했습니다. ROS 2·Gazebo·모델 추론 실행과 실제 인식 성능은 검증하지 않았습니다. `test/`는 ROS 기본 검사 파일입니다.

프로젝트 메타데이터의 라이선스는 `TODO` 상태입니다. 기본 테스트 파일의 기존 저작권·라이선스 주석은 유지되어 있습니다.
