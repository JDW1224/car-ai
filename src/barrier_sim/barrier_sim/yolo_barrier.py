import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import cv2
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory

class YoloBarrierNode(Node):
    def __init__(self):
        super().__init__('yolo_barrier_node')
        
        # 1. Gazebo 차단기 제어 토픽 퍼블리셔 생성
        # Gazebo Harmonic의 JointPositionController가 구독하는 기본 토픽입니다.
        self.cmd_pub = self.create_publisher(
            Float64, 
            '/barrier_cmd', 
            10
        )
        
        # 2. 파일 경로 설정
        pkg_path = get_package_share_directory('barrier_sim')
        weights_path = os.path.join(pkg_path, 'models', 'yolov4-ANPR.weights')
        cfg_path = os.path.join(pkg_path, 'models', 'yolov4-ANPR.cfg')
        video_path = os.path.join(pkg_path, 'models', 'test_video.mp4')
        
        # 3. OpenCV DNN으로 YOLOv4 로드
        self.get_logger().info('YOLO 모델을 불러오는 중...')
        self.net = cv2.dnn.readNet(weights_path, cfg_path)
        self.classes = ['car', 'license_plate']
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
        # 4. 동영상 캡처 객체 초기화
        self.cap = cv2.VideoCapture(video_path)
        
        # 5. 제어 로직용 변수
        self.first_detect_time = 0.0
        self.barrier_open = False
        
        # 타이머 생성 (약 30 FPS 속도로 영상 처리)
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info('AI 비전 제어 노드 시작 완료!')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('영상이 끝났습니다. 영상을 처음부터 다시 재생합니다.')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        height, width, channels = frame.shape
        
        # 이미지를 YOLO 입력 형식(Blob)으로 변환
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        
        car_detected = False
        max_confidence = 0.0

        # YOLO 탐지 결과 분석
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                # 'car'(0번 클래스)이면서 신뢰도가 80% 이상인 경우
                if class_id == 0 and confidence > 0.8:
                    car_detected = True
                    if confidence > max_confidence:
                        max_confidence = confidence
                    
                    # 화면에 박스 그리기 (시연용)
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f"Car: {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ----------------------------------------------------
        # 핵심 로직: 80% 이상 신뢰도로 1초 이상 차량이 감지되면 차단기 개방
        # ----------------------------------------------------
        current_time = time.time()
        if car_detected:
            if self.first_detect_time == 0.0:
                self.first_detect_time = current_time # 처음 감지된 시간 기록
            
            # 감지된 지 1초가 지났다면
            elif current_time - self.first_detect_time > 1.0:
                if not self.barrier_open:
                    self.get_logger().info('>>> 차량 인식 완료! 차단기를 엽니다. <<<')
                    self.barrier_open = True
        else:
            # 차량이 안 보이면 타이머 리셋 및 차단기 닫힘
            self.first_detect_time = 0.0
            if self.barrier_open:
                self.get_logger().info('>>> 차량 통과 확인. 차단기를 닫습니다. <<<')
                self.barrier_open = False

        # Gazebo로 관절 각도 명령 퍼블리시 (-1.57: 열림, 0.0: 닫힘)
        cmd_msg = Float64()
        cmd_msg.data = -1.57 if self.barrier_open else 0.0
        self.cmd_pub.publish(cmd_msg)

        # 화면 출력
        cv2.imshow("AI Parking System (YOLOv4)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloBarrierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
