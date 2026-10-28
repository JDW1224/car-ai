import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import cv2
import numpy as np
import time
import os
import easyocr
from ament_index_python.packages import get_package_share_directory


class YoloOcrBarrierNode(Node):
    # ---- 연출 파라미터 (가제보 좌표, 단위 m / rad) ----
    CAR_START  = -9.0    # 차 대기 위치
    CAR_WAIT   = -2.2    # 차단기 바로 앞 정지 위치
    CAR_EXIT   =  8.0    # 통과 후 빠져나가는 위치
    OPEN_ANGLE = -1.57   # 차단봉 개방 각도
    CLOSE_ANGLE = 0.0    # 차단봉 폐쇄 각도
    SPEED_DRIVE = 3.0    # 주행 속도(m/s)
    SPEED_RESET = 9.0    # 대기위치 복귀 속도(m/s)
    OPEN_WAIT   = 1.2    # 차단봉이 다 올라갈 때까지 대기(s)
    CLOSE_WAIT  = 1.2    # 폐쇄 후 대기(s)

    def __init__(self):
        super().__init__('yolo_ocr_barrier_node')

        self.barrier_pub = self.create_publisher(Float64, '/barrier_cmd', 10)
        self.car_pub = self.create_publisher(Float64, '/car_cmd', 10)

        pkg_path = get_package_share_directory('barrier_sim')
        weights_path = os.path.join(pkg_path, 'models', 'yolov4-ANPR.weights')
        cfg_path = os.path.join(pkg_path, 'models', 'yolov4-ANPR.cfg')
        video_path = os.path.join(pkg_path, 'models', 'test_video.mp4')

        self.get_logger().info('YOLO & EasyOCR 모델 초기화 중... (CPU 모드)')
        self.net = cv2.dnn.readNet(weights_path, cfg_path)
        self.classes = ['car', 'license_plate']
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        self.cap = cv2.VideoCapture(video_path)
        self.reader = easyocr.Reader(['ko', 'en'], gpu=False)

        # 🔑 등록 차량
        self.registered_plates = [
            plate.strip()
            for plate in os.getenv('REGISTERED_PLATES', '').split(',')
            if plate.strip()
        ]

        # ---- 인식 상태 ----
        self.plate_confirmed = False        
        self.detected_text = ""

        # ---- 연출 상태머신 ----
        self.state = 'IDLE'
        self.state_time = time.time()
        self.last_time = time.time()
        self.car_pos = self.CAR_START        
        self.car_target = self.CAR_START
        self.car_speed = self.SPEED_RESET
        self.barrier = self.CLOSE_ANGLE

        self._publish_car(self.CAR_START)

        self.frame_count = 0
        self.timer = self.create_timer(0.033, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        h, w = frame.shape[:2]
        new_h = 800
        new_w = int(w * (new_h / h))
        frame = cv2.resize(frame, (new_w, new_h))

        self.frame_count += 1

        if self.state == 'IDLE':
            if self.frame_count % 3 == 0:
                self._detect(frame)

        now = time.time()
        self._update_sequence(now)

        cv2.putText(frame, f"STATE: {self.state}", (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        if self.detected_text:
            cv2.putText(frame, f"PLATE: {self.detected_text}", (15, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("AI Parking System (YOLO + EasyOCR)", frame)
        cv2.waitKey(1)

    def _detect(self, frame):
        height, width, _ = frame.shape

        blob = cv2.dnn.blobFromImage(frame, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        box_coords = None
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if class_id == 1 and confidence > 0.7:
                    center_x, center_y = int(detection[0] * width), int(detection[1] * height)
                    bw, bh = int(detection[2] * width), int(detection[3] * height)
                    x, y = int(center_x - bw / 2), int(center_y - bh / 2)
                    box_coords = (x, y, bw, bh)
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
                    break

        if box_coords:
            x, y, bw, bh = box_coords
            y1, y2 = max(0, y), min(height, y + bh)
            x1, x2 = max(0, x), min(width, x + bw)
            plate_roi = frame[y1:y2, x1:x2]

            if plate_roi.size != 0:
                # ★ 복잡한 필터 삭제! 순수 회색조 변환만 남김 ★
                gray_plate = cv2.cvtColor(plate_roi, cv2.COLOR_BGR2GRAY)
                
                # 원본에 가까운 이미지를 던져줍니다
                result = self.reader.readtext(gray_plate, detail=0)
                
                if result:
                    text = "".join(result)
                    clean_text = "".join(filter(str.isalnum, text))
                    if clean_text:
                        self.get_logger().info(f'[EasyOCR 디버그] 읽은 글자: {clean_text}')
                        cv2.putText(frame, clean_text, (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        if clean_text in self.registered_plates:
                            self.plate_confirmed = True
                            self.detected_text = clean_text
                
                # ★ 창은 그대로 띄움: 이제 글자가 선명하게 보일 겁니다! ★
                cv2.imshow("Plate Preprocessing", gray_plate)

    def _update_sequence(self, now):
        dt = now - self.last_time
        self.last_time = now
        if dt > 0.2:
            dt = 0.2

        step = self.car_speed * dt
        if self.car_pos < self.car_target:
            self.car_pos = min(self.car_pos + step, self.car_target)
        else:
            self.car_pos = max(self.car_pos - step, self.car_target)

        s = self.state
        if s == 'IDLE':
            self.car_target = self.CAR_START
            self.car_speed = self.SPEED_RESET
            self.barrier = self.CLOSE_ANGLE
            
            if self.plate_confirmed and abs(self.car_pos - self.CAR_START) < 0.3:
                self.get_logger().info(f'>>> 인가차량 [{self.detected_text}] 접근 확인! <<<')
                self.state = 'APPROACH'
                self.state_time = now

        elif s == 'APPROACH':
            self.car_target = self.CAR_WAIT
            self.car_speed = self.SPEED_DRIVE
            self.barrier = self.CLOSE_ANGLE
            if abs(self.car_pos - self.CAR_WAIT) < 0.1:
                self.state = 'OPEN'
                self.state_time = now

        elif s == 'OPEN':
            self.barrier = self.OPEN_ANGLE
            if now - self.state_time > self.OPEN_WAIT:
                self.get_logger().info('>>> 차단기 개방 / 차량 진입 <<<')
                self.state = 'PASS'
                self.state_time = now

        elif s == 'PASS':
            self.car_target = self.CAR_EXIT
            self.car_speed = self.SPEED_DRIVE
            self.barrier = self.OPEN_ANGLE
            if abs(self.car_pos - self.CAR_EXIT) < 0.2:
                self.state = 'CLOSE'
                self.state_time = now

        elif s == 'CLOSE':
            self.barrier = self.CLOSE_ANGLE
            if now - self.state_time > self.CLOSE_WAIT:
                self.get_logger().info('>>> 차단기 폐쇄 <<<')
                self.plate_confirmed = False  
                self.state = 'IDLE'
                self.state_time = now
                self.detected_text = ""

        self._publish_barrier(self.barrier)
        self._publish_car(self.car_pos)

    def _publish_barrier(self, val):
        msg = Float64()
        msg.data = float(val)
        self.barrier_pub.publish(msg)

    def _publish_car(self, val):
        msg = Float64()
        msg.data = float(val)
        self.car_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloOcrBarrierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()