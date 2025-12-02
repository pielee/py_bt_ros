import cv2
import json
import threading
import time
from cv_bridge import CvBridge

class AsyncQRProcessor:
    def __init__(self, config):
        self.config = config
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        
        # 설정 로드
        self.target_width = config['qr_system'].get('process_width', 640)
        self.areas = config['qr_system']['areas']
        self.locations = config['qr_system']['locations']

        # 쓰레드 관련 변수
        self._current_image = None
        self._lock = threading.Lock()
        self._running = True
        self._latest_result = None  # (x, y, yaw)

        # 백그라운드 워커 시작
        self._worker_thread = threading.Thread(target=self._process_loop, daemon=True)
        self._worker_thread.start()

    def update_image(self, ros_image_msg):
        """ROS 이미지 메시지를 받아서 최신 프레임으로 업데이트"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgr8')
            # LIMO Pro 성능을 위해 리사이즈 (인식률 유지하며 속도 향상)
            if cv_image.shape[1] > self.target_width:
                scale = self.target_width / cv_image.shape[1]
                cv_image = cv2.resize(cv_image, None, fx=scale, fy=scale)
            
            with self._lock:
                self._current_image = cv_image
        except Exception as e:
            print(f"[QRProcessor] Error converting image: {e}")

    def get_result(self):
        """현재까지 인식된 결과 반환 (Non-blocking)"""
        with self._lock:
            return self._latest_result

    def _process_loop(self):
        """별도 쓰레드에서 계속 돌아가는 QR 인식 루프"""
        while self._running:
            img_to_process = None
            
            # 1. 처리할 이미지 가져오기 (Lock 최소화)
            with self._lock:
                if self._current_image is not None:
                    img_to_process = self._current_image.copy()
                    self._current_image = None # 처리 했으면 비움
            
            # 2. 이미지가 없으면 잠시 대기
            if img_to_process is None:
                time.sleep(0.05)
                continue

            # 3. QR 인식 (무거운 작업)
            try:
                data, points, _ = self.qr_detector.detectAndDecode(img_to_process)
                if data:
                    parsed_pose = self._parse_data(data)
                    if parsed_pose:
                        with self._lock:
                            self._latest_result = parsed_pose
            except Exception as e:
                print(f"[QRProcessor] Detection Error: {e}")

    def _parse_data(self, data):
        """문자열 -> 좌표 변환 로직"""
        try:
            data_list = json.loads(data)
            if len(data_list) <= 1: return None
            area_name = str(data_list[1])

            # Config에 정의된 지역 매칭
            for region_key, keywords in self.areas.items():
                if area_name in keywords:
                    loc = self.locations[region_key]
                    return (loc['x'], loc['y'], loc['yaw'])
        except:
            pass
        return None

    def stop(self):
        self._running = False
        self._worker_thread.join()