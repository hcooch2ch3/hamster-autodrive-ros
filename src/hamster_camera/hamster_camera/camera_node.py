#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import urllib.request
import threading


class HamsterCameraNode(Node):
    def __init__(self):
        super().__init__("hamster_camera_node")

        # CV Bridge 초기화 (ROS Image 메시지와 OpenCV 이미지 변환용)
        self.bridge = CvBridge()

        # 카메라 스트림 URL 및 GStreamer 파이프라인
        self.declare_parameter(
            "camera_base_url", "http://192.168.66.1:9527/videostream.cgi"
        )
        self.declare_parameter("camera_username", "admin")
        self.declare_parameter("camera_password", "admin")

        camera_base_url = (
            self.get_parameter("camera_base_url").get_parameter_value().string_value
        )
        camera_username = (
            self.get_parameter("camera_username").get_parameter_value().string_value
        )
        camera_password = (
            self.get_parameter("camera_password").get_parameter_value().string_value
        )

        self.camera_url = (
            f"{camera_base_url}?loginuse={camera_username}&loginpas={camera_password}"
        )
        self.gstreamer_pipeline = (
            f"souphttpsrc location={self.camera_url} ! multipartdemux ! "
            "jpegdec ! videoconvert ! appsink"
        )

        # OpenCV VideoCapture 초기화 (GStreamer 파이프라인 우선 시도)
        self.cap = self._initialize_camera()
        self.current_frame = None
        self.frame_lock = threading.Lock()

        # 카메라 연결 확인
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera stream: {self.camera_url}")
            return

        # 카메라 이미지 퍼블리셔
        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.processed_image_publisher = self.create_publisher(
            Image, "/camera/image_processed", 10
        )

        # 객체 검출 결과 기반 제어 명령 퍼블리셔 (선택사항)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/camera/cmd_vel", 10)

        # 카메라 프레임 읽기 스레드 시작
        self.camera_thread = threading.Thread(target=self.camera_read_thread)
        self.camera_thread.daemon = True
        self.camera_thread.start()

        # 타이머로 주기적으로 카메라 데이터 처리
        self.timer = self.create_timer(0.1, self.camera_callback)  # 10Hz

        # 파라미터 선언
        self.declare_parameter("enable_object_following", False)
        self.declare_parameter("detection_threshold", 500)
        self.declare_parameter("canny_low", 50)
        self.declare_parameter("canny_high", 150)

        self.get_logger().info(
            f"Hamster Camera Node started with URL: {self.camera_url}"
        )

    def _initialize_camera(self):
        """카메라 초기화 - GStreamer 파이프라인 우선, 실패시 직접 URL 사용"""
        try:
            # GStreamer 파이프라인으로 시도
            cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.get_logger().info("Camera initialized with GStreamer pipeline")
                return cap
            else:
                self.get_logger().warn("GStreamer pipeline failed, trying direct URL")
                cap.release()
        except Exception as e:
            self.get_logger().warn(f"GStreamer initialization failed: {str(e)}")

        try:
            # 직접 URL로 시도
            cap = cv2.VideoCapture(self.camera_url)
            if cap.isOpened():
                self.get_logger().info("Camera initialized with direct URL")
                return cap
            else:
                self.get_logger().error("Both GStreamer and direct URL failed")
                cap.release()
        except Exception as e:
            self.get_logger().error(f"Direct URL initialization failed: {str(e)}")

        return None

    def camera_read_thread(self):
        """별도 스레드에서 카메라 프레임을 지속적으로 읽기"""
        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.current_frame = frame.copy()
                else:
                    self.get_logger().warn("Failed to read frame from camera")
                    # 연결이 끊어진 경우 재연결 시도
                    self.cap.release()
                    self.cap = self._initialize_camera()
                    if self.cap is None or not self.cap.isOpened():
                        self.get_logger().error("Failed to reconnect to camera")
                        break
            except Exception as e:
                self.get_logger().error(f"Camera read thread error: {str(e)}")
                break

    def camera_callback(self):
        """카메라 데이터를 주기적으로 읽고 처리"""
        try:
            # 현재 프레임 가져오기
            with self.frame_lock:
                if self.current_frame is None:
                    return
                cv_image = self.current_frame.copy()

            # 원본 이미지 퍼블리시
            self.publish_image(cv_image, self.image_publisher)

            # 이미지 처리 수행
            processed_image, detection_results = self.process_image(cv_image)

            # 처리된 이미지 퍼블리시
            self.publish_image(processed_image, self.processed_image_publisher)

            # 객체 추적 기능 (선택사항)
            if self.get_parameter("enable_object_following").value:
                self.object_following_control(detection_results, cv_image.shape)

        except Exception as e:
            self.get_logger().warn(f"Camera processing error: {str(e)}")

    def __del__(self):
        """소멸자: 카메라 리소스 해제"""
        if hasattr(self, "cap"):
            self.cap.release()

    def process_image(self, cv_image):
        """이미지 처리 함수 - 컴퓨터 비전 알고리즘 적용"""
        try:
            # 이미지 복사
            processed = cv_image.copy()

            # 파라미터 가져오기
            canny_low = self.get_parameter("canny_low").value
            canny_high = self.get_parameter("canny_high").value

            # 예시 처리: 에지 검출
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, canny_low, canny_high)

            # 윤곽선 찾기
            contours, _ = cv2.findContours(
                edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # 윤곽선 그리기
            cv2.drawContours(processed, contours, -1, (0, 255, 0), 2)

            # 객체 검출 및 결과 반환
            detection_results = self.detect_objects(processed, contours)

            return processed, detection_results

        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
            return cv_image, []

    def detect_objects(self, image, contours):
        """간단한 객체 검출 및 정보 표시"""
        object_count = 0
        detection_results = []
        threshold = self.get_parameter("detection_threshold").value

        for contour in contours:
            area = cv2.contourArea(contour)

            # 일정 크기 이상의 윤곽선만 객체로 인식
            if area > threshold:
                object_count += 1

                # 바운딩 박스 계산
                x, y, w, h = cv2.boundingRect(contour)

                # 객체 중심점 계산
                center_x = x + w // 2
                center_y = y + h // 2

                # 검출 결과 저장
                detection_results.append(
                    {
                        "id": object_count,
                        "area": area,
                        "bbox": (x, y, w, h),
                        "center": (center_x, center_y),
                    }
                )

                # 바운딩 박스 그리기
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # 객체 정보 표시
                cv2.putText(
                    image,
                    f"Obj{object_count}({int(area)})",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    1,
                )

                # 중심점 표시
                cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)

        # 전체 객체 수 표시
        cv2.putText(
            image,
            f"Objects: {object_count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        if object_count > 0:
            self.get_logger().info(f"Detected {object_count} objects")

        return detection_results

    def object_following_control(self, detection_results, image_shape):
        """객체 추적 제어 (가장 큰 객체를 추적)"""
        if not detection_results:
            return

        # 가장 큰 객체 찾기
        largest_object = max(detection_results, key=lambda x: x["area"])

        # 이미지 중심과 객체 중심 간의 차이 계산
        image_center_x = image_shape[1] // 2
        object_center_x = largest_object["center"][0]

        # 제어 명령 생성
        cmd = Twist()

        # 간단한 P 제어기
        error_x = object_center_x - image_center_x
        angular_gain = 0.001  # 조정 가능한 게인

        # 객체가 화면 중앙에 오도록 회전 제어
        cmd.angular.z = -angular_gain * error_x

        # 객체 크기에 따른 전진/후진 제어
        if largest_object["area"] < 2000:  # 객체가 작으면 전진
            cmd.linear.x = 0.1
        elif largest_object["area"] > 5000:  # 객체가 크면 후진
            cmd.linear.x = -0.1

        # 명령 퍼블리시
        self.cmd_vel_publisher.publish(cmd)

        self.get_logger().info(
            f'Following object: error_x={error_x}, area={largest_object["area"]}'
        )

    def publish_image(self, cv_image, publisher):
        """OpenCV 이미지를 ROS Image 메시지로 변환하여 퍼블리시"""
        try:
            # OpenCV 이미지를 ROS Image 메시지로 변환
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

            # 헤더 정보 설정
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_link"

            # 퍼블리시
            publisher.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HamsterCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
