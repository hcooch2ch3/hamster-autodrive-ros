#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.processed_image_publisher = self.create_publisher(
            Image, '/line_follower/processed_image', 10)
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_gain', 0.005)
        self.declare_parameter('roi_height_ratio', 0.3)  # Bottom 30% of image
        self.declare_parameter('roi_y_offset', 0.7)  # Start from 70% down (맨 하단 30%)
        self.declare_parameter('blur_kernel_size', 7)  # 약간 더 크게
        self.declare_parameter('canny_low', 40)   # 더 낮은 임계값으로 약한 edge도 검출
        self.declare_parameter('canny_high', 120)  # 비율 유지하면서 낮춤
        self.declare_parameter('min_line_length', 8)   # 더욱 짧은 점선도 검출
        self.declare_parameter('max_line_gap', 15)     # 더 촘촘한 간격 연결
        self.declare_parameter('use_contour_detection', True)  # 윤곽선 검출 사용
        self.declare_parameter('white_line_detection', True)   # 흰색 선 검출 모드
        self.declare_parameter('brightness_threshold', 200)    # 흰색 선 임계값
        self.declare_parameter('adaptive_threshold', True)     # 적응적 임계값 사용
        self.declare_parameter('contrast_enhancement', True)   # 대비 향상 사용
        self.declare_parameter('color_filtering', True)        # HSV 색상 필터링 사용
        self.declare_parameter('saturation_max', 30)           # 채도 최대값 (낮을수록 흰색에 가까움)
        self.declare_parameter('line_thinning', True)          # 굵은 선을 얇게 만들기
        self.declare_parameter('merge_close_lines', True)      # 가까운 선들 병합

        # PID Controller variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)

        self.get_logger().info('Line Follower Node started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Process image to detect line
            line_center, processed_image = self.detect_line(cv_image)

            # Publish processed image for visualization
            self.publish_processed_image(processed_image)

            # Generate control command
            if line_center is not None:
                self.follow_line(line_center, cv_image.shape[1])
            else:
                self.stop_robot()

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def detect_line(self, image):
        """Detect line using edge detection and Hough transform"""
        height, width = image.shape[:2]

        # Define ROI (Region of Interest) - bottom portion of image
        roi_height = int(height * self.get_parameter('roi_height_ratio').value)
        roi_y = int(height * self.get_parameter('roi_y_offset').value)

        # Create mask for ROI
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[roi_y:roi_y + roi_height, :] = 255

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # HSV 색상 필터링으로 흰색만 추출 (다른 색상 제거)
        if self.get_parameter('color_filtering').value:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # 흰색 범위 정의 (HSV에서)
            saturation_max = self.get_parameter('saturation_max').value
            lower_white = np.array([0, 0, 180])        # H, S, V 최소값
            upper_white = np.array([180, saturation_max, 255])  # H, S, V 최대값

            # 흰색 마스크 생성
            white_color_mask = cv2.inRange(hsv, lower_white, upper_white)

            # 원본 그레이스케일에 색상 마스크 적용
            gray = cv2.bitwise_and(gray, gray, mask=white_color_mask)

        # Apply ROI mask
        masked_gray = cv2.bitwise_and(gray, mask)

        # 대비 향상 (회색 도로와 흰색 차선 구분 강화)
        if self.get_parameter('contrast_enhancement').value:
            # CLAHE (Contrast Limited Adaptive Histogram Equalization) 적용
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            masked_gray = clahe.apply(masked_gray)

        # 흰색 선 검출을 위한 전처리
        if self.get_parameter('white_line_detection').value:
            if self.get_parameter('adaptive_threshold').value:
                # 적응적 임계값 처리 (조명 변화에 강함)
                white_mask = cv2.adaptiveThreshold(
                    masked_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY, 15, -5  # 음수 C값으로 밝은 부분만 추출
                )
            else:
                # 고정 임계값 처리
                brightness_threshold = self.get_parameter('brightness_threshold').value
                _, white_mask = cv2.threshold(
                    masked_gray, brightness_threshold, 255, cv2.THRESH_BINARY)

            # 형태학적 연산으로 노이즈 제거
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

            processed_for_edges = white_mask
        else:
            processed_for_edges = masked_gray

        # Gaussian blur to reduce noise
        kernel_size = self.get_parameter('blur_kernel_size').value
        blurred = cv2.GaussianBlur(processed_for_edges, (kernel_size, kernel_size), 0)

        # Edge detection
        canny_low = self.get_parameter('canny_low').value
        canny_high = self.get_parameter('canny_high').value
        edges = cv2.Canny(blurred, canny_low, canny_high)

        # 굵은 선을 얇게 만들기 (부드러운 처리)
        if self.get_parameter('line_thinning').value:
            # 너무 강한 스켈레톤화 대신 부드러운 morphology 연산 사용
            kernel_thin = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel_thin)  # 연결
            edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel_thin)   # 정리

        # 점선 검출을 위한 추가 전처리 (카메라 각도 고려)
        if self.get_parameter('use_contour_detection').value:
            # ROI 영역에 따라 다른 크기의 커널 사용
            roi_center_y = roi_y + roi_height // 2

            # 멀리 있는 부분(위쪽)은 큰 커널, 가까운 부분(아래쪽)은 작은 커널
            if roi_center_y < height * 0.7:  # 상대적으로 멀리 있는 부분
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 8))
            else:  # 가까운 부분
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 4))

            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Hough Line Transform
        min_line_length = self.get_parameter('min_line_length').value
        max_line_gap = self.get_parameter('max_line_gap').value

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 360,  # 각도 해상도 2배 증가
            threshold=12,     # 더 민감하게
            minLineLength=min_line_length,
            maxLineGap=max_line_gap
        )

        # Create visualization image
        processed_image = image.copy()
        cv2.rectangle(processed_image, (0, roi_y),
                      (width, roi_y + roi_height), (0, 255, 0), 2)

        line_center = None

        if lines is not None:
            # Filter and average lines
            valid_lines = []

            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Calculate line angle
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

                # Filter lines by angle (모든 방향의 선을 허용, 각도 제한 완화)
                # 너무 수평인 선만 제외 (노이즈일 가능성 높음)
                if abs(angle) > 15 and abs(angle) < 165:  # 대부분의 선 허용
                    valid_lines.append(line[0])

            # 가까운 선들 병합 (굵은 선이 두 개로 인식되는 것 방지)
            if self.get_parameter('merge_close_lines').value and valid_lines:
                merged_lines = self.merge_nearby_lines(valid_lines)
                valid_lines = merged_lines

            # 최종 선들 시각화
            for x1, y1, x2, y2 in valid_lines:
                cv2.line(processed_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # 디버깅용 각도 표시
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                cv2.putText(processed_image, f'{angle:.1f}°',
                            ((x1 + x2) // 2, (y1 + y2) // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            if valid_lines:
                # Calculate average line position
                line_centers = []
                for x1, y1, x2, y2 in valid_lines:
                    center_x = (x1 + x2) // 2
                    line_centers.append(center_x)

                line_center = int(np.mean(line_centers))

                # Draw line center
                cv2.circle(processed_image,
                           (line_center, roi_y + roi_height // 2),
                           10, (255, 0, 0), -1)

        # Draw image center reference
        image_center = width // 2
        cv2.line(processed_image, (image_center, 0),
                 (image_center, height), (255, 255, 0), 2)

        return line_center, processed_image

    def merge_nearby_lines(self, lines, distance_threshold=20):
        """가까운 선들을 병합하여 굵은 선이 두 개로 인식되는 것 방지"""
        if len(lines) <= 1:
            return lines

        merged_lines = []
        used = [False] * len(lines)

        for i, line1 in enumerate(lines):
            if used[i]:
                continue

            x1, y1, x2, y2 = line1
            center1 = ((x1 + x2) // 2, (y1 + y2) // 2)
            similar_lines = [line1]
            used[i] = True

            # 현재 선과 가까운 선들 찾기
            for j, line2 in enumerate(lines):
                if used[j] or i == j:
                    continue

                x3, y3, x4, y4 = line2
                center2 = ((x3 + x4) // 2, (y3 + y4) // 2)

                # 중심점 간 거리 계산
                distance = np.sqrt(
                    (center1[0] - center2[0]) ** 2
                    + (center1[1] - center2[1]) ** 2
                )

                # 각도 차이 계산
                angle1 = np.arctan2(y2 - y1, x2 - x1)
                angle2 = np.arctan2(y4 - y3, x4 - x3)
                angle_diff = abs(angle1 - angle2) * 180 / np.pi

                # 가까우면서 비슷한 각도의 선들 병합
                if distance < distance_threshold and angle_diff < 30:
                    similar_lines.append(line2)
                    used[j] = True

            # 병합된 선들의 평균으로 새로운 선 생성
            if len(similar_lines) > 1:
                avg_x1 = int(np.mean([line[0] for line in similar_lines]))
                avg_y1 = int(np.mean([line[1] for line in similar_lines]))
                avg_x2 = int(np.mean([line[2] for line in similar_lines]))
                avg_y2 = int(np.mean([line[3] for line in similar_lines]))
                merged_lines.append([avg_x1, avg_y1, avg_x2, avg_y2])
            else:
                merged_lines.append(line1)

        return merged_lines

    def follow_line(self, line_center, image_width):
        """Generate control commands to follow the detected line"""
        image_center = image_width // 2
        error = line_center - image_center

        # PID Controller
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        # Calculate PID terms
        proportional = error
        self.integral_error += error
        derivative = error - self.previous_error

        # PID output
        pid_output = kp * proportional + ki * self.integral_error + kd * derivative

        # Create control command
        cmd = Twist()
        cmd.linear.x = self.get_parameter('linear_speed').value
        cmd.angular.z = -pid_output * self.get_parameter('angular_gain').value

        # Limit angular velocity
        max_angular = 1.0
        cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))

        # Publish command
        self.cmd_vel_publisher.publish(cmd)

        # Update previous error
        self.previous_error = error

        self.get_logger().info(
            f'Line following: error={error}, angular_z={cmd.angular.z:.3f}'
        )

    def stop_robot(self):
        """Stop the robot when no line is detected"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().warn('No line detected - stopping robot')

    def publish_processed_image(self, cv_image):
        """Publish processed image for visualization"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_link'
            self.processed_image_publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Failed to publish processed image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LineFollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
