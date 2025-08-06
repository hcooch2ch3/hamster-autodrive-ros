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
        super().__init__("line_follower_node")

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.processed_image_publisher = self.create_publisher(
            Image, "/line_follower/processed_image", 10
        )
        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        # Parameters
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_gain", 0.005)
        self.declare_parameter("roi_height_ratio", 0.5)  # Bottom 50% of image
        self.declare_parameter("roi_y_offset", 0.5)  # Start from 50% down (맨 하단 50%)
        self.declare_parameter("blur_kernel_size", 7)  # 약간 더 크게
        self.declare_parameter("canny_low", 40)  # 더 낮은 임계값으로 약한 edge도 검출
        self.declare_parameter("canny_high", 120)  # 비율 유지하면서 낮춤
        self.declare_parameter("min_line_length", 8)  # 더욱 짧은 점선도 검출
        self.declare_parameter("max_line_gap", 15)  # 더 촘촘한 간격 연결
        self.declare_parameter("use_contour_detection", True)  # 윤곽선 검출 사용
        self.declare_parameter("white_line_detection", True)  # 흰색 선 검출 모드
        self.declare_parameter("brightness_threshold", 200)  # 흰색 선 임계값
        self.declare_parameter("adaptive_threshold", True)  # 적응적 임계값 사용
        self.declare_parameter("contrast_enhancement", True)  # 대비 향상 사용
        self.declare_parameter("color_filtering", True)  # HSV 색상 필터링 사용
        self.declare_parameter(
            "saturation_max", 30
        )  # 채도 최대값 (낮을수록 흰색에 가까움)
        self.declare_parameter("line_thinning", True)  # 굵은 선을 얇게 만들기
        self.declare_parameter("merge_close_lines", True)  # 가까운 선들 병합
        self.declare_parameter(
            "curve_detection_method", "contour"
        )  # 'hough', 'contour', 'moment', 'regression'
        self.declare_parameter("use_bgr_filtering", True)  # BGR + HSV 조합 필터링
        self.declare_parameter("separate_lanes", True)  # 좌우 차선 분리 검출
        self.declare_parameter("use_linear_regression", True)  # 선형 회귀 사용
        self.declare_parameter("regression_min_points", 10)  # 회귀 최소 점 개수
        self.declare_parameter("adaptive_lane_width", True)  # 적응적 차선 폭 추정
        self.declare_parameter("curve_prediction", True)  # 커브에서 차선 예측
        self.declare_parameter("min_lane_width", 200)  # 최소 차선 폭 (픽셀)
        self.declare_parameter("max_lane_width", 500)  # 최대 차선 폭 (픽셀)

        # PID Controller variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.1)

        # 차선 폭 히스토리 (적응적 추정용)
        self.lane_width_history = []
        self.max_history_size = 10

        self.get_logger().info("Line Follower Node started")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

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
            self.get_logger().error(f"Image processing error: {str(e)}")

    def detect_line(self, image):
        """Detect line using edge detection and Hough transform"""
        height, width = image.shape[:2]

        # Define ROI (Region of Interest) - bottom portion of image
        roi_height = int(height * self.get_parameter("roi_height_ratio").value)
        roi_y = int(height * self.get_parameter("roi_y_offset").value)

        # Create mask for ROI
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[roi_y : roi_y + roi_height, :] = 255

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 강화된 색상 필터링 (BGR + HSV 조합으로 흰색 검출)
        if self.get_parameter("color_filtering").value:
            white_color_mask = self.detect_white_lines_enhanced(image)
            # 원본 그레이스케일에 색상 마스크 적용
            gray = cv2.bitwise_and(gray, gray, mask=white_color_mask)

        # Apply ROI mask
        masked_gray = cv2.bitwise_and(gray, mask)

        # 대비 향상 (회색 도로와 흰색 차선 구분 강화)
        if self.get_parameter("contrast_enhancement").value:
            # CLAHE (Contrast Limited Adaptive Histogram Equalization) 적용
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            masked_gray = clahe.apply(masked_gray)

        # 흰색 선 검출을 위한 전처리
        if self.get_parameter("white_line_detection").value:
            if self.get_parameter("adaptive_threshold").value:
                # 적응적 임계값 처리 (조명 변화에 강함)
                white_mask = cv2.adaptiveThreshold(
                    masked_gray,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY,
                    15,
                    -5,  # 음수 C값으로 밝은 부분만 추출
                )
            else:
                # 고정 임계값 처리
                brightness_threshold = self.get_parameter("brightness_threshold").value
                _, white_mask = cv2.threshold(
                    masked_gray, brightness_threshold, 255, cv2.THRESH_BINARY
                )

            # 형태학적 연산으로 노이즈 제거
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

            processed_for_edges = white_mask
        else:
            processed_for_edges = masked_gray

        # Gaussian blur to reduce noise
        kernel_size = self.get_parameter("blur_kernel_size").value
        blurred = cv2.GaussianBlur(processed_for_edges, (kernel_size, kernel_size), 0)

        # Edge detection
        canny_low = self.get_parameter("canny_low").value
        canny_high = self.get_parameter("canny_high").value
        edges = cv2.Canny(blurred, canny_low, canny_high)

        # 굵은 선을 얇게 만들기 (부드러운 처리)
        if self.get_parameter("line_thinning").value:
            # 너무 강한 스켈레톤화 대신 부드러운 morphology 연산 사용
            kernel_thin = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel_thin)  # 연결
            edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel_thin)  # 정리

        # 점선 검출을 위한 추가 전처리 (카메라 각도 고려)
        if self.get_parameter("use_contour_detection").value:
            # ROI 영역에 따라 다른 크기의 커널 사용
            roi_center_y = roi_y + roi_height // 2

            # 멀리 있는 부분(위쪽)은 큰 커널, 가까운 부분(아래쪽)은 작은 커널
            if roi_center_y < height * 0.7:  # 상대적으로 멀리 있는 부분
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 8))
            else:  # 가까운 부분
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 4))

            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Choose detection method based on parameter
        detection_method = self.get_parameter("curve_detection_method").value

        # Create visualization image
        processed_image = image.copy()
        cv2.rectangle(
            processed_image, (0, roi_y), (width, roi_y + roi_height), (0, 255, 0), 2
        )

        # Detect line and get visualization data
        line_center = None
        valid_lines = []  # 모든 검출 방법에서 사용할 기본값

        if detection_method == "moment":
            line_center = self.detect_line_moment_based(edges, roi_y, roi_height)
            # Draw moment-based visualization - center line as vertical red line
            if line_center is not None:
                cv2.line(
                    processed_image,
                    (line_center, roi_y),
                    (line_center, roi_y + roi_height),
                    (0, 0, 255),
                    3,
                )
                # Also draw the edge pixels in red for debugging
                roi_edges = edges[roi_y : roi_y + roi_height, :]
                red_overlay = processed_image[roi_y : roi_y + roi_height, :].copy()
                red_overlay[:, :, 0] = np.maximum(
                    red_overlay[:, :, 0], roi_edges
                )  # Add red channel
                processed_image[roi_y : roi_y + roi_height, :] = red_overlay
        elif detection_method == "contour":
            line_center, largest_contour = self.detect_line_contour_based(edges)
            # Draw the detected contour in red
            if largest_contour is not None:
                cv2.drawContours(processed_image, [largest_contour], -1, (0, 0, 255), 3)
        elif detection_method == "regression":
            line_center, regression_lines = self.detect_line_regression_based(
                edges, roi_y, roi_height, width
            )
            # Draw regression lines
            if regression_lines:
                for line_points in regression_lines:
                    if len(line_points) >= 2:
                        cv2.line(
                            processed_image,
                            tuple(line_points[0]),
                            tuple(line_points[-1]),
                            (0, 0, 255),
                            3,
                        )
        else:  # hough (기존 방식)
            line_center, valid_lines = self.detect_line_hough_based(
                edges, roi_y, roi_height
            )
            # Draw the detected lines with left/right color coding
            if valid_lines and self.get_parameter("separate_lanes").value:
                left_lines, right_lines = self.separate_left_right_lines(
                    valid_lines, width
                )
                # 좌측 차선: 빨간색, 우측 차선: 파란색
                for x1, y1, x2, y2 in left_lines:
                    cv2.line(
                        processed_image, (x1, y1), (x2, y2), (0, 0, 255), 3
                    )  # 빨간색
                for x1, y1, x2, y2 in right_lines:
                    cv2.line(
                        processed_image, (x1, y1), (x2, y2), (255, 0, 0), 3
                    )  # 파란색
            elif valid_lines:
                # 기존 방식: 모든 선을 빨간색으로
                for x1, y1, x2, y2 in valid_lines:
                    cv2.line(processed_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Draw image center reference
        image_center = width // 2
        cv2.line(
            processed_image, (image_center, 0), (image_center, height), (255, 255, 0), 2
        )

        # Draw detected line center if found
        if line_center is not None:
            cv2.circle(
                processed_image,
                (line_center, roi_y + roi_height // 2),
                10,
                (255, 0, 0),
                -1,
            )
            cv2.putText(
                processed_image,
                f"Method: {detection_method}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            # 차선 예측 시각화 추가 (좌우 분리 검출 시에만)
            if (
                self.get_parameter("separate_lanes").value
                and valid_lines
                and detection_method == "hough"
            ):
                left_lines, right_lines = self.separate_left_right_lines(
                    valid_lines, width
                )
                left_center = None
                right_center = None

                if left_lines:
                    left_centers = [(x1 + x2) // 2 for x1, y1, x2, y2 in left_lines]
                    left_center = int(np.mean(left_centers))

                if right_lines:
                    right_centers = [(x1 + x2) // 2 for x1, y1, x2, y2 in right_lines]
                    right_center = int(np.mean(right_centers))

                # 예측 시각화 실행
                self.visualize_prediction(
                    processed_image, left_center, right_center, width, roi_y, roi_height
                )

        return line_center, processed_image

    def visualize_prediction(
        self, processed_image, left_center, right_center, image_width, roi_y, roi_height
    ):
        """차선 예측 과정 시각화"""
        if left_center is not None and right_center is not None:
            # 양쪽 차선 모두 보이는 경우
            self.draw_prediction_info(
                processed_image,
                "BOTH LANES",
                f"L:{left_center} R:{right_center}",
                (0, 255, 0),
            )
            # 차선 폭 히스토리 업데이트
            if self.get_parameter("adaptive_lane_width").value:
                current_width = abs(right_center - left_center)
                self.update_lane_width_history(current_width)
        elif self.get_parameter("curve_prediction").value:
            estimated_width = self.get_estimated_lane_width()

            if left_center is not None:
                # 좌측만 보이는 경우
                predicted_right = left_center + estimated_width
                self.draw_predicted_lane(
                    processed_image,
                    predicted_right,
                    roi_y,
                    roi_height,
                    (255, 0, 255),
                    "RIGHT",
                )
                self.draw_prediction_info(
                    processed_image,
                    "LEFT ONLY",
                    f"Est.W:{estimated_width} Pred.R:{predicted_right}",
                    (255, 0, 255),
                )

            elif right_center is not None:
                # 우측만 보이는 경우
                predicted_left = right_center - estimated_width
                self.draw_predicted_lane(
                    processed_image,
                    predicted_left,
                    roi_y,
                    roi_height,
                    (255, 255, 0),
                    "LEFT",
                )
                self.draw_prediction_info(
                    processed_image,
                    "RIGHT ONLY",
                    f"Est.W:{estimated_width} Pred.L:{predicted_left}",
                    (255, 255, 0),
                )

    def detect_white_lines_enhanced(self, image):
        """BGR + HSV 조합으로 강화된 흰색 선 검출 - 요철 필터링 개선"""
        # HSV 색상 공간에서 흰색 검출
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        saturation_max = self.get_parameter("saturation_max").value

        # HSV에서 적당히 관대한 흰색 범위 (차선 검출 우선)
        lower_white_hsv = np.array([0, 0, 180])  # 원래대로 복원
        upper_white_hsv = np.array([180, saturation_max, 255])  # 채도 제한 완화
        hsv_white_mask = cv2.inRange(hsv, lower_white_hsv, upper_white_hsv)

        # BGR + HSV 조합 사용 시
        if self.get_parameter("use_bgr_filtering").value:
            # BGR에서 적당한 흰색 검출 (차선 놓치지 않도록)
            lower_white_bgr = np.array([210, 210, 210])  # 임계값 하향 (230→210)
            upper_white_bgr = np.array([255, 255, 255])  # 순백색
            bgr_white_mask = cv2.inRange(image, lower_white_bgr, upper_white_bgr)

            # 좀 더 관대한 조건: OR 연산으로 결합 (합집합 - 차선 놓치지 않게)
            combined_mask = cv2.bitwise_or(hsv_white_mask, bgr_white_mask)

            # 추가 필터링: 색상 균일성 검사
            if self.get_parameter("color_filtering").value:
                combined_mask = self.filter_color_uniformity(image, combined_mask)

            return combined_mask
        else:
            return hsv_white_mask

    def filter_color_uniformity(self, image, mask):
        """색상 균일성 기반 필터링 - 요철의 불균일한 색상 제거"""
        # 마스크 영역에서 BGR 표준편차 계산
        result_mask = mask.copy()

        # 연결된 컴포넌트 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # 작은 영역은 건너뛰기
            if cv2.contourArea(contour) < 50:
                continue

            # 컨투어 마스크 생성
            temp_mask = np.zeros_like(mask)
            cv2.fillPoly(temp_mask, [contour], 255)

            # 해당 영역의 색상 분석
            region_pixels = image[temp_mask > 0]
            if len(region_pixels) > 10:
                # BGR 채널별 표준편차 계산
                b_std = np.std(region_pixels[:, 0])
                g_std = np.std(region_pixels[:, 1])
                r_std = np.std(region_pixels[:, 2])
                avg_std = (b_std + g_std + r_std) / 3

                # 표준편차가 매우 큰 영역만 제거 (임계값 완화)
                if avg_std > 35:  # 임계값 완화 (20→35)
                    cv2.fillPoly(result_mask, [contour], 0)

        return result_mask

    def separate_left_right_lines(self, lines, image_width):
        """좌/우 차선 분리 검출"""
        if not lines:
            return [], []

        left_lines = []
        right_lines = []
        image_center = image_width // 2

        for x1, y1, x2, y2 in lines:
            if x2 == x1:  # 수직선 제외
                continue

            slope = (y2 - y1) / (x2 - x1)
            center_x = (x1 + x2) // 2

            # 기울기와 위치로 좌/우 판단
            # 좌측 차선 (음의 기울기)
            if slope < -0.3 and center_x < image_center * 1.2:
                left_lines.append([x1, y1, x2, y2])
            # 우측 차선 (양의 기울기)
            elif slope > 0.3 and center_x > image_center * 0.8:
                right_lines.append([x1, y1, x2, y2])

        return left_lines, right_lines

    def update_lane_width_history(self, width):
        """차선 폭 히스토리 업데이트"""
        min_width = self.get_parameter("min_lane_width").value
        max_width = self.get_parameter("max_lane_width").value

        # 유효한 범위 내의 폭만 저장
        if min_width <= width <= max_width:
            self.lane_width_history.append(width)
            if len(self.lane_width_history) > self.max_history_size:
                self.lane_width_history.pop(0)

    def get_estimated_lane_width(self):
        """히스토리 기반 추정 차선 폭"""
        if not self.lane_width_history:
            return 300  # 기본값
        return int(np.median(self.lane_width_history))

    def calculate_center_with_prediction(
        self, left_center, right_center, image_width, processed_image, roi_y, roi_height
    ):
        """곡선 예측을 포함한 중심점 계산 + 시각화"""
        if left_center is not None and right_center is not None:
            # 양쪽 차선 모두 보이는 경우
            self.draw_prediction_info(
                processed_image,
                "BOTH LANES",
                f"L:{left_center} R:{right_center}",
                (0, 255, 0),
            )
            return (left_center + right_center) // 2

        # 커브 예측 기능이 활성화된 경우만
        if self.get_parameter("curve_prediction").value:
            estimated_width = self.get_estimated_lane_width()

            if left_center is not None:
                # 좌측만 보일 때: 히스토리 기반으로 우측 예측
                predicted_right = left_center + estimated_width

                # 예측된 우측 차선 시각화 (점선으로)
                self.draw_predicted_lane(
                    processed_image,
                    predicted_right,
                    roi_y,
                    roi_height,
                    (255, 0, 255),
                    "RIGHT",
                )

                # 이미지 경계 체크
                if predicted_right < image_width:
                    center = (left_center + predicted_right) // 2
                    self.draw_prediction_info(
                        processed_image,
                        "LEFT ONLY",
                        f"Est.W:{estimated_width} Pred.R:{predicted_right}",
                        (255, 0, 255),
                    )
                    return center
                else:
                    # 우측이 이미지를 벗어나면 좌측 기준으로 조정
                    center = min(
                        left_center + estimated_width // 2,
                        image_width - estimated_width // 4,
                    )
                    self.draw_prediction_info(
                        processed_image,
                        "LEFT ONLY (BOUNDARY)",
                        "Adjusted center",
                        (255, 100, 0),
                    )
                    return center

            elif right_center is not None:
                # 우측만 보일 때: 히스토리 기반으로 좌측 예측
                predicted_left = right_center - estimated_width

                # 예측된 좌측 차선 시각화 (점선으로)
                self.draw_predicted_lane(
                    processed_image,
                    predicted_left,
                    roi_y,
                    roi_height,
                    (255, 255, 0),
                    "LEFT",
                )

                # 이미지 경계 체크
                if predicted_left >= 0:
                    center = (predicted_left + right_center) // 2
                    self.draw_prediction_info(
                        processed_image,
                        "RIGHT ONLY",
                        f"Est.W:{estimated_width} Pred.L:{predicted_left}",
                        (255, 255, 0),
                    )
                    return center
                else:
                    # 좌측이 이미지를 벗어나면 우측 기준으로 조정
                    center = max(
                        right_center - estimated_width // 2, estimated_width // 4
                    )
                    self.draw_prediction_info(
                        processed_image,
                        "RIGHT ONLY (BOUNDARY)",
                        "Adjusted center",
                        (255, 100, 0),
                    )
                    return center
        else:
            # 예측 기능 비활성화 - 기존 고정 방식
            if left_center is not None:
                self.draw_prediction_info(
                    processed_image,
                    "LEFT ONLY (FIXED)",
                    "Fixed offset +150",
                    (128, 128, 128),
                )
                return left_center + 150
            elif right_center is not None:
                self.draw_prediction_info(
                    processed_image,
                    "RIGHT ONLY (FIXED)",
                    "Fixed offset -150",
                    (128, 128, 128),
                )
                return right_center - 150

        # 양쪽 모두 없는 경우
        self.draw_prediction_info(
            processed_image, "NO LANES", "No detection", (0, 0, 255)
        )
        return None

    def draw_predicted_lane(self, image, x_pos, roi_y, roi_height, color, side):
        """예측된 차선을 점선으로 그리기"""
        if 0 <= x_pos < image.shape[1]:
            # 점선으로 예측 차선 표시
            for y in range(roi_y, roi_y + roi_height, 10):
                if y + 5 < roi_y + roi_height:
                    cv2.line(image, (int(x_pos), y), (int(x_pos), y + 5), color, 2)

            # 예측 표시 텍스트
            cv2.putText(
                image,
                f"PRED {side}",
                (int(x_pos) - 30, roi_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

    def draw_prediction_info(self, image, status, details, color):
        """예측 정보를 화면에 표시"""
        # 상태 표시
        cv2.putText(image, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 세부 정보 표시
        cv2.putText(image, details, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # 차선 폭 히스토리 표시
        if self.lane_width_history:
            avg_width = int(np.mean(self.lane_width_history))
            cv2.putText(
                image,
                f"Avg Width: {avg_width} (n={len(self.lane_width_history)})",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

    def calculate_center_with_prediction_simple(
        self, left_center, right_center, image_width
    ):
        """시각화 없는 간단한 중심점 계산"""
        if left_center is not None and right_center is not None:
            return (left_center + right_center) // 2

        # 커브 예측 기능이 활성화된 경우만
        if self.get_parameter("curve_prediction").value:
            estimated_width = self.get_estimated_lane_width()

            if left_center is not None:
                predicted_right = left_center + estimated_width
                if predicted_right < image_width:
                    return (left_center + predicted_right) // 2
                else:
                    return min(
                        left_center + estimated_width // 2,
                        image_width - estimated_width // 4,
                    )

            elif right_center is not None:
                predicted_left = right_center - estimated_width
                if predicted_left >= 0:
                    return (predicted_left + right_center) // 2
                else:
                    return max(
                        right_center - estimated_width // 2, estimated_width // 4
                    )
        else:
            # 기존 고정 방식
            if left_center is not None:
                return left_center + 150
            elif right_center is not None:
                return right_center - 150

        return None

    def calculate_lane_center_improved(self, lines, image_width):
        """좌우 분리된 차선으로 더 정확한 중심점 계산 + 적응적 차선 폭"""
        if not self.get_parameter("separate_lanes").value:
            # 기존 방식 (모든 선의 평균)
            if not lines:
                return None
            line_centers = [(x1 + x2) // 2 for x1, y1, x2, y2 in lines]
            return int(np.mean(line_centers))

        # 좌우 분리 방식
        left_lines, right_lines = self.separate_left_right_lines(lines, image_width)

        left_center = None
        right_center = None

        # 좌측 차선 중심
        if left_lines:
            left_centers = [(x1 + x2) // 2 for x1, y1, x2, y2 in left_lines]
            left_center = int(np.mean(left_centers))

        # 우측 차선 중심
        if right_lines:
            right_centers = [(x1 + x2) // 2 for x1, y1, x2, y2 in right_lines]
            right_center = int(np.mean(right_centers))

        # 적응적 차선 폭 추정
        if (
            left_center is not None
            and right_center is not None
            and self.get_parameter("adaptive_lane_width").value
        ):
            current_width = abs(right_center - left_center)
            self.update_lane_width_history(current_width)

        # 차선 중앙점 계산 (시각화는 detect_line에서 처리)
        return self.calculate_center_with_prediction_simple(
            left_center, right_center, image_width
        )

    def detect_line_regression_based(self, edges, roi_y, roi_height, image_width):
        """선형 회귀 기반 라인 검출 - 노이즈에 강한 직선/곡선 검출"""
        # ROI 영역에서만 검출
        roi_edges = edges[roi_y : roi_y + roi_height, :]

        # 엣지 포인트 추출
        edge_points = np.column_stack(np.where(roi_edges > 0))

        if len(edge_points) < self.get_parameter("regression_min_points").value:
            return None, []

        # y와 x 좌표 변환 (OpenCV 좌표계)
        y_coords = edge_points[:, 0] + roi_y  # ROI offset 추가
        x_coords = edge_points[:, 1]

        # 좌우 차선 분리 검출 적용
        if self.get_parameter("separate_lanes").value:
            return self.detect_lanes_with_regression(
                x_coords, y_coords, image_width, roi_y, roi_height
            )
        else:
            # 단일 라인 회귀
            try:
                # 1차 다항식 피팅 (직선)
                coeffs = np.polyfit(y_coords, x_coords, 1)
                poly_func = np.poly1d(coeffs)

                # 중심점 계산 (ROI 중앙에서의 x 값)
                center_y = roi_y + roi_height // 2
                line_center = int(poly_func(center_y))

                # 시각화용 라인 포인트
                y_line = np.linspace(roi_y, roi_y + roi_height, 50)
                x_line = poly_func(y_line)
                line_points = [
                    [int(x), int(y)]
                    for x, y in zip(x_line, y_line)
                    if 0 <= x < image_width
                ]

                return line_center, [line_points] if line_points else []

            except (np.linalg.LinAlgError, np.RankWarning):
                return None, []

    def detect_lanes_with_regression(
        self, x_coords, y_coords, image_width, roi_y, roi_height
    ):
        """좌우 차선을 분리하여 각각 회귀 분석"""
        image_center = image_width // 2

        # 좌우 분리
        left_mask = x_coords < image_center * 1.2
        right_mask = x_coords > image_center * 0.8

        left_x = x_coords[left_mask]
        left_y = y_coords[left_mask]
        right_x = x_coords[right_mask]
        right_y = y_coords[right_mask]

        min_points = self.get_parameter("regression_min_points").value // 2

        left_line = None
        right_line = None
        regression_lines = []

        # 좌측 차선 회귀
        if len(left_x) >= min_points:
            try:
                left_coeffs = np.polyfit(left_y, left_x, 1)
                left_func = np.poly1d(left_coeffs)

                # 시각화용 포인트
                y_line = np.linspace(roi_y, roi_y + roi_height, 30)
                x_line = left_func(y_line)
                left_points = [
                    [int(x), int(y)]
                    for x, y in zip(x_line, y_line)
                    if 0 <= x < image_width
                ]
                if left_points:
                    regression_lines.append(left_points)
                    left_line = int(left_func(roi_y + roi_height // 2))
            except (np.linalg.LinAlgError, np.RankWarning):
                pass
        # 우측 차선 회귀
        if len(right_x) >= min_points:
            try:
                right_coeffs = np.polyfit(right_y, right_x, 1)
                right_func = np.poly1d(right_coeffs)

                # 시각화용 포인트
                y_line = np.linspace(roi_y, roi_y + roi_height, 30)
                x_line = right_func(y_line)
                right_points = [
                    [int(x), int(y)]
                    for x, y in zip(x_line, y_line)
                    if 0 <= x < image_width
                ]
                if right_points:
                    regression_lines.append(right_points)
                    right_line = int(right_func(roi_y + roi_height // 2))
            except (np.linalg.LinAlgError, np.RankWarning):
                pass

        # 중심점 계산
        if left_line is not None and right_line is not None:
            line_center = (left_line + right_line) // 2
        elif left_line is not None:
            line_center = left_line + 150  # 좌측만 있으면 우측 추정
        elif right_line is not None:
            line_center = right_line - 150  # 우측만 있으면 좌측 추정
        else:
            return None, []

        return line_center, regression_lines

    def detect_line_moment_based(self, edges, roi_y, roi_height):
        """모멘트 기반 중심선 검출 - 곡선에 최고 성능"""
        # ROI 영역에서만 모멘트 계산
        roi_edges = edges[roi_y : roi_y + roi_height, :]

        # 각 행(y좌표)별로 중심점 계산
        centers = []
        weights = []  # 가중치 (더 강한 신호일수록 높은 가중치)

        for y_idx, y in enumerate(range(roi_edges.shape[0])):
            row = roi_edges[y, :]
            white_pixels = np.where(row > 0)[0]

            if len(white_pixels) > 0:
                # 가중 평균으로 중심점 계산
                weights_row = row[white_pixels].astype(float)
                if np.sum(weights_row) > 0:
                    center_x = np.average(white_pixels, weights=weights_row)
                    centers.append(center_x)
                    # 가까운 행일수록 더 중요하게 (로봇에 가까운 부분)
                    weight = np.sum(weights_row) * (y_idx + 1)
                    weights.append(weight)

        if centers and weights:
            # 가중 평균으로 최종 중심점 계산
            final_center = np.average(centers, weights=weights)
            return int(final_center)
        return None

    def detect_line_contour_based(self, edges):
        """컨투어 기반 중심선 검출 - 곡선에 적합"""
        # Find contours
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, None

        # Get largest contour (main line)
        largest_contour = max(contours, key=cv2.contourArea)

        # 최소 면적 체크
        if cv2.contourArea(largest_contour) < 50:
            return None, None

        # Calculate moments to find centroid
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            return cx, largest_contour
        return None, None

    def detect_line_hough_based(self, edges, roi_y, roi_height):
        """기존 Hough Transform 방식 - 직선에만 적합"""
        min_line_length = self.get_parameter("min_line_length").value
        max_line_gap = self.get_parameter("max_line_gap").value

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 360,
            threshold=12,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap,
        )

        if lines is None:
            return None, []

        # Filter and average lines
        valid_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            if abs(angle) > 15 and abs(angle) < 165:
                valid_lines.append(line[0])

        if valid_lines:
            # 개선된 중심점 계산 (좌우 분리 고려)
            line_center = self.calculate_lane_center_improved(
                valid_lines, 640
            )  # 기본 이미지 폭
            return line_center, valid_lines
        return None, []

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
                    (center1[0] - center2[0]) ** 2 + (center1[1] - center2[1]) ** 2
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
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        # Calculate PID terms
        proportional = error
        self.integral_error += error
        derivative = error - self.previous_error

        # PID output
        pid_output = kp * proportional + ki * self.integral_error + kd * derivative

        # Create control command
        cmd = Twist()
        cmd.linear.x = self.get_parameter("linear_speed").value
        cmd.angular.z = -pid_output * self.get_parameter("angular_gain").value

        # Limit angular velocity
        max_angular = 1.0
        cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))

        # Debug: 명령어 로깅 강화
        self.get_logger().info(
            f"🚗 MOVE: linear.x={cmd.linear.x:.3f}, "
            f"angular.z={cmd.angular.z:.3f}, error={error}"
        )

        # Publish command
        self.cmd_vel_publisher.publish(cmd)

        # Update previous error
        self.previous_error = error

        self.get_logger().info(
            f"Line following: error={error}, angular_z={cmd.angular.z:.3f}"
        )

    def stop_robot(self):
        """Stop the robot when no line is detected"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().warn("🛑 STOP: No line detected - stopping robot")

    def publish_processed_image(self, cv_image):
        """Publish processed image for visualization"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_link"
            self.processed_image_publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Failed to publish processed image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LineFollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
