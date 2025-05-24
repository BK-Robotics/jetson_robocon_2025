#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ultralytics import YOLO

# Cấu hình cho việc phát hiện và xử lý bảng rổ
CONFIG = {
    # Model
    'model_path': 'v2.pt',
    'confidence': 0.25,
    
    # Camera
    'frame_size': (640, 480),  # width, height
    'fov_horizontal': 60,  # độ
    
    # Hiển thị
    'display_size': (800, 1200),
    'debug_size': (300, 200),
    
    # HSV thresholds
    'hsv_white': ([0, 0, 160], [180, 45, 255]),  # (lower, upper)
    'hsv_black': ([0, 0, 0], [180, 255, 50]),
    
    # Contour filtering
    'min_area': 200,
    'aspect_ratio': (1.2, 2.0),
    'max_area_ratio': 0.9,
    
    # Morphology
    'kernel_sizes': {
        'small': (3, 3),
        'medium': (5, 5)
    },
    'morph_iters': {
        'open': 1,
        'close': 2
    },
    
    # Colors (BGR)
    'colors': {
        'centroid': (0, 0, 255),      # Đỏ
        'centroid_outline': (255, 255, 255),  # Trắng
        'contour': (0, 255, 0),       # Xanh lá
        'backboard': (0, 255, 255),   # Vàng
        'hoop': (0, 255, 255)         # Vàng
    },
    
    # Text
    'font': cv2.FONT_HERSHEY_SIMPLEX,
    'font_scale': 0.6,
    'font_thickness': 2
}

def process_roi(roi):
    """Xử lý vùng ảnh ROI để tìm contour và centroid"""
    if roi.size == 0:
        return None, None, None
        
    # Chuyển sang ảnh xám
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # Làm mờ để giảm nhiễu
    blurred = cv2.GaussianBlur(gray, CONFIG['kernel_sizes']['small'], 0)
    
    # Phân ngưỡng để tách đối tượng
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    # Tìm contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None, None
        
    # Lấy contour lớn nhất
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    # Kiểm tra diện tích tối thiểu
    if area < CONFIG['min_area']:
        return None, None, None
        
    # Tính centroid
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        centroid = (cx, cy)
    else:
        centroid = None
        
    return largest_contour, centroid, area

class BasketDetector:
    def __init__(self):
        """Khởi tạo detector với model đã train"""
        self.model = YOLO(CONFIG['model_path'])
        self.last_centroid_x = None
        self.last_centroid_y = None
        self.last_angle = None
    
    def get_last_centroid_x(self):
        """Trả về tọa độ x của centroid cuối cùng"""
        return self.last_centroid_x
        
    def get_last_centroid_y(self):
        """Trả về tọa độ y của centroid cuối cùng"""
        return self.last_centroid_y
        
    def get_last_angle(self):
        """Trả về góc cuối cùng đã tính"""
        return self.last_angle
    
    def process_frame(self, frame):
        """Xử lý một frame để phát hiện bảng rổ và rổ"""
        # Reset centroid và góc
        self.last_centroid_x = None
        self.last_centroid_y = None
        self.last_angle = None
        
        # Chạy model để phát hiện đối tượng
        results = self.model.predict(frame, conf=CONFIG['confidence'], verbose=False)
        
        # Sao chép frame để vẽ
        annotated_frame = results[0].plot()
        
        # Vẽ tâm camera và đường dọc
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        
        # Vẽ trục y đi qua tâm ảnh (kéo dài toàn bộ chiều cao)
        cv2.line(annotated_frame, (center_x, 0), (center_x, frame.shape[0]), (255, 255, 255), 1, cv2.LINE_AA)
        
        # Vẽ đường chữ thập ở tâm camera
        cv2.line(annotated_frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)  # Đường ngang
        cv2.line(annotated_frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)  # Đường dọc
        
        # Vẽ điểm tâm với 2 vòng tròn
        cv2.circle(annotated_frame, (center_x, center_y), 8, (0, 0, 255), -1)  # Điểm tâm đỏ
        cv2.circle(annotated_frame, (center_x, center_y), 12, (255, 255, 255), 2)
        
        # Duyệt qua tất cả các box được phát hiện
        for box in results[0].boxes:
            # Lấy tọa độ, class_id, và độ tin cậy
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            class_id = int(box.cls[0].item())
            
            # Xử lý bảng rổ
            if class_id == 0:  # Bảng rổ
                # Cắt và xử lý vùng bảng rổ
                roi = frame[y1:y2, x1:x2]
                contour, centroid, _ = process_roi(roi)
                
                if centroid is not None:
                    # Chuyển tọa độ về frame gốc
                    global_cx = x1 + centroid[0]
                    global_cy = y1 + centroid[1]
                    
                    # Lưu tọa độ centroid
                    self.last_centroid_x = global_cx
                    self.last_centroid_y = global_cy
                    
                    # Vẽ centroid và thông tin
                    cv2.circle(annotated_frame, (global_cx, global_cy), 5, CONFIG['colors']['centroid'], -1)
                    cv2.circle(annotated_frame, (global_cx, global_cy), 8, CONFIG['colors']['centroid_outline'], 2)
                    cv2.putText(annotated_frame, "Backboard Center", 
                              (global_cx + 10, global_cy), CONFIG['font'], CONFIG['font_scale'], 
                              CONFIG['colors']['centroid'], CONFIG['font_thickness'])
                    
                    # Vẽ đường nối từ tâm camera đến centroid
                    cv2.line(annotated_frame, (center_x, center_y), (global_cx, global_cy), (0, 255, 255), 2, cv2.LINE_AA)
                    
                    # Tính góc với trục y (0° đến ±180°)
                    dx = global_cx - center_x
                    dy = global_cy - center_y
                    angle = np.arctan2(dx, -dy) * 180 / np.pi  # Đổi dấu dy để 0° hướng lên
                    
                    # Lưu góc đã tính
                    self.last_angle = angle
                    
                    # Vẽ text góc với nền đen để dễ đọc
                    angle_text = f"+{angle:.1f}°" if angle > 0 else f"{angle:.1f}°"
                    text_size = cv2.getTextSize(angle_text, CONFIG['font'], CONFIG['font_scale'], CONFIG['font_thickness'])[0]
                    cv2.rectangle(annotated_frame, 
                                (center_x + 20, center_y - text_size[1] - 5),
                                (center_x + 20 + text_size[0] + 10, center_y + 5),
                                (0, 0, 0), -1)
                    cv2.putText(annotated_frame, angle_text,
                              (center_x + 25, center_y), CONFIG['font'],
                              CONFIG['font_scale'], (0, 255, 255), CONFIG['font_thickness'])
                    
                    # Vẽ contour
                    if contour is not None:
                        shifted_contour = contour.copy()
                        shifted_contour[:,:,0] += x1
                        shifted_contour[:,:,1] += y1
                        cv2.drawContours(annotated_frame, [shifted_contour], -1, CONFIG['colors']['contour'], 2)
            
            # Xử lý rổ
            elif class_id == 1:  # Rổ
                top_center_x = (x1 + x2) // 2
                top_center_y = y1
                cv2.circle(annotated_frame, (top_center_x, top_center_y), 5, CONFIG['colors']['hoop'], -1)
                cv2.putText(annotated_frame, "Hoop Top",
                          (top_center_x + 10, top_center_y), CONFIG['font'], CONFIG['font_scale'],
                          CONFIG['colors']['hoop'], CONFIG['font_thickness'])
        
        return annotated_frame 