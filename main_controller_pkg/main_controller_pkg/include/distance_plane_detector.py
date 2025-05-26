#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from transforms3d.euler import euler2quat

class PlaneDetector:
    def __init__(self, distance_threshold=0.05, max_iterations=300, min_points=50, min_z=0.3, max_z=8.0):
        # Các tham số RANSAC
        self.distance_threshold = distance_threshold
        self.max_iterations = max_iterations
        self.min_points = min_points
        self.min_z = min_z
        self.max_z = max_z

        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
    def filter_points(self, pc_data):
        """Lọc điểm theo khoảng cách Z"""
        points_xyz = []
        for p in pc_data:
            if self.min_z < p.z < self.max_z:
                points_xyz.append([p.x, p.y, p.z])
        
        return np.array(points_xyz, dtype=np.float32) if len(points_xyz) >= self.min_points else None
    
    def process_point_cloud(self, pc_data):
        """Xử lý point cloud để phát hiện mặt phẳng"""
        # Lọc điểm
        points = self.filter_points(pc_data)
        if points is None:
            return None
        
        # Phát hiện mặt phẳng bằng RANSAC
        best_plane = self.ransac_plane_detection(points)
        if best_plane is None:
            return None
        
        # Trích xuất các điểm thuộc mặt phẳng
        plane_model, inliers = best_plane
        plane_points = points[inliers]
        
        # Phân tích hình học
        plane_info = self.analyze_plane_geometry(plane_points)
        
        # Tạo danh sách điểm mặt phẳng để publish
        plane_points_list = []
        for point in plane_points:
            plane_points_list.append([point[0], point[1], point[2]])
        
        return {
            'plane_info': plane_info,
            'plane_points': plane_points_list
        }
    
    def ransac_plane_detection(self, points):
        """Thuật toán RANSAC để tìm mặt phẳng lớn nhất trong point cloud"""
        n_points = len(points)
        if n_points < 3:
            return None
            
        best_inliers = []
        best_plane = None
        
        # RANSAC
        for _ in range(self.max_iterations):
            # Chọn ngẫu nhiên 3 điểm
            sample_indices = np.random.choice(n_points, 3, replace=False)
            p1, p2, p3 = points[sample_indices]
            
            # Tính vector pháp tuyến
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            
            if norm < 1e-6:  # Tránh chia cho 0
                continue
                
            normal = normal / norm
            d = -np.dot(normal, p1)
            
            # Tìm inliers
            distances = np.abs(np.dot(points, normal) + d)
            inliers = np.where(distances < self.distance_threshold)[0]
            
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_plane = (np.append(normal, d), inliers)
        
        # Kiểm tra số lượng inliers
        if best_plane is None or len(best_inliers) < self.min_points:
            return None
            
        return best_plane
    
    def analyze_plane_geometry(self, plane_points):
        """Phân tích hình học của mặt phẳng"""
        # Tính centroid
        centroid = np.mean(plane_points, axis=0)
        
        # Tính ma trận hiệp phương sai
        points_centered = plane_points - centroid
        cov_matrix = np.cov(points_centered.T)
        
        # Tìm vector riêng và giá trị riêng
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Sắp xếp
        idx = eigenvalues.argsort()
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Vector pháp tuyến
        normal = eigenvectors[:, 0]
        
        # Đảm bảo normal hướng về phía camera
        if normal[0] < 0:
            normal = normal
        
        # Các trục chính
        major_axis = eigenvectors[:, 2]
        minor_axis = eigenvectors[:, 1]
        
        # Kích thước
        major_length = 2.0 * np.sqrt(5.0 * eigenvalues[2])
        minor_length = 2.0 * np.sqrt(5.0 * eigenvalues[1])
        
        # Tính độ phẳng
        flatness = eigenvalues[0] / eigenvalues[1]
        
        # Góc với các trục
        self.angle_x = np.degrees(np.arccos(np.abs(np.dot(normal, [1, 0, 0]))))
        self.angle_y = np.degrees(np.arccos(np.abs(np.dot(normal, [0, 1, 0]))))
        self.angle_z = np.degrees(np.arccos(np.abs(np.dot(normal, [0, 0, 1]))))

        print(f"Plane angles: X={self.angle_x:.2f}, Y={self.angle_y:.2f}, Z={self.angle_z:.2f}")
        
        return {
            'centroid': centroid,
            'normal': normal,
            'dimensions': (major_length, minor_length),
            'axes': (major_axis, minor_axis),
            'flatness': flatness,
            'angles': (self.angle_x, self.angle_y, self.angle_z)
        }
    
    def create_plane_markers(self, plane_info, header):
        """Tạo markers để hiển thị thông tin về mặt phẳng"""
        marker_array = MarkerArray()
        
        # 1. Marker cho tâm (centroid)
        centroid_marker = Marker()
        centroid_marker.header = header
        centroid_marker.ns = "plane_centroid"
        centroid_marker.id = 0
        centroid_marker.type = Marker.SPHERE
        centroid_marker.action = Marker.ADD
        centroid_marker.pose.position.x = float(plane_info['centroid'][0])
        centroid_marker.pose.position.y = float(plane_info['centroid'][1])
        centroid_marker.pose.position.z = float(plane_info['centroid'][2])
        centroid_marker.pose.orientation.w = 1.0
        centroid_marker.scale.x = 0.1
        centroid_marker.scale.y = 0.1
        centroid_marker.scale.z = 0.1
        centroid_marker.color.r = 1.0  # Màu đỏ
        centroid_marker.color.g = 0.0
        centroid_marker.color.b = 0.0
        centroid_marker.color.a = 1.0
        centroid_marker.lifetime.sec = 1
        marker_array.markers.append(centroid_marker)
        
        # 2. Marker cho vector pháp tuyến
        normal_marker = Marker()
        normal_marker.header = header
        normal_marker.ns = "plane_normal"
        normal_marker.id = 0
        normal_marker.type = Marker.ARROW
        normal_marker.action = Marker.ADD
        
        # Sử dụng points
        start_point = Point()
        start_point.x = float(plane_info['centroid'][0])
        start_point.y = float(plane_info['centroid'][1])
        start_point.z = float(plane_info['centroid'][2])
        normal_marker.points.append(start_point)
        
        normal = plane_info['normal']
        arrow_length = 0.5
        end_point = Point()
        end_point.x = float(plane_info['centroid'][0] + normal[0] * arrow_length)
        end_point.y = float(plane_info['centroid'][1] + normal[1] * arrow_length)
        end_point.z = float(plane_info['centroid'][2] + normal[2] * arrow_length)
        normal_marker.points.append(end_point)
        
        normal_marker.scale.x = 0.02
        normal_marker.scale.y = 0.05
        normal_marker.scale.z = 0.05
        normal_marker.color.r = 0.0
        normal_marker.color.g = 0.0
        normal_marker.color.b = 1.0
        normal_marker.color.a = 1.0
        normal_marker.lifetime.sec = 1
        marker_array.markers.append(normal_marker)
        
        # 3. Marker cho hình chữ nhật
        rect_marker = Marker()
        rect_marker.header = header
        rect_marker.ns = "plane_rectangle"
        rect_marker.id = 0
        rect_marker.type = Marker.LINE_STRIP
        rect_marker.action = Marker.ADD
        
        centroid = plane_info['centroid']
        major_axis = plane_info['axes'][0]
        minor_axis = plane_info['axes'][1]
        major_length = plane_info['dimensions'][0] / 2.0
        minor_length = plane_info['dimensions'][1] / 2.0
        
        corners = [
            centroid + major_axis * major_length + minor_axis * minor_length,
            centroid + major_axis * major_length - minor_axis * minor_length,
            centroid - major_axis * major_length - minor_axis * minor_length,
            centroid - major_axis * major_length + minor_axis * minor_length,
            centroid + major_axis * major_length + minor_axis * minor_length
        ]
        
        for corner in corners:
            point = Point()
            point.x = float(corner[0])
            point.y = float(corner[1])
            point.z = float(corner[2])
            rect_marker.points.append(point)
        
        rect_marker.scale.x = 0.02
        rect_marker.color.r = 0.0
        rect_marker.color.g = 1.0
        rect_marker.color.b = 0.0
        rect_marker.color.a = 1.0
        rect_marker.lifetime.sec = 1
        marker_array.markers.append(rect_marker)
        
        # 4. Bề mặt mặt phẳng
        surface_marker = Marker()
        surface_marker.header = header
        surface_marker.ns = "plane_surface"
        surface_marker.id = 0
        surface_marker.type = Marker.TRIANGLE_LIST
        surface_marker.action = Marker.ADD
        
        for i in [0, 1, 2, 0, 2, 3]:
            point = Point()
            point.x = float(corners[i][0])
            point.y = float(corners[i][1])
            point.z = float(corners[i][2])
            surface_marker.points.append(point)
            
        surface_marker.scale.x = 1.0
        surface_marker.scale.y = 1.0
        surface_marker.scale.z = 1.0
        surface_marker.color.r = 0.0
        surface_marker.color.g = 0.8
        surface_marker.color.b = 0.2
        surface_marker.color.a = 0.3
        surface_marker.lifetime.sec = 1
        marker_array.markers.append(surface_marker)
        
        # 5. Marker text
        text_marker = Marker()
        text_marker.header = header
        text_marker.ns = "plane_info"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(plane_info['centroid'][0])
        text_marker.pose.position.y = float(plane_info['centroid'][1])
        text_marker.pose.position.z = float(plane_info['centroid'][2] + 0.2)
        text_marker.pose.orientation.w = 1.0
        
        info_text = f"Size: {plane_info['dimensions'][0]:.2f}m x {plane_info['dimensions'][1]:.2f}m\n"
        info_text += f"Normal: [{plane_info['normal'][0]:.2f}, {plane_info['normal'][1]:.2f}, {plane_info['normal'][2]:.2f}]\n"
        info_text += f"Angles: X={plane_info['angles'][0]:.0f}°, Y={plane_info['angles'][1]:.0f}°, Z={plane_info['angles'][2]:.0f}°"
        
        text_marker.text = info_text
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.lifetime.sec = 1
        marker_array.markers.append(text_marker)
        
        return marker_array
    
    def create_distance_markers(self, centroid_point, header, target_frame='camera_link', transform_func=None):
        """Tạo marker đường thẳng từ camera_link đến centroid và hiển thị khoảng cách"""
        marker_array = MarkerArray()
        
        # Nếu có hàm transform thì thực hiện transform
        centroid_in_camera_link = centroid_point
        if transform_func and header.frame_id != target_frame:
            try:
                transformed_pose = transform_func(centroid_point, header.frame_id, target_frame)
                if transformed_pose:
                    centroid_in_camera_link = np.array([
                        transformed_pose.pose.position.x,
                        transformed_pose.pose.position.y,
                        transformed_pose.pose.position.z
                    ])
            except Exception:
                # Sử dụng giá trị gốc nếu transform thất bại
                pass
        
        # Đảm bảo giá trị là numpy array
        if not isinstance(centroid_in_camera_link, np.ndarray):
            centroid_in_camera_link = np.array(centroid_in_camera_link)
        
        # 1. Marker đường thẳng
        line_marker = Marker()
        line_marker.header.frame_id = target_frame
        line_marker.header.stamp = header.stamp
        line_marker.ns = "distance_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # Điểm đầu ở camera_link
        base_point = Point()
        base_point.x = 0.0
        base_point.y = 0.0
        base_point.z = 0.0
        line_marker.points.append(base_point)
        
        # Điểm cuối là centroid
        centroid_point = Point()
        centroid_point.x = float(centroid_in_camera_link[0])
        centroid_point.y = float(centroid_in_camera_link[1])
        centroid_point.z = float(centroid_in_camera_link[2])
        line_marker.points.append(centroid_point)
        
        line_marker.scale.x = 0.02
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        line_marker.lifetime.sec = 1
        marker_array.markers.append(line_marker)
        
        # 2. Tính khoảng cách
        distance = np.linalg.norm(centroid_in_camera_link)
        
        # 3. Marker text
        distance_marker = Marker()
        distance_marker.header.frame_id = target_frame
        distance_marker.header.stamp = header.stamp
        distance_marker.ns = "distance_text"
        distance_marker.id = 0
        distance_marker.type = Marker.TEXT_VIEW_FACING
        distance_marker.action = Marker.ADD
        
        distance_marker.pose.position.x = float(centroid_in_camera_link[0] / 2)
        distance_marker.pose.position.y = float(centroid_in_camera_link[1] / 2)
        distance_marker.pose.position.z = float(centroid_in_camera_link[2] / 2 + 0.1)
        distance_marker.pose.orientation.w = 1.0
        
        distance_marker.text = f"Distance: {distance:.2f} m"
        distance_marker.scale.z = 0.1
        distance_marker.color.r = 1.0
        distance_marker.color.g = 1.0
        distance_marker.color.b = 0.0
        distance_marker.color.a = 1.0
        distance_marker.lifetime.sec = 1
        marker_array.markers.append(distance_marker)
        
        return marker_array, distance
    
    def calculate_transform_from_plane(self, plane_info, header):
        """Tính transform từ camera frame đến plane centroid frame"""
        # Vector pháp tuyến
        normal = np.array(plane_info['normal'])
        
        # Vector z chuẩn
        z_axis = np.array([0, 0, 1])
        
        # Tính trục quay
        rotation_axis = np.cross(z_axis, normal)
        rotation_axis_norm = np.linalg.norm(rotation_axis)
        
        if rotation_axis_norm < 1e-6:
            # Gần như song song với z
            q = [0, 0, 0, 1]
        else:
            # Chuẩn hóa trục quay
            rotation_axis = rotation_axis / rotation_axis_norm
            
            # Tính góc quay
            angle = np.arccos(np.dot(z_axis, normal))
            
            # Chuyển sang quaternion
            q = euler2quat(
                rotation_axis[0] * angle,
                rotation_axis[1] * angle,
                rotation_axis[2] * angle,
                'sxyz'
            )
        
        return {
            'frame_id': header.frame_id,
            'child_frame_id': "plane_centroid",
            'translation': plane_info['centroid'],
            'rotation': q  # [w, x, y, z]
        } 