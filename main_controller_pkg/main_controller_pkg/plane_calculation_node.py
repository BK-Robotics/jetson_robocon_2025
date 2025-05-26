#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs_py import point_cloud2
import tf_transformations
import numpy as np
import time
# Import module tùy chỉnh
from main_controller_pkg.include.distance_plane_detector import PlaneDetector
from robot_interfaces.srv import RequestDistance

class PlaneDetectionNode(Node):
    def __init__(self):
        super().__init__('plane_detection_node')
        
        # Khai báo tham số
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('max_iterations', 300)
        self.declare_parameter('min_points', 50)
        self.declare_parameter('min_z', 0.3)
        self.declare_parameter('max_z', 8.0)
        
        # Lấy giá trị tham số
        distance_threshold = self.get_parameter('distance_threshold').value
        max_iterations = self.get_parameter('max_iterations').value
        min_points = self.get_parameter('min_points').value
        min_z = self.get_parameter('min_z').value
        max_z = self.get_parameter('max_z').value
        
        # Khởi tạo PlaneDetector
        self.detector = PlaneDetector(
            distance_threshold=distance_threshold,
            max_iterations=max_iterations,
            min_points=min_points,
            min_z=min_z,
            max_z=max_z
        )
        
        # Input và output topics
        self.input_topic = '/straight_point_cloud'
        self.output_plane_topic = '/plane_cloud'
        self.output_markers_topic = '/plane_markers'
        self.output_distance_topic = '/distance_markers'
        
        # Publishers
        self.plane_publisher = self.create_publisher(
            PointCloud2,
            self.output_plane_topic,
            10)
        
        self.markers_publisher = self.create_publisher(
            MarkerArray,
            self.output_markers_topic,
            10)
        
        self.distance_publisher = self.create_publisher(
            MarkerArray,
            self.output_distance_topic,
            10)
        
        # Service server
        self.point_cloud_service = self.create_service(
            RequestDistance,
            '/request_point_cloud_calculation',
            self.request_calculation_callback
        )
        
        self.get_logger().info('Plane Detection Node initialized /straight_point_cloud !')
    
    def transform_point(self, point, from_frame, to_frame):
        """Chuyển đổi điểm từ frame này sang frame khác"""
        try:
            # Tạo pose từ điểm
            pose = PoseStamped()
            pose.header.frame_id = from_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            
            # Lookup transform
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            
            # Áp dụng transform
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            return transformed_pose
            
        except Exception as e:
            self.get_logger().warn(f'Transform error: {str(e)}')
            return None
    
    # Hàm tính toán ra distance khi nhận thông tin từ topic
    def point_cloud_callback(self, msg):
        """Callback khi nhận point cloud"""
        try:
            start_time = time.time()
            
            # Đọc point cloud
            pc_data = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
            
            # Xử lý point cloud
            result = self.detector.process_point_cloud(pc_data)
            if result is None:
                self.get_logger().warn('No plane detected')
                return
            
            # Lấy thông tin mặt phẳng
            plane_info = result['plane_info']
            plane_points = result['plane_points']
            
            # Xuất bản point cloud mặt phẳng
            plane_cloud_msg = point_cloud2.create_cloud_xyz32(msg.header, plane_points)
            self.plane_publisher.publish(plane_cloud_msg)
            
            # Xuất bản markers
            markers = self.detector.create_plane_markers(plane_info, msg.header)
            self.markers_publisher.publish(markers)
            
            # Tính khoảng cách và xuất bản markers
            distance_markers, distance = self.detector.create_distance_markers(
                plane_info['centroid'], 
                msg.header,
                transform_func=self.transform_point
            )
            self.distance_publisher.publish(distance_markers)
            
            # Tính và broadcast transform
            transform_data = self.detector.calculate_transform_from_plane(plane_info, msg.header)
            self.broadcast_transform(transform_data)
            
            
            processing_time = time.time() - start_time
            self.get_logger().info(f'Plane detected. Distance: {distance:.2f}m. Processing time: {processing_time:.3f}s')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())


    def broadcast_transform(self, transform_data):
        """Broadcast transform from camera to plane centroid with a modifiable rotation matrix."""
        t = TransformStamped()
        
        # Set header and frames
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = transform_data['frame_id']
        t.child_frame_id = transform_data['child_frame_id']
        
        # Set translation (from transform_data)
        t.transform.translation.x = float(transform_data['translation'][0])
        t.transform.translation.y = float(transform_data['translation'][1])
        t.transform.translation.z = float(transform_data['translation'][2])
        

        # Define the Euler angles in degrees (modify these values as needed)
        roll_angle = self.detector.angle_x    # rotation about X-axis in degrees
        pitch_angle = self.detector.angle_y   # rotation about Y-axis in degrees
        yaw_angle = self.detector.angle_z    # rotation about Z-axis in degrees

        # Convert angles to radians
        roll = np.deg2rad(roll_angle)
        pitch = np.deg2rad(pitch_angle)
        yaw = np.deg2rad(yaw_angle)

        # Rotation matrix about X-axis (Roll)
        R_x = np.array([[1,            0,           0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll),  np.cos(roll)]], dtype=np.float32)

        # Rotation matrix about Y-axis (Pitch)
        R_y = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                        [             0, 1,             0],
                        [-np.sin(pitch), 0, np.cos(pitch)]], dtype=np.float32)

        # Rotation matrix about Z-axis (Yaw)
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw),  np.cos(yaw), 0],
                        [          0,            0, 1]], dtype=np.float32)

        # Multiply the matrices in the order R = R_z * R_y * R_x.
        R = R_z @ R_y @ R_x
 
                    
        H = np.eye(4, dtype=np.float32)
        H[:3, :3] = R
        quat = tf_transformations.quaternion_from_matrix(H)  # Returns quaternion in [x, y, z, w] order.
        
        # Set the rotation in the TransformStamped message.
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        # Broadcast the transform.
        self.tf_broadcaster.sendTransform(t)
    
    def request_calculation_callback(self, request, response):
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.point_cloud_callback,
            10)
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('Received request for point cloud calculation')
        return response  # Return the response to acknowledge the service request

def main(args=None):
    rclpy.init(args=args)
    node = PlaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 