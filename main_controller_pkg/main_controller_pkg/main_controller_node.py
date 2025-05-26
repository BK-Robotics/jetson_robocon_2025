import rclpy
from rclpy.node import Node
from robot_interfaces.msg import IMU
from robot_interfaces.srv import Control
from robot_interfaces.srv import RequestCalculation
from robot_interfaces.srv import RotateBase
from robot_interfaces.srv import RequestAction
from robot_interfaces.srv import RequestDistance
from .include.basket_detect import BasketDetector
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
import cv2

class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')

        self.cv_image = None
        self.imu_angle = 0.0
        self.action = 0
        self.distance = 6.15
        self.detected_angle = 0.0
        self.base_mode = 0

        # Khởi tạo detector
        self.detector = BasketDetector()
        self.cv_bridge = CvBridge()

        self.action_srv = self.create_service(RequestAction, 'request_action', self.request_action_callback)
        self.control_client = self.create_client(Control, 'control')
        self.request_calculation_client = self.create_client(RequestCalculation, 'request_calculation')
        self.rotate_base_client = self.create_client(RotateBase, 'rotate_base')
        self.point_cloud_client = self.create_client(RequestDistance, '/request_point_cloud_calculation')

        self.imu_subscriber = self.create_subscription(
            IMU,
            'imu',
            self.imu_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.process_image = self.create_publisher(
            Image,
            '/processed_image',
            10
        )

        self.get_logger().info('Main controller node is ready.')

    def imu_callback(self, msg):
        self.imu_angle = msg.angle
        self.get_logger().info('IMU data received: %f' % self.imu_angle)
    
    def image_callback(self, msg):
        # Change ROS Image message to OpenCV format
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the image using OpenCV
        # cv2.imshow("Camera Image", self.cv_image)
        # cv2.waitKey(1)
        self.get_logger().info('Image received and displayed.')


    def request_action_callback(self, request, response):
        self.get_logger().info('Received action request: %d' % request.action)
        success = False
        if request.action == 1:
            # Main rotate base and distance calculation
            # self.image_process_timer = self.create_timer(0.1, self.image_process_timer_callback)
            # self.distance = self.send_request_point_cloud()

            # if self.detected_angle < 1:
            #     self.image_process_timer.destroy()
            
            self.send_request_calculation(self.distance)
        elif request.action >= 5:
            self.base_mode = request.action - 5
        else:
            self.action = request.action

            # Call the control client as part of the action process
            self.send_control_request(self.action)
        
        # Set response fields appropriately.
        response.success = success  # or False based on your logic
        return response
    
    def send_request_point_cloud(self):
        if not self.point_cloud_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Point cloud service not available, exiting...')
            return -1
        
        request = RequestDistance.Request()
        request.wait_for_completion = 0

        future = self.point_cloud_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2)
        if future.result() is not None:
            self.get_logger().info('Point cloud request sent successfully.')
            return future.result()
        else:
            self.get_logger().warn('No response from point cloud service: %r' % future.exception())
            return -1

    def send_control_request(self, action, velocity = 0):
        if not self.control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Control service not available, exiting...')
            return
        request = Control.Request()
        request.action = action
        request.velocity = velocity
        future = self.control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if future.result() is not None:
            self.get_logger().info('Feedback %d' % action)
        else:
            self.get_logger().warn('No response from control service: %r' % future.exception())

    def send_request_calculation(self, distance):
        if not self.request_calculation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Control service not available, exiting...')
            return
        request = RequestCalculation.Request()
        request.distance = distance
        future = self.request_calculation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if future.result() is not None:
            self.get_logger().info('Feedback %d' % distance)
        else:
            self.get_logger().warn('No response from control service: %r' % future.exception())
    
    def send_rotate_base_request(self, angle):
        if not self.rotate_base_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('Control service not available, exiting...')
            return
        request = RotateBase.Request()
        request.angle = angle
        future = self.rotate_base_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if future.result() is not None:
            self.get_logger().info('Feedback %d' % angle)
        else:
            self.get_logger().warn('No response from control service: %r' % future.exception())

    def image_process_timer_callback(self):
        try:
            # Xử lý frame để phát hiện bảng rổ
            image = self.detector.process_frame(self.cv_image)
            self.process_image.publish(image)
            self.get_logger().info('Publish processed image.')

            # Lấy góc phát hiện được từ detector
            self.detected_angle = self.detector.get_last_angle()
            
            if self.detected_angle is not None:
                # Tính tổng góc cần xoay (góc phát hiện + góc hiện tại của robot)
                total_angle = self.detected_angle + self.imu_angle
                
                # Gửi request tới service /rotate_base với góc tổng
                self.send_rotate_request(total_angle)
                
                # Log thông tin
                self.get_logger().info(f'Detedted angle: {self.detected_angle:.2f}°, ' +
                                     f'IMU angle: {self.imu_angle:.2f}°, ' +
                                     f'Sum angle: {total_angle:.2f}°')
                
        except Exception as e:
            self.get_logger().error(f'Image process error: {str(e)}')
    
def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
