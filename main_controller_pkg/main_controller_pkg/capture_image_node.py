import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.save_interval = 2.5  # seconds
        self.image_folder = 'captured_images'
        os.makedirs(self.image_folder, exist_ok=True)
        self.current_image = None
        self.timer = self.create_timer(self.save_interval, self.save_image_timer_callback)
        self.image_count = self.get_start_count()
        self.get_logger().info('Image capture node started.')

    def get_start_count(self):
        # Find the max count of existing images to continue counting
        existing = [f for f in os.listdir(self.image_folder) if f.endswith('.jpg')]
        if not existing:
            return 1
        nums = []
        for f in existing:
            try:
                num = int(f.split('.')[0])
                nums.append(num)
            except Exception:
                continue
        return max(nums) + 1 if nums else 1

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def save_image_timer_callback(self):
        if self.current_image is not None:
            filename = os.path.join(self.image_folder, f'{self.image_count}.jpg')
            cv2.imwrite(filename, self.current_image)
            self.get_logger().info(f'Saved image: {filename}')
            self.image_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()