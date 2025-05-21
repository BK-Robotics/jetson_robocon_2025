import rclpy
from rclpy.node import Node
from robot_interfaces.msg import IMU
from robot_interfaces.srv import Control
from robot_interfaces.srv import RequestCalculation
from robot_interfaces.srv import RotateBase
from robot_interfaces.srv import RequestAction

class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        self.imu_angle = 0.0
        self.action = 0
        self.distance = 6.15
        self.request_angle = 0.0
        self.base_mode = 0

        self.action_srv = self.create_service(RequestAction, 'request_action', self.request_action_callback)
        self.control_client = self.create_client(Control, 'control')
        self.request_calculation_client = self.create_client(RequestCalculation, 'request_calculation')
        self.rotate_base_client = self.create_client(RotateBase, 'rotate_base')
        self.imu_subscriber = self.create_subscription(
            IMU,
            'imu',
            self.imu_callback,
            10
        )
        self.get_logger().info('Main controller node is ready.')

    def imu_callback(self, msg):
        self.imu_angle = msg.angle
        self.get_logger().info('IMU data received: %f' % self.imu_angle)
    
    def request_action_callback(self, request, response):
        self.get_logger().info('Received action request: %d' % request.action)
        success = False
        if request.action == 1:
            # Main rotate base and distance calculation
            # self.distance = self.shooting_distance_process()
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

    def shooting_distance_process(self):
        calculated_distance = 0.0
        
        return calculated_distance
    
def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()