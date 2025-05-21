import rclpy
import math
from rclpy.node import Node
from robot_interfaces.srv import RequestCalculation
from robot_interfaces.srv import Control
#!/usr/bin/env python3

class FullCalculationNode(Node):
    def __init__(self):
        super().__init__('full_calculation_node')

        # Create a service called 'request_calculation'
        self.calculation_service = self.create_service(
            RequestCalculation,
            'request_calculation',
            self.request_calculation_callback
        )

        # Create a client for the 'control' service
        self.control_client = self.create_client(Control, 'control')

        self.get_logger().info('Full calculation node is ready.')

    def request_calculation_callback(self, request, response):
        self.get_logger().info('Received a calculation request.')

        velocity = self.calculate_initial_velocity(request.distance)
        self.get_logger().info('Calculated initial velocity: %f' % velocity)

        rps = int(self.calculate_rps(velocity))
        self.get_logger().info('Calculation velocity: %f' % rps)

        # Send the control request
        self.send_control_request(1, rps)
        self.get_logger().info('Control request sent.')

        # Set response fields appropriately.
        response.success = True  # or False based on your logic
        return response

    def send_control_request(self, action, velocity):
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
    # Calculate RPS from linear velocity and pulley diameter
    def calculate_rps(self, linear_velocity):
        """
        Calculate revolutions per second (RPS) from linear velocity and pulley diameter.

        Args:
            linear_velocity (float): Belt speed (m/s)
            diameter (float): Pulley diameter (mm)

        Returns:
            float: Required RPS
        """
        diameter = 54.5  # Pulley diameter in mm
        # Convert diameter from mm to meters
        diameter_m = diameter / 1000.0
        circumference = math.pi * diameter_m
        if circumference == 0:
            return 0
        return linear_velocity / circumference
    def calculate_initial_velocity(self,distance, angle_degrees=60, h0=1.0):
        """
        Calculate the initial velocity required to fire a ball such that it travels a given horizontal distance,
        when launched at a specific angle. This function assumes projectile motion under gravity.

        The equation used is derived from:
            target_height = h0 + distance*tan(theta) - (g*distance^2)/(2*v^2*cos^2(theta))
        which can be rearranged to solve for v:
            v = sqrt((g * distance^2) / (2*cos^2(theta)*(distance*tan(theta)+h0-target_height)))
        
        Args:
            distance (float): Horizontal distance to target (meters)
            angle_degrees (float): Launch angle (degrees)
            h0 (float, optional): Initial launch height (meters). Default is 0.0.
            target_height (float, optional): Target (landing) height (meters). Default is 0.0.
        
        Returns:
            float: Required initial velocity (m/s)
        
        Raises:
            ValueError: If the provided parameters result in an invalid trajectory.
        """
        target_height=2.43
        g = 9.81  # gravitational acceleration in m/s²
        theta = math.radians(angle_degrees)
        
        # Calculate denominator from rearranged projectile motion equation:
        # distance*tan(theta) + h0 - target_height must be positive for a valid trajectory.
        denominator = 2 * (distance * math.tan(theta) + h0 - target_height) * (math.cos(theta) ** 2)
        if denominator <= 0:
            raise ValueError("Invalid parameters: trajectory cannot be computed with these values.")
        
        v = math.sqrt((g * (distance ** 2)) / denominator)
        return v

def main(args=None):
    rclpy.init(args=args)
    node = FullCalculationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

