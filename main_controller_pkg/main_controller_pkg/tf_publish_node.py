#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from geometry_msgs.msg import TransformStamped

class StraightenPointCloudNode(Node):
    def __init__(self):
        super().__init__('straighten_point_cloud_node')

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Create the static transform from camera_color_optical_frame to base_link.
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "camera_color_optical_frame"
        static_transformStamped.child_frame_id = "base_link"

        # Set translation [x, y, z] = [0, 0.2, 1.0]
        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = -1.0

        # Set rotation to identity (no rotation).
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0

        # Broadcast the static transform.
        self.broadcaster.sendTransform(static_transformStamped)
        self.get_logger().info("Published static transform from 'camera_color_optical_frame' to 'base_link'.")


        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',  # Input topic; adjust as needed.
            self.pointcloud_callback,
            10)
        self.publisher_ = self.create_publisher(PointCloud2, 'straight_point_cloud', 10)
        self.get_logger().info("Straighten Point Cloud Node started")
        # Set the rotation angle (in radians) used to "straighten" the cloud.
        self.camera_angle = 25
        self.rotation_angle = (-self.camera_angle + 70) * math.pi / 180.0  # Adjust as needed.

    def pointcloud_callback(self, msg):
        # Read points from the incoming message (x, y, z).
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        transformed_points = []

        # Create a 3x3 rotation matrix about the Z axis.
        theta = self.rotation_angle
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        # Rotation matrix about X-axis (Roll)
        R_x = np.array([[1,     0,           0    ],
                      [0,  cos_theta,  sin_theta],
                      [0, -sin_theta,  cos_theta]])
        
        # Rotation matrix about z-axis (Yaw)
        R_z = np.array([[0, 1, 0],
                       [-1,  0, 0],
                       [0,   0, 1]])
        
        R = R_z.dot(R_x)

        # Apply the rotation to each point.
        for pt in points:
            # Explicitly build a float array from the tuple elements.
            p = np.array([pt[0], pt[1], pt[2]], dtype=np.float32)
            p_new = R.dot(p)
            transformed_points.append((p_new[0], p_new[1], p_new[2]))

        # Create a new PointCloud2 message using the standard layout.
        new_msg = point_cloud2.create_cloud_xyz32(msg.header, transformed_points)
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StraightenPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()