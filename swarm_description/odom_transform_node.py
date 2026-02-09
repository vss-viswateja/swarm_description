#!/usr/bin/env python3
"""
Odometry Transform Node

This node subscribes to odometry data published by Gazebo (in gz_world frame)
and transforms it to the odom frame, publishing both the transformed odometry
and the odom -> base_link TF transform.

Static Transform (gz_world -> odom):
    x: 0.0, y: -3.0, z: 0.213, roll: 0, pitch: 0, yaw: 0

Transform Logic:
    P_odom = P_gz_world - offset
    Where offset = (0.0, -3.0, 0.213)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTransformNode(Node):
    """
    Node that transforms odometry from Gazebo world frame to odom frame.
    """

    def __init__(self):
        super().__init__('odom_transform_node')

        # Declare parameters for the transform offset (gz_world -> odom)
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', -3.0)
        self.declare_parameter('offset_z', 0.213)
        self.declare_parameter('input_topic', 'odom_gz')
        self.declare_parameter('output_topic', 'odometry/filtered')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Get parameters
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Log configuration
        self.get_logger().info(f'Odom Transform Node initialized')
        self.get_logger().info(f'  Offset: ({self.offset_x}, {self.offset_y}, {self.offset_z})')
        self.get_logger().info(f'  Input topic: {input_topic}')
        self.get_logger().info(f'  Output topic: {output_topic}')
        self.get_logger().info(f'  Odom frame: {self.odom_frame}')
        self.get_logger().info(f'  Base frame: {self.base_frame}')

        # Create subscriber for Gazebo odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            10
        )

        # Create publisher for transformed odometry
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)

        # Create TF broadcaster for odom -> base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        """
        Callback to process incoming Gazebo odometry and publish transformed odometry.

        Args:
            msg: Incoming Odometry message in gz_world frame
        """
        # Create transformed odometry message
        transformed_odom = Odometry()

        # Copy header and update frame_id
        transformed_odom.header = msg.header
        transformed_odom.header.frame_id = self.odom_frame
        transformed_odom.child_frame_id = self.base_frame

        # Transform position: P_odom = P_gz_world - offset
        # Since odom is at (0, -3, 0.213) in gz_world, we subtract the offset
        transformed_odom.pose.pose.position.x = msg.pose.pose.position.x - self.offset_x
        transformed_odom.pose.pose.position.y = msg.pose.pose.position.y - self.offset_y
        transformed_odom.pose.pose.position.z = msg.pose.pose.position.z - self.offset_z

        # Orientation remains the same (no rotation between frames)
        transformed_odom.pose.pose.orientation = msg.pose.pose.orientation

        # Copy pose covariance
        transformed_odom.pose.covariance = msg.pose.covariance

        # Copy twist (velocities are frame-independent for aligned frames)
        transformed_odom.twist = msg.twist

        # Publish transformed odometry
        self.odom_pub.publish(transformed_odom)

        # Publish TF transform (odom -> base_link)
        self.publish_tf(transformed_odom)

    def publish_tf(self, odom_msg: Odometry):
        """
        Publish the odom -> base_link transform to TF.

        Args:
            odom_msg: Transformed Odometry message
        """
        tf_msg = TransformStamped()

        tf_msg.header = odom_msg.header
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        # Set translation from odometry pose
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z

        # Set rotation from odometry pose
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
