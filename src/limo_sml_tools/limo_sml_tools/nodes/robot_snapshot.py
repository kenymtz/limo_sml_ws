#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class RobotSnapshot(Node):

    def __init__(self):
        super().__init__('robot_snapshot')

        self.pose = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info(
            "Robot snapshot iniciado. Esperando /amcl_pose..."
        )

    def pose_callback(self, msg):
        self.pose = msg


    def print_pose(self):

        if self.pose is None:
            print("AMCL pose: NO DISPONIBLE")
            return

        p = self.pose.pose.pose.position
        q = self.pose.pose.pose.orientation

        yaw = math.atan2(
            2.0*(q.w*q.z),
            1.0 - 2.0*(q.z*q.z)
        )

        print()
        print("=== AMCL POSE ===")
        print(f"x   : {p.x:.4f}")
        print(f"y   : {p.y:.4f}")
        print(
            f"yaw : {math.degrees(yaw):.2f} deg"
        )


def main():

    rclpy.init()

    node = RobotSnapshot()

    try:
        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=1.0
            )

            node.print_pose()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
